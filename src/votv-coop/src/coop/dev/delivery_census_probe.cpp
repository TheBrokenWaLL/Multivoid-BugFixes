// coop/dev/delivery_census_probe.cpp -- COUNT the delivery-path actors over time. Diagnostic only,
// ini-gated off by default ([dev] delivery_census=1); it never ships behaviour.
//
// The lane it exists for synced the wrong actor: a drone's BeginPlay resolves its container by the
// save key n'droneContainer', which occurs in exactly one cooked asset -- the drone's own -- while
// the world-placed container is keyed 'drone_InventoryContainer'. That lookup is an exact
// Array_Find over keyObj_key whose only writer is lib_C::assignKey, so it misses, the drone spawns
// its own container, and compileOrder writes through (drone.container)->propInventory. The probe
// therefore enumerates BY CLASS, never by key, and answers three questions with a count or a
// pointer comparison rather than a hypothesis: Q-A how many containers exist (the row count per
// class, per sample); Q-B which one receives a delivery (the per-row contents count watched across
// one); Q-C whether that receiver is stable across two deliveries or churns. A full census is
// logged on the first sample and thereafter only when a hash over the rows changes, so the log
// shows edges -- a spawn, a destroy, a contents delta, an Index repoint -- rather than a wall of
// identical rows.

#include "coop/dev/delivery_census_probe.h"

#include "coop/element/registry.h"
#include "coop/player/players_registry.h"
#include "coop/props/remote_prop.h"
#include "coop/net/session.h"
#include "coop/config/config.h"
#include "ue_wrap/actors/inventory.h"    // ResolveSaveSlot
#include "ue_wrap/actors/prop.h"         // GetInteractableKeyString
#include "ue_wrap/engine/engine.h"       // TryGetActorLocation
#include "ue_wrap/actors/save_record.h"  // Arr / ReadArr / kMxStride
#include "ue_wrap/core/log.h"
#include "ue_wrap/core/reflection.h"
#include "ue_wrap/core/sdk_profile.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace coop::dev::delivery_census {
namespace {

namespace R  = ue_wrap::reflection;
namespace SR = ue_wrap::save_record;
namespace E  = ue_wrap::engine;
namespace P  = ue_wrap::profile;

constexpr uint64_t kSampleMs = 2000;

uint64_t NowMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}  // dev probe; a GUObjectArray walk is not free

uint64_t g_nextSample = 0;
uint64_t g_lastHash = 0;
uint64_t g_lastSackDiagHash = 0;
bool g_sackDiagFirst = true;

// Per-GObjStack-slot occupancy from the previous sample. The census answered Q-A and Q-B but
// exposed a fact it could not explain: on the host a delivery landed (slot count 0 -> 2) and the
// SAME slot went back to 0 within eight seconds, while the player-facing sack was never observed as
// an actor at all. So the contents LEAVE the container we sync. This tracks EVERY slot's count and
// reports the deltas, which turns "where did they go" into a direct reading -- one slot loses N,
// another gains N -- instead of another inference.
std::vector<int32_t> g_prevSlots;
bool     g_first = true;
int      g_sampleNo = 0;

template <class T> T ReadAt(const void* base, int32_t off) {
    T v{};
    std::memcpy(&v, reinterpret_cast<const uint8_t*>(base) + off, sizeof(T));
    return v;
}

int32_t Off(void* cls, const wchar_t* name) {
    return cls ? R::FindPropertyOffset(cls, name) : -1;
}

// One census row -- everything needed to answer Q-A/Q-B/Q-C without inferring anything.
struct Row {
    void*        actor = nullptr;
    std::wstring cls;
    std::wstring key;          // the save Key, or empty
    uint32_t     eid = 0;      // 0 = not enrolled as an Element
    void*        inv = nullptr;
    int32_t      index = -1;   // propInventory.Index -- the GObjStack slot
    float        currVol = 0.f;
    bool         player = false;
    int32_t      contents = -1;  // GObjStack[index].obj.Num, or -1 if unresolvable
    float        x = 0, y = 0, z = 0;
    bool         locRead = false;   // false: x/y/z are not a position
    bool         isDroneContainer = false;  // == drone.container (what compileOrder writes through)
    bool         isSackContainer = false;   // == some sack.container
};

// The contents count for a propInventory, read straight out of the global GObjStack.
int32_t ContentsOf(void* inv, int32_t index) {
    if (!inv || index < 0) return -1;
    void* save = ue_wrap::inventory::ResolveSaveSlot();
    if (!save) return -1;
    const int32_t offStack = Off(R::ClassOf(save), L"GObjStack");
    if (offStack < 0) return -1;
    const SR::Arr stack = SR::ReadArr(save, offStack);
    if (index >= stack.num) return -1;
    const uint8_t* slot = stack.data + static_cast<size_t>(index) * SR::kMxStride;
    return SR::ReadArr(slot, 0).num;  // struct_mObject.obj @ +0
}

void FillInventory(Row& r) {
    const int32_t offInv = Off(R::ClassOf(r.actor), L"propInventory");
    if (offInv < 0) return;
    r.inv = ReadAt<void*>(r.actor, offInv);
    if (!r.inv || !R::IsLive(r.inv)) { r.inv = nullptr; return; }
    void* icls = R::ClassOf(r.inv);
    const int32_t oi = Off(icls, L"Index"), ov = Off(icls, L"currVol"), op = Off(icls, L"Player");
    if (oi >= 0) r.index   = ReadAt<int32_t>(r.inv, oi);
    if (ov >= 0) r.currVol = ReadAt<float>(r.inv, ov);
    if (op >= 0) r.player  = ReadAt<uint8_t>(r.inv, op) != 0;
    r.contents = ContentsOf(r.inv, r.index);
}

void FillCommon(Row& r) {
    r.cls = R::ClassNameOf(r.actor);
    r.key = ue_wrap::prop::GetInteractableKeyString(r.actor);
    const auto id = coop::element::Registry::Get().EidForActor(r.actor);
    r.eid = (id == coop::element::kInvalidId) ? 0u : static_cast<uint32_t>(id);
    ue_wrap::FVector loc{};
    r.locRead = ue_wrap::engine::TryGetActorLocation(r.actor, loc);
    r.x = loc.X; r.y = loc.Y; r.z = loc.Z;
}


// The sack visual can survive as a component of drone_C rather than as a second
// prop_dronesack_C actor. This read-only census distinguishes those cases before
// another actor-destruction rule is attempted.
uint64_t MixDiag(uint64_t h, uint64_t value) {
    h ^= value;
    return h * 1099511628211ull;
}

uint64_t MixText(uint64_t h, const std::wstring& value) {
    for (wchar_t ch : value) h = MixDiag(h, static_cast<uint64_t>(ch));
    return MixDiag(h, value.size());
}

struct ComponentDiag {
    void* object = nullptr;
    std::wstring name;
    std::wstring cls;
    void* attachParent = nullptr;
    void* parentOwner = nullptr;
    void* meshAsset = nullptr;
    std::wstring meshName;
    void* childActor = nullptr;
    std::wstring childActorName;
    std::wstring childActorClass;
    uint8_t visByte = 0;
    uint8_t hiddenByte = 0;
};

struct DroneDiag {
    void* actor = nullptr;
    std::wstring name;
    void* container = nullptr;
    bool hasSack = false;
    bool canTakeOff = false;
    std::vector<ComponentDiag> components;
};

struct SackDiag {
    void* actor = nullptr;
    std::wstring name;
    std::wstring cls;
    std::wstring key;
    void* container = nullptr;
    void* parent = nullptr;
    bool parentRead = false;
    bool taken = false;
    uint32_t eid = 0;
    uint32_t mirrorEid = 0;
    void* root = nullptr;
    void* rootAttachParent = nullptr;
    void* rootParentOwner = nullptr;
    uint8_t rootVisByte = 0;
    uint8_t rootHiddenByte = 0;
    int rootCollision = -1;
    bool simulating = false;
    bool locRead = false;
    ue_wrap::FVector loc{};
    bool heldByLocal = false;
};

void EmitDroneSackDiagnostic(bool isHost, const std::vector<void*>& drones,
                             const std::vector<void*>& sacks) {
    std::vector<DroneDiag> droneRows;
    std::vector<SackDiag> sackRows;
    const int32_t sceneParentOff = static_cast<int32_t>(P::off::USceneComponent_AttachParent);
    void* sceneBase = R::FindClass(L"SceneComponent");
    void* sceneBases[] = {sceneBase};

    uint64_t hash = 1469598103934665603ull;
    hash = MixDiag(hash, isHost ? 1u : 0u);
    hash = MixDiag(hash, drones.size());
    hash = MixDiag(hash, sacks.size());

    for (void* drone : drones) {
        if (!drone || !R::IsLive(drone)) continue;
        DroneDiag d;
        d.actor = drone;
        d.name = R::ToString(R::NameOf(drone));
        void* cls = R::ClassOf(drone);
        const int32_t offContainer = Off(cls, L"container");
        const int32_t offHasSack = Off(cls, L"hasSack");
        const int32_t offCanTakeOff = Off(cls, L"canTakeOff");
        if (offContainer >= 0) d.container = ReadAt<void*>(drone, offContainer);
        if (offHasSack >= 0) d.hasSack = ReadAt<uint8_t>(drone, offHasSack) != 0;
        if (offCanTakeOff >= 0) d.canTakeOff = ReadAt<uint8_t>(drone, offCanTakeOff) != 0;

        if (sceneBase) {
            for (const auto& child : R::ChildObjectsOf(drone)) {
                if (!child.object || !R::IsLive(child.object) ||
                    !R::IsDescendantOfAny(R::ClassOf(child.object), sceneBases, 16)) continue;
                ComponentDiag c;
                c.object = child.object;
                c.name = child.name;
                c.cls = child.className;
                c.attachParent = ReadAt<void*>(child.object, sceneParentOff);
                if (c.attachParent && R::IsLive(c.attachParent))
                    c.parentOwner = R::OuterOf(c.attachParent);
                c.visByte = ReadAt<uint8_t>(child.object,
                    static_cast<int32_t>(P::off::USceneComponent_VisFlagsByte));
                c.hiddenByte = ReadAt<uint8_t>(child.object,
                    static_cast<int32_t>(P::off::USceneComponent_HiddenFlagsByte));
                const int32_t meshOff = Off(R::ClassOf(child.object), L"StaticMesh");
                const int32_t skeletalOff = Off(R::ClassOf(child.object), L"SkeletalMesh");
                if (meshOff >= 0) c.meshAsset = ReadAt<void*>(child.object, meshOff);
                if (!c.meshAsset && skeletalOff >= 0)
                    c.meshAsset = ReadAt<void*>(child.object, skeletalOff);
                if (c.meshAsset && R::IsLive(c.meshAsset))
                    c.meshName = R::ToString(R::NameOf(c.meshAsset));
                const int32_t childActorOff = Off(R::ClassOf(child.object), L"ChildActor");
                if (childActorOff >= 0) {
                    c.childActor = ReadAt<void*>(child.object, childActorOff);
                    if (c.childActor && R::IsLive(c.childActor)) {
                        c.childActorName = R::ToString(R::NameOf(c.childActor));
                        c.childActorClass = R::ClassNameOf(c.childActor);
                    } else {
                        c.childActor = nullptr;
                    }
                }
                hash = MixDiag(hash, reinterpret_cast<uintptr_t>(c.object));
                hash = MixDiag(hash, reinterpret_cast<uintptr_t>(c.attachParent));
                hash = MixDiag(hash, reinterpret_cast<uintptr_t>(c.parentOwner));
                hash = MixDiag(hash, reinterpret_cast<uintptr_t>(c.meshAsset));
                hash = MixDiag(hash, reinterpret_cast<uintptr_t>(c.childActor));
                hash = MixText(hash, c.childActorName);
                hash = MixText(hash, c.childActorClass);
                hash = MixDiag(hash, c.visByte);
                hash = MixDiag(hash, c.hiddenByte);
                hash = MixText(hash, c.name);
                hash = MixText(hash, c.cls);
                hash = MixText(hash, c.meshName);
                d.components.push_back(std::move(c));
            }
        }
        hash = MixDiag(hash, reinterpret_cast<uintptr_t>(d.actor));
        hash = MixDiag(hash, reinterpret_cast<uintptr_t>(d.container));
        hash = MixDiag(hash, d.hasSack ? 1u : 0u);
        hash = MixDiag(hash, d.canTakeOff ? 1u : 0u);
        hash = MixText(hash, d.name);
        droneRows.push_back(std::move(d));
    }

    void* localPlayer = coop::players::Registry::Get().Local();
    for (void* sack : sacks) {
        if (!sack || !R::IsLive(sack)) continue;
        SackDiag r;
        r.actor = sack;
        r.name = R::ToString(R::NameOf(sack));
        r.cls = R::ClassNameOf(sack);
        r.key = ue_wrap::prop::GetKeyString(sack);
        void* cls = R::ClassOf(sack);
        const int32_t offContainer = Off(cls, L"container");
        const int32_t offTaken = Off(cls, L"takenByDrone");
        if (offContainer >= 0) r.container = ReadAt<void*>(sack, offContainer);
        if (offTaken >= 0) r.taken = ReadAt<uint8_t>(sack, offTaken) != 0;
        r.parentRead = E::GetAttachParentActor(sack, r.parent);
        const auto id = coop::element::Registry::Get().EidForActor(sack);
        if (id != coop::element::kInvalidId) r.eid = static_cast<uint32_t>(id);
        const auto mirrorId = coop::remote_prop::ResolveMirrorEidByActor(sack, true);
        if (mirrorId != coop::element::kInvalidId) r.mirrorEid = static_cast<uint32_t>(mirrorId);
        r.root = E::RootComponentOf(sack);
        if (r.root && R::IsLive(r.root)) {
            r.rootAttachParent = ReadAt<void*>(r.root, sceneParentOff);
            if (r.rootAttachParent && R::IsLive(r.rootAttachParent))
                r.rootParentOwner = R::OuterOf(r.rootAttachParent);
            r.rootVisByte = ReadAt<uint8_t>(r.root,
                static_cast<int32_t>(P::off::USceneComponent_VisFlagsByte));
            r.rootHiddenByte = ReadAt<uint8_t>(r.root,
                static_cast<int32_t>(P::off::USceneComponent_HiddenFlagsByte));
        }
        r.rootCollision = E::GetActorRootCollisionEnabled(sack);
        r.simulating = E::IsActorRootSimulatingPhysics(sack);
        r.locRead = E::TryGetActorLocation(sack, r.loc);
        r.heldByLocal = localPlayer && E::IsMainPlayerGrabbing(localPlayer, sack);

        hash = MixDiag(hash, reinterpret_cast<uintptr_t>(r.actor));
        hash = MixDiag(hash, reinterpret_cast<uintptr_t>(r.container));
        hash = MixDiag(hash, reinterpret_cast<uintptr_t>(r.parent));
        hash = MixDiag(hash, r.parentRead ? 1u : 0u);
        hash = MixDiag(hash, r.taken ? 1u : 0u);
        hash = MixDiag(hash, r.eid);
        hash = MixDiag(hash, r.mirrorEid);
        hash = MixDiag(hash, reinterpret_cast<uintptr_t>(r.rootAttachParent));
        hash = MixDiag(hash, reinterpret_cast<uintptr_t>(r.rootParentOwner));
        hash = MixDiag(hash, r.rootVisByte);
        hash = MixDiag(hash, r.rootHiddenByte);
        hash = MixDiag(hash, static_cast<uint64_t>(r.rootCollision + 1));
        hash = MixDiag(hash, r.simulating ? 1u : 0u);
        hash = MixDiag(hash, r.heldByLocal ? 1u : 0u);
        if (r.locRead) {
            hash = MixDiag(hash, static_cast<uint64_t>(static_cast<int64_t>(r.loc.X * 10.0f)));
            hash = MixDiag(hash, static_cast<uint64_t>(static_cast<int64_t>(r.loc.Y * 10.0f)));
            hash = MixDiag(hash, static_cast<uint64_t>(static_cast<int64_t>(r.loc.Z * 10.0f)));
        }
        hash = MixText(hash, r.name);
        hash = MixText(hash, r.cls);
        hash = MixText(hash, r.key);
        sackRows.push_back(std::move(r));
    }

    if (!g_sackDiagFirst && hash == g_lastSackDiagHash) return;
    g_sackDiagFirst = false;
    g_lastSackDiagHash = hash;
    UE_LOGI("[drone_sack_diag] %s -- drones=%zu prop_dronesack_actors=%zu (component rows follow)",
            isHost ? "HOST" : "CLIENT", droneRows.size(), sackRows.size());
    for (const auto& d : droneRows) {
        UE_LOGI("[drone_sack_diag] DRONE actor=%p name='%ls' hasSack=%d canTakeOff=%d container=%p components=%zu",
                d.actor, d.name.c_str(), d.hasSack ? 1 : 0, d.canTakeOff ? 1 : 0,
                d.container, d.components.size());
        for (const auto& c : d.components) {
            const bool visible = (c.visByte & (1u << 4)) != 0;
            const bool hidden = (c.hiddenByte & (1u << 2)) != 0;
            UE_LOGI("[drone_sack_diag] DRONE-COMP owner=%p comp=%p name='%ls' class='%ls' parentComp=%p parentActor=%p mesh=%p('%ls') childActor=%p('%ls'/%ls) visible=%d hidden=%d",
                    d.actor, c.object, c.name.c_str(), c.cls.c_str(), c.attachParent,
                    c.parentOwner, c.meshAsset, c.meshName.c_str(), c.childActor,
                    c.childActorName.c_str(), c.childActorClass.c_str(), visible ? 1 : 0, hidden ? 1 : 0);
        }
    }
    for (const auto& r : sackRows) {
        const bool rootVisible = (r.rootVisByte & (1u << 4)) != 0;
        const bool rootHidden = (r.rootHiddenByte & (1u << 2)) != 0;
        const std::wstring parentName = (r.parent && R::IsLive(r.parent))
            ? R::ToString(R::NameOf(r.parent)) : L"<null>";
        const std::wstring parentClass = (r.parent && R::IsLive(r.parent))
            ? R::ClassNameOf(r.parent) : L"<null>";
        const std::wstring rootOwnerName = (r.rootParentOwner && R::IsLive(r.rootParentOwner))
            ? R::ToString(R::NameOf(r.rootParentOwner)) : L"<null>";
        UE_LOGI("[drone_sack_diag] SACK actor=%p name='%ls' class='%ls' key='%ls' eid=%u mirrorEid=%u container=%p taken=%d "
                "parentRead=%d parent=%p('%ls'/%ls) root=%p rootParentComp=%p rootParentOwner=%p('%ls') "
                "visible=%d hidden=%d collision=%d simulating=%d heldByLocal=%d locRead=%d loc=(%.1f,%.1f,%.1f)",
                r.actor, r.name.c_str(), r.cls.c_str(), r.key.c_str(), r.eid, r.mirrorEid, r.container,
                r.taken ? 1 : 0, r.parentRead ? 1 : 0, r.parent, parentName.c_str(), parentClass.c_str(),
                r.root, r.rootAttachParent, r.rootParentOwner, rootOwnerName.c_str(),
                rootVisible ? 1 : 0, rootHidden ? 1 : 0, r.rootCollision, r.simulating ? 1 : 0,
                r.heldByLocal ? 1 : 0, r.locRead ? 1 : 0, r.loc.X, r.loc.Y, r.loc.Z);
    }
}

uint64_t HashRows(const std::vector<Row>& rows) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](uint64_t v) { h ^= v; h *= 1099511628211ull; };
    for (const auto& r : rows) {
        mix(reinterpret_cast<uintptr_t>(r.actor));
        mix(static_cast<uint64_t>(r.eid));
        mix(static_cast<uint64_t>(static_cast<uint32_t>(r.index)));
        mix(static_cast<uint64_t>(static_cast<uint32_t>(r.contents)));
        mix(r.isDroneContainer ? 1u : 0u);
        mix(r.isSackContainer ? 2u : 0u);
        for (wchar_t c : r.key) mix(static_cast<uint64_t>(c));
    }
    mix(static_cast<uint64_t>(rows.size()));
    return h;
}

}  // namespace

void Tick(bool isHost) {
    static const bool s_enabled = coop::config::ResolveFlag(::coop::config_registry::rows::delivery_census);
    if (!s_enabled) return;
    const uint64_t nowMs = NowMs();
    if (nowMs < g_nextSample) return;
    g_nextSample = nowMs + kSampleMs;

    // The two pointers that DEFINE the answer. Read them first so the rows can be marked.
    void* droneContainer = nullptr;
    std::vector<void*> sackContainers;

    const std::vector<void*> drones = R::FindObjectsByClass(L"drone_C");
    for (void* d : drones) {
        if (!d || !R::IsLive(d)) continue;
        const int32_t off = Off(R::ClassOf(d), L"container");
        if (off >= 0) droneContainer = ReadAt<void*>(d, off);
    }

    // Q-B: a sack REFERENCES a container through its own `container` field rather than holding
    // contents, so enumerating the sacks turns "is the sack a third receiver" into a pointer
    // compare.
    const std::vector<void*> sacks = R::FindObjectsByClass(L"prop_dronesack_C");
    for (void* s : sacks) {
        if (!s || !R::IsLive(s)) continue;
        void* scls = R::ClassOf(s);
        const int32_t offC = Off(scls, L"container");
        const int32_t offT = Off(scls, L"takenByDrone");
        void* sc = (offC >= 0) ? ReadAt<void*>(s, offC) : nullptr;
        if (sc) sackContainers.push_back(sc);
        UE_LOGI("[delivery_census] SACK actor=%p taken=%d container=%p", s,
                (offT >= 0) ? (ReadAt<uint8_t>(s, offT) ? 1 : 0) : -1, sc);
    }
    EmitDroneSackDiagnostic(isHost, drones, sacks);

    // Q-A: COUNT the containers by CLASS. Not a key lookup -- the key lookup is the broken thing.
    std::vector<Row> rows;
    for (void* a : R::FindObjectsByClass(L"prop_inventoryContainer_drone_C")) {
        if (!a || !R::IsLive(a)) continue;
        Row r;
        r.actor = a;
        FillCommon(r);
        FillInventory(r);
        r.isDroneContainer = (a == droneContainer);
        for (void* sc : sackContainers) if (sc == a) r.isSackContainer = true;
        rows.push_back(std::move(r));
    }

    // --- whole-GObjStack slot delta (the "where did it go" reading) ---
    {
        std::vector<int32_t> cur;
        if (void* save = ue_wrap::inventory::ResolveSaveSlot()) {
            const int32_t offStack = Off(R::ClassOf(save), L"GObjStack");
            if (offStack >= 0) {
                const SR::Arr stack = SR::ReadArr(save, offStack);
                cur.reserve(static_cast<size_t>(stack.num));
                for (int32_t i = 0; i < stack.num; ++i)
                    cur.push_back(SR::ReadArr(stack.data + static_cast<size_t>(i) * SR::kMxStride, 0).num);
            }
        }
        if (!g_prevSlots.empty() || !cur.empty()) {
            std::wstring deltas;
            const size_t n = cur.size() > g_prevSlots.size() ? cur.size() : g_prevSlots.size();
            int changed = 0;
            for (size_t i = 0; i < n; ++i) {
                const int32_t a = i < g_prevSlots.size() ? g_prevSlots[i] : -1;
                const int32_t b = i < cur.size() ? cur[i] : -1;
                if (a == b) continue;
                ++changed;
                if (changed <= 24) {
                    deltas += L" [" + std::to_wstring(i) + L"] " + std::to_wstring(a)
                            + L"->" + std::to_wstring(b);
                }
            }
            if (changed)
                UE_LOGI("[delivery_census] GObjStack slots=%zu changed=%d:%ls",
                        cur.size(), changed, deltas.c_str());
        }
        g_prevSlots.swap(cur);
    }

    const uint64_t h = HashRows(rows);
    if (!g_first && h == g_lastHash) return;   // EDGES only -- no identical-row wall
    g_first = false;
    g_lastHash = h;
    ++g_sampleNo;

    UE_LOGI("[delivery_census] #%d %s -- containers=%zu sacks=%zu drone.container=%p",
            g_sampleNo, isHost ? "HOST" : "CLIENT", rows.size(), sacks.size(), droneContainer);
    for (const auto& r : rows) {
        UE_LOGI("[delivery_census]   actor=%p cls='%ls' key='%ls' eid=%u idx=%d contents=%d "
                "vol=%.1f player=%d loc=(%.0f,%.0f,%.0f)%s%s%s",
                r.actor, r.cls.c_str(), r.key.empty() ? L"<none>" : r.key.c_str(), r.eid,
                r.index, r.contents, r.currVol, r.player ? 1 : 0, r.x, r.y, r.z, r.locRead ? "" : " (unread)",
                r.isDroneContainer ? "  <== drone.container (compileOrder writes HERE)" : "",
                r.isSackContainer ? "  <== sack.container" : "");
    }
    if (rows.empty())
        UE_LOGW("[delivery_census]   (no prop_inventoryContainer_drone_C actors alive)");
}

}  // namespace coop::dev::delivery_census
