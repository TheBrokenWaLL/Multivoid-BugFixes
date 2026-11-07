// ue_wrap/devices/drone.cpp -- see ue_wrap/devices/drone.h. Engine access for the delivery drone
// (Adrone_C). Offsets resolved from the live class via reflection (version-portable); the Alpha
// 0.9.0-n value is a logged fallback. The pose and the state bits are read from memory (the root
// component's relative transform, the dust component's bIsActive bit), writes go through
// ue_wrap::engine at the actor level: the drone flies by forces on its root sphere (`coll`, pushed by
// AddForce and AddTorqueInDegrees in its ReceiveTick), which carries the actor. Find() is the world
// singleton's drone_C.

#include "ue_wrap/devices/drone.h"

#include "ue_wrap/core/call.h"
#include "ue_wrap/core/cached_obj_ref.h"
#include "ue_wrap/engine/engine.h"
#include "ue_wrap/core/fname_utils.h"
#include "ue_wrap/core/log.h"
#include "ue_wrap/core/object_index.h"
#include "ue_wrap/core/reflection.h"
#include "ue_wrap/world/world_singleton.h"
#include "ue_wrap/core/sdk_profile.h"

#include <atomic>
#include <cmath>
#include <cstdint>

namespace ue_wrap::drone {
namespace {

namespace R = reflection;
namespace E = engine;

std::atomic<bool> g_resolved{false};
void*   g_cls       = nullptr;  // drone_C UClass
int32_t g_activeOff = -1;       // Adrone_C::Active (Alpha 0.9.0-n: 0x0370, bool)
int32_t g_leaveAfter5minOff = -1; // Adrone_C::leaveAfter5min

constexpr int32_t kActiveOffFallback = 0x0370;

                               // (the project rule; IsLive(ptr) derefs the maybe-freed object itself)

// ---- FX mirroring -- resolved lazily (separate from the pose path) ----
bool    g_fxResolved   = false;
bool    g_fxTried      = false;  // one attempt once the class is loaded: what it misses stays missing
int32_t g_canTakeOffOff = -1;  // Adrone_C::canTakeOff @0x0500 (arrival edge)
int32_t g_dustOff       = -1;  // Adrone_C::eff_droneDust @0x0278 (UParticleSystemComponent*)
int32_t g_audioAlarmOff = -1;  // Adrone_C::audio_alarm  @0x0230 (UAudioComponent*)
void*   g_setActiveFn   = nullptr;  // UActorComponent::SetActive(bool bNewActive, bool bReset)
void*   g_activateFn    = nullptr;  // UActorComponent::Activate(bool bReset)
int32_t g_activeByteOff  = -1;      // UActorComponent::bIsActive, a bitfield byte (what IsActive returns)
uint8_t g_activeMask     = 0;
int32_t g_lightAlarmOff = -1;       // Adrone_C::light_alarm @0x0240 (UPointLightComponent*) -- arrival signal light
void*   g_setFloatParamFn = nullptr;// UFXSystemComponent::SetFloatParameter(FName, float) -- dust intensity
void*   g_setVisFn        = nullptr;// USceneComponent::SetVisibility(bool, bool) -- the signal light
void*   g_getCompLocFn    = nullptr;// USceneComponent::K2_GetComponentLocation() -- host dust-anchor read
void*   g_setWorldLocFn   = nullptr;// USceneComponent::K2_SetWorldLocation(...) -- mirror dust-anchor pin
R::FName g_dustFName{0, 0};         // the 'dust' particle intensity param name (lazy; needs Kismet up)
constexpr int32_t kCanTakeOffFallback = 0x0500;
constexpr int32_t kDustFallback       = 0x0278;
constexpr int32_t kAudioAlarmFallback = 0x0230;
constexpr int32_t kLightAlarmFallback = 0x0240;
// The blueprint's per-tick intensity formula: 'dust' = 1 - distance(trace start, the hit, or the
// trace end when nothing was hit) / 2000 -- a RateScale multiplier on both spawn modules, so its
// valid range is [0,1].
constexpr float kDustTraceLen = 2000.f;
// Interaction-gate fields: the client mirror must carry these so a PARKED drone is interactable,
// since the suppressed tick never sets them. canTakeOff@0x0500 IS the gate (true means parked);
// hasSack@0x0501 gates the action options (open-inv and drop-sack); container@0x04F8 is the
// inventory actor openPropInv opens, a keyed Aprop_C the prop pipeline already mirrors.
int32_t g_hasSackOff   = -1;   // Adrone_C::hasSack   @0x0501
int32_t g_containerOff = -1;   // Adrone_C::container @0x04F8 (Aprop_inventoryContainer_drone_C*)
constexpr int32_t kHasSackFallback   = 0x0501;
constexpr int32_t kContainerFallback = 0x04F8;
bool g_gateFieldsResolved = false;

bool EnsureGateFieldsResolved() {
    if (g_gateFieldsResolved) return true;
    if (!EnsureResolved() || !g_cls) return false;
    g_canTakeOffOff = R::FindPropertyOffset(g_cls, L"canTakeOff");
    if (g_canTakeOffOff < 0) g_canTakeOffOff = kCanTakeOffFallback;
    g_hasSackOff = R::FindPropertyOffset(g_cls, L"hasSack");
    if (g_hasSackOff < 0) g_hasSackOff = kHasSackFallback;
    g_containerOff = R::FindPropertyOffset(g_cls, L"container");
    if (g_containerOff < 0) g_containerOff = kContainerFallback;
    g_gateFieldsResolved = true;
    return true;
}

bool EnsureFxResolved() {
    if (g_fxResolved) return true;
    if (g_fxTried || !EnsureResolved() || !g_cls) return false;
    g_fxTried = true;
    if (!EnsureGateFieldsResolved()) return false;
    g_dustOff = R::FindPropertyOffset(g_cls, L"eff_droneDust");
    if (g_dustOff < 0) g_dustOff = kDustFallback;
    g_audioAlarmOff = R::FindPropertyOffset(g_cls, L"audio_alarm");
    if (g_audioAlarmOff < 0) g_audioAlarmOff = kAudioAlarmFallback;
    g_lightAlarmOff = R::FindPropertyOffset(g_cls, L"light_alarm");
    if (g_lightAlarmOff < 0) g_lightAlarmOff = kLightAlarmFallback;
    // SetActive/Activate and bIsActive are declared on UActorComponent (the components' base); resolve
    // them there and dispatch on the concrete component instance (ProcessEvent resolves the impl).
    if (void* acCls = R::FindClass(L"ActorComponent")) {
        g_setActiveFn = R::FindFunction(acCls, L"SetActive");
        g_activateFn  = R::FindFunction(acCls, L"Activate");
        R::FindBoolProperty(acCls, L"bIsActive", g_activeByteOff, g_activeMask);
    }
    if (!g_setActiveFn || !g_activateFn || g_activeByteOff < 0) {
        UE_LOGW("drone: FX component members incomplete (SetActive=%p Activate=%p bIsActive@%d) -- "
                "FX mirror disabled", g_setActiveFn, g_activateFn, g_activeByteOff);
        return false;
    }
    // Best-effort FX-polish UFunctions (dust intensity + the signal light) -- not required for the
    // core mirror, so they don't gate g_fxResolved; logged if absent.
    // SetFloatParameter is declared on UFXSystemComponent (the PARENT of UParticleSystemComponent),
    // NOT on UParticleSystemComponent itself -- and FindFunction does NOT climb the super chain, so
    // it MUST be resolved on the owning class (the same gotcha SetActive->ActorComponent handles
    // above + engine_widget's SetVisibility->Widget). Resolving on ParticleSystemComponent returns
    // null -> the 'dust' param is never set -> the emitter stays at rate 0 -> NO dust (the bug).
    if (void* fxCls = R::FindClass(L"FXSystemComponent"))
        g_setFloatParamFn = R::FindFunction(fxCls, L"SetFloatParameter");
    if (void* scCls = R::FindClass(profile::name::SceneComponentClass)) {
        g_setVisFn      = R::FindFunction(scCls, profile::name::SetVisibilityFn);
        g_getCompLocFn  = R::FindFunction(scCls, L"K2_GetComponentLocation");
        g_setWorldLocFn = R::FindFunction(scCls, L"K2_SetWorldLocation");
    }
    if (!g_setFloatParamFn || !g_setVisFn || !g_getCompLocFn || !g_setWorldLocFn)
        UE_LOGW("drone: FX polish UFunctions partial (SetFloatParameter=%p SetVisibility=%p "
                "GetCompLoc=%p SetWorldLoc=%p) -- dust / signal light may be limited",
                g_setFloatParamFn, g_setVisFn, g_getCompLocFn, g_setWorldLocFn);
    g_fxResolved = true;
    UE_LOGI("drone: FX resolved canTakeOff@0x%04X eff_droneDust@0x%04X audio_alarm@0x%04X "
            "light_alarm@0x%04X hasSack@0x%04X container@0x%04X",
            g_canTakeOffOff, g_dustOff, g_audioAlarmOff, g_lightAlarmOff, g_hasSackOff, g_containerOff);
    return true;
}

void* ReadComp(void* drone, int32_t off) {
    if (!drone || off < 0) return nullptr;
    void* c = *reinterpret_cast<void**>(reinterpret_cast<char*>(drone) + off);
    return (c && R::IsLive(c)) ? c : nullptr;
}

// UActorComponent::IsActive returns bIsActive, so the bit is read where it lives.
bool ComponentIsActive(void* comp) {
    if (!comp || g_activeByteOff < 0) return false;
    return (*(reinterpret_cast<const uint8_t*>(comp) + g_activeByteOff) & g_activeMask) != 0;
}

void SetComponentActive(void* comp, bool on) {
    if (!comp || !g_setActiveFn) return;
    ParamFrame f(g_setActiveFn);
    if (!f.valid()) return;
    f.Set<bool>(L"bNewActive", on);
    f.Set<bool>(L"bReset", false);
    Call(comp, f);
}

void ActivateComponent(void* comp) {
    if (!comp || !g_activateFn) return;
    ParamFrame f(g_activateFn);
    if (!f.valid()) return;
    f.Set<bool>(L"bReset", true);  // restart the cue from the top each arrival
    Call(comp, f);
}

void* g_sackClass = nullptr;
int32_t g_sackContainerOff = -1;
int32_t g_sackTakenOff = -1;
// The Blueprint may clear/rebind the sack container during dropSack. Keep a slot-validated
// reference to the drone's current sack so the host can still publish its detached actor.
CachedObjRef g_sackOwnerDrone;
CachedObjRef g_sackActor;

bool EnsureSackFieldsResolved() {
    if (!g_sackClass) g_sackClass = object_index::ClassByName(L"prop_dronesack_C");
    if (!g_sackClass) return false;
    if (g_sackContainerOff < 0)
        g_sackContainerOff = R::FindPropertyOffset(g_sackClass, L"container");
    if (g_sackTakenOff < 0)
        g_sackTakenOff = R::FindPropertyOffset(g_sackClass, L"takenByDrone");
    return g_sackContainerOff >= 0 && g_sackTakenOff >= 0;
}

struct SackSearch {
    void* drone = nullptr;
    void* container = nullptr;
    void* exclude = nullptr;
    void* best = nullptr;
    bool attachedOnly = false;
    bool includeOrphanedMountFlags = false; // cleanup scans only; never the normal sack resolver
    int bestAttachmentRank = 0; // 3=physical parent, 2=orphaned takenByDrone marker
    bool bestContainerMatch = false;
    float bestDistanceSq = 1.0e30f;
};

void FindSackCandidate(void* raw, void* sack, int32_t) {
    auto& search = *static_cast<SackSearch*>(raw);
    if (!sack || sack == search.exclude || !R::IsLive(sack)) return;
    const auto* bytes = reinterpret_cast<const uint8_t*>(sack);
    void* const container = *reinterpret_cast<void* const*>(bytes + g_sackContainerOff);
    const bool containerMatch = search.container && container == search.container;

    // The actor relationship is a stronger identity signal than a possibly stale Blueprint
    // container pointer. This is important during the drop transition: the old mounted sack must
    // still be found so the host can publish it and the client can retire/adopt its local copy.
    const bool taken = *reinterpret_cast<const bool*>(bytes + g_sackTakenOff);
    void* parent = nullptr;
    const bool parentRead = E::GetAttachParentActor(sack, parent);
    // GetAttachParentActor can successfully return nullptr for a sack that the Blueprint still
    // marks as mounted (the sack may be held at the drone socket by takenByDrone rather than by an
    // actor-parent edge). Keep that actor in the cleanup candidate set so the client can retire its
    // save-loaded visual twin. Do NOT use this fallback for the gameplay attachment gate:
    // IsSackAttached deliberately remains strict and trusts a readable physical parent.
    const bool attached = parentRead ? parent == search.drone : taken;
    const bool orphanedMountFlag = search.includeOrphanedMountFlags && parentRead && parent == nullptr && taken;
    const bool attachedCandidate = attached || orphanedMountFlag;
    if (search.attachedOnly ? !attachedCandidate : (!containerMatch && !attachedCandidate)) return;

    FVector loc{};
    float distanceSq = 0.f;
    if (E::TryGetActorLocation(sack, loc)) {
        FVector droneLoc{};
        if (E::TryGetActorLocation(search.drone, droneLoc)) {
            const float dx = loc.X - droneLoc.X, dy = loc.Y - droneLoc.Y, dz = loc.Z - droneLoc.Z;
            distanceSq = dx * dx + dy * dy + dz * dz;
        }
    }
    // Prefer physically attached actors, then orphaned mount markers (cleanup only), then actors
    // already bound to the shared inventory, then proximity. The strict IsSackAttached query below
    // is unchanged, so a stale marker can never enable the drone's drop/interaction option.
    const int rank = (attached ? 3 : (orphanedMountFlag ? 2 : 0)) + (containerMatch ? 1 : 0);
    const int bestRank = search.bestAttachmentRank + (search.bestContainerMatch ? 1 : 0);
    if (!search.best || rank > bestRank || (rank == bestRank && distanceSq < search.bestDistanceSq)) {
        search.best = sack;
        search.bestAttachmentRank = attached ? 3 : (orphanedMountFlag ? 2 : 0);
        search.bestContainerMatch = containerMatch;
        search.bestDistanceSq = distanceSq;
    }
}

bool WriteSackTaken(void* sack, bool taken) {
    if (!sack || !R::IsLive(sack) || !EnsureSackFieldsResolved()) return false;
    *reinterpret_cast<bool*>(reinterpret_cast<char*>(sack) + g_sackTakenOff) = taken;
    return true;
}

}  // namespace

bool EnsureResolved() {
    if (g_resolved.load(std::memory_order_acquire)) return true;
    void* cls = object_index::ClassByName(L"drone_C");   // one lookup, loaded or not
    if (!cls) return false;  // not loaded yet -- caller retries

    int32_t activeOff = R::FindPropertyOffset(cls, L"Active");
    if (activeOff < 0) {
        UE_LOGW("drone: reflected Active offset not found -- using fallback 0x%04X", kActiveOffFallback);
        activeOff = kActiveOffFallback;
    }

    g_cls = cls;
    g_activeOff = activeOff;
    g_leaveAfter5minOff = R::FindPropertyOffset(cls, L"leaveAfter5min");
    if (g_leaveAfter5minOff < 0)
        UE_LOGW("drone: reflected leaveAfter5min offset not found -- the leave timer does not sync");
    g_resolved.store(true, std::memory_order_release);
    UE_LOGI("drone: resolved drone_C=%p Active@0x%04X", cls, activeOff);
    return true;
}

void* Find() {
    if (!EnsureResolved()) return nullptr;
    return world_singleton::Find(L"drone_C");   // the delivery drone, one of it
}

bool IsActive(void* drone) {
    if (!drone || g_activeOff < 0) return false;
    return *reinterpret_cast<const bool*>(reinterpret_cast<const char*>(drone) + g_activeOff);
}

bool ReadLeaveAfter5min(void* drone, bool& out) {
    if (!drone || !EnsureResolved() || g_leaveAfter5minOff < 0) return false;
    out = *reinterpret_cast<const bool*>(reinterpret_cast<const char*>(drone) + g_leaveAfter5minOff);
    return true;
}

bool WriteLeaveAfter5min(void* drone, bool value) {
    if (!drone || !EnsureResolved() || g_leaveAfter5minOff < 0) return false;
    *reinterpret_cast<bool*>(reinterpret_cast<char*>(drone) + g_leaveAfter5minOff) = value;
    return true;
}

bool ReadPose(void* drone, FVector& loc, FRotator& rot) {
    if (!drone) return false;
    if (E::TryGetRootPoseRaw(drone, loc, rot)) return true;
    return E::TryGetActorLocation(drone, loc) && E::TryGetActorRotation(drone, rot);
}

bool DriveMirror(void* drone, const FVector& loc, const FRotator& rot) {
    if (!drone) return false;
    bool ok = E::SetActorLocation(drone, loc);
    ok = E::SetActorRotation(drone, rot) && ok;
    return ok;
}

void SuppressTick(void* drone) {
    if (!drone) return;
    E::SetActorTickEnabled(drone, false);  // stop the flight integrator -- the client drone is a mirror
}

void RestoreTick(void* drone) {
    if (!drone) return;
    E::SetActorTickEnabled(drone, true);
}

void* FindAttachedSack(void* drone, void* except) {
    if (!drone || !R::IsLive(drone) || !EnsureSackFieldsResolved()) return nullptr;
    SackSearch search;
    search.drone = drone;
    search.container = ContainerOf(drone);
    search.exclude = except;
    search.attachedOnly = true;
    search.includeOrphanedMountFlags = true;
    object_index::ForEachInstance(g_sackClass, &FindSackCandidate, &search);
    return search.best;
}

void* FindSack(void* drone) {
    if (!drone || !R::IsLive(drone) || !EnsureSackFieldsResolved()) return nullptr;
    if (!g_sackOwnerDrone.Is(drone)) {
        g_sackOwnerDrone.Set(drone);
        g_sackActor.Reset();
    }
    // Prefer a sack physically mounted on this drone. This normal resolver deliberately excludes
    // orphaned takenByDrone markers: those are discovered only by the cleanup scans below, so a
    // stale local visual can never win over the authoritative dropped actor.
    SackSearch mountedSearch;
    mountedSearch.drone = drone;
    mountedSearch.container = ContainerOf(drone);
    mountedSearch.attachedOnly = true;
    object_index::ForEachInstance(g_sackClass, &FindSackCandidate, &mountedSearch);
    if (mountedSearch.best) {
        if (!g_sackActor.Is(mountedSearch.best)) g_sackActor.Set(mountedSearch.best);
        return mountedSearch.best;
    }
    if (void* cached = g_sackActor.Get()) return cached;

    SackSearch search;
    search.drone = drone;
    search.container = ContainerOf(drone);
    object_index::ForEachInstance(g_sackClass, &FindSackCandidate, &search);
    if (search.best) g_sackActor.Set(search.best);
    return search.best;
}

bool SetSackContainer(void* drone, void* sack) {
    if (!drone || !sack || !R::IsLive(drone) || !R::IsLive(sack) ||
        !EnsureGateFieldsResolved() || !EnsureSackFieldsResolved()) return false;
    void* const container = ContainerOf(drone);
    if (!container || !R::IsLive(container)) return false;
    void** const slot = reinterpret_cast<void**>(reinterpret_cast<char*>(sack) + g_sackContainerOff);
    if (*slot == container) return true;
    void* const previous = *slot;
    *slot = container;
    UE_LOGI("drone: rebound sack container @0x%04X %p -> %p", g_sackContainerOff, previous, container);
    return true;
}

bool IsSackAttached(void* drone, void* sack) {
    if (!drone || !sack || !R::IsLive(drone) || !R::IsLive(sack) ||
        !EnsureSackFieldsResolved()) return false;
    const bool taken = *reinterpret_cast<const bool*>(reinterpret_cast<const char*>(sack) + g_sackTakenOff);
    void* parent = nullptr;
    if (E::GetAttachParentActor(sack, parent)) return parent == drone;
    return taken;  // use the Blueprint flag only when the engine cannot read the parent edge
}

bool ApplySackMirror(void* drone, void* sack, bool present, bool attached, bool simulatePhysics,
                     bool poseValid, const FVector& loc, const FRotator& rot) {
    if (!drone || !R::IsLive(drone)) return false;
    if (!present) return true;
    if (!sack || !R::IsLive(sack) || !EnsureSackFieldsResolved()) return false;
    // The sack actor itself is the opener used by the game's actionOptionIndex -> openPropInv
    // path. The drone's container field alone is not enough: repair both sides of that relation.
    if (!SetSackContainer(drone, sack)) return false;

    if (attached) {
        // Recreate the attachment when this peer is missing it; merely setting takenByDrone
        // leaves the actor on the floor while the host reports it mounted. The host pose in this
        // state edge corrects the world transform once, then the actor follows the drone root.
        void* parent = nullptr;
        const bool parentRead = E::GetAttachParentActor(sack, parent);
        if (parentRead && parent && parent != drone) return false; // preserve a locally owned prop
        if (!parentRead || parent != drone) {
            void* const root = E::RootComponentOf(drone);
            if (!root) return false;
            E::SetActorSimulatePhysics(sack, false);
            if (!E::AttachActorToComponentKeepWorld(sack, root)) return false;
        }
        if (poseValid) {
            E::SetActorLocation(sack, loc);
            E::SetActorRotation(sack, rot);
        }
        E::SetActorEnableCollision(sack, true);
        E::SetActorRootCollisionEnabled(sack, 3); // QueryAndPhysics
        E::SetActorSimulatePhysics(sack, simulatePhysics);
        return WriteSackTaken(sack, true);
    }

    // A detached mirror must remain grabbable with E. Only detach from this drone; never steal a
    // sack currently parented to the local player/another owner. Restore actor + root query collision
    // because the Blueprint drop graph is intentionally suppressed on clients.
    void* parent = nullptr;
    const bool parentRead = E::GetAttachParentActor(sack, parent);
    const bool taken = *reinterpret_cast<const bool*>(
        reinterpret_cast<const char*>(sack) + g_sackTakenOff);
    if (parentRead && parent == drone && !E::DetachActorFromParent(sack)) return false;
    if (!parentRead && taken && !E::DetachActorFromParent(sack)) return false;
    if (parentRead && parent && parent != drone) return WriteSackTaken(sack, false);
    if (poseValid) {
        E::SetActorLocation(sack, loc);
        E::SetActorRotation(sack, rot);
    }
    E::SetActorEnableCollision(sack, true);
    E::SetActorRootCollisionEnabled(sack, 3); // QueryAndPhysics
    E::SetActorSimulatePhysics(sack, simulatePhysics);
    return WriteSackTaken(sack, false);
}

uint8_t ReadFxBits(void* drone) {
    if (!drone) return 0;
    uint8_t bits = 0;
    bool leaveAfter5min = false;
    if (ReadLeaveAfter5min(drone, leaveAfter5min) && leaveAfter5min) bits |= kFxLeaveAfter5min;
    if (EnsureGateFieldsResolved()) {
        const auto readBool = [drone](int32_t off) -> bool {
            return off >= 0 && *reinterpret_cast<const bool*>(reinterpret_cast<const char*>(drone) + off);
        };
        if (readBool(g_canTakeOffOff)) bits |= kFxArrived;  // parked/arrived -> the interaction gate
        // Keep the Blueprint's own hasSack flag as the fallback when its sack actor is not resolvable.
        // triggerFly/compileOrder may also consult this game-owned flag; an incomplete actor search
        // must not turn a normal drone operation into a false "no sack" state.
        if (readBool(g_hasSackOff)) bits |= kFxHasSack;
    }
    // When the actor is positively found detached, its physical state can clear a stale Blueprint
    // hasSack flag. If actor lookup fails, retain the game-owned value above instead of mutating the
    // drone into an unusable state.
    if (void* sack = FindSack(drone)) {
        bits |= kFxSackPresent;
        if (IsSackAttached(drone, sack)) bits |= kFxSackAttached | kFxHasSack;
        else bits &= static_cast<uint8_t>(~kFxHasSack);
        if (E::IsActorRootSimulatingPhysics(sack)) bits |= kFxSackSimulating;
    }
    // FX components are optional; their absence must not hide the interaction-gate state above.
    if (!EnsureFxResolved()) return bits;
    if (void* dust = ReadComp(drone, g_dustOff))
        if (ComponentIsActive(dust)) bits |= kFxDust;
    return bits;
}

bool ReadDustAnchor(void* drone, FVector& out) {
    if (!drone || !EnsureFxResolved() || !g_getCompLocFn) return false;
    void* dust = ReadComp(drone, g_dustOff);
    if (!dust) return false;
    ParamFrame f(g_getCompLocFn);
    if (!f.valid() || !Call(dust, f)) return false;
    out = f.Get<FVector>(L"ReturnValue");
    return true;
}

void ApplyDustMirror(void* drone, bool on, const FVector& anchor) {
    if (!drone || !EnsureFxResolved()) return;
    void* dust = ReadComp(drone, g_dustOff);
    if (!dust) return;
    // 1. The blueprint's own edge form: SetActive(want, bReset=false) only when IsActive() != want.
    //    Load-bearing both ways -- it deactivates on leaving the ground AND re-arms the one-loop
    //    system after it self-completes mid-hover, which a latched rising-edge replay cannot do.
    if (ComponentIsActive(dust) != on) SetComponentActive(dust, on);
    if (!on) return;
    // 2. Pin the component to the host's ground-trace hit. eff_droneDust is bAbsoluteLocation with
    //    NO relative offset, so unmoved it renders at WORLD ORIGIN and its fixed relative bounding
    //    box is frustum-culled, so the dust draws invisibly rather than not at all. Same call
    //    form as the
    //    blueprint: sweep=false, teleport=false. The blueprint VInterpTo's toward the hit; we do
    //    not, because the host streams its already-interpolated component location at 20 Hz.
    if (g_setWorldLocFn) {
        ParamFrame f(g_setWorldLocFn);
        if (f.valid()) {
            f.Set<FVector>(L"NewLocation", anchor);
            f.Set<bool>(L"bSweep", false);
            f.Set<bool>(L"bTeleport", false);
            Call(dust, f);
        }
    }
    // 3. The blueprint's intensity formula: 'dust' = 1 - distance(trace start, hit) / 2000, clamped
    //    to [0,1]. Lazy-resolve the FName -- it needs Kismet up, which post-boot it is.
    if (g_setFloatParamFn) {
        if (g_dustFName.ComparisonIndex == 0 && g_dustFName.Number == 0)
            g_dustFName = ue_wrap::fname_utils::StringToFName(L"dust");
        FVector loc{};   // unread: the dust keeps its last value
        if ((g_dustFName.ComparisonIndex != 0 || g_dustFName.Number != 0) && E::TryGetActorLocation(drone, loc)) {
            const float dx = loc.X - anchor.X, dy = loc.Y - anchor.Y, dz = loc.Z - anchor.Z;
            float dustVal = 1.f - std::sqrt(dx * dx + dy * dy + dz * dz) / kDustTraceLen;
            if (dustVal < 0.f) dustVal = 0.f;
            if (dustVal > 1.f) dustVal = 1.f;
            ParamFrame f(g_setFloatParamFn);
            if (f.valid()) {
                f.SetRaw(L"ParameterName", &g_dustFName, sizeof(g_dustFName));
                f.Set<float>(L"Param", dustVal);
                Call(dust, f);
            }
        }
    }
}

void PlayArrivalCue(void* drone) {
    if (!drone || !EnsureFxResolved()) return;
    // audio_alarm.Activate(true) = the "items ready" ding. Fire-and-forget on the arrival edge.
    if (void* audio = ReadComp(drone, g_audioAlarmOff)) ActivateComponent(audio);
}

void SetSignalLight(void* drone, bool on) {
    if (!drone || !EnsureFxResolved() || !g_setVisFn) return;
    // light_alarm is the visible arrival "signal" (the host blinks it; the mirror shows it steady
    // while the drone is parked-with-cargo, off otherwise -- the visual half of "items ready").
    if (void* light = ReadComp(drone, g_lightAlarmOff)) {
        ParamFrame f(g_setVisFn);
        if (!f.valid()) return;
        f.Set<bool>(L"bNewVisibility", on);
        f.Set<bool>(L"bPropagateToChildren", false);
        Call(light, f);
    }
}

bool SetSackVisual(void* drone, bool on) {
    if (!drone || !R::IsLive(drone)) return false;
    // The apparatus around the drone's own sack: the mesh component `sack` (SkeletalMeshComponent
    // over /Game/meshes/dronesack) is the visual, and it is the only one this mirrors. `sackPhys`
    // (the drone's root sphere), `sackHitbox` (the loose-sack overlap) and `sackConstraint` (the
    // re-mount physics) are machinery the Blueprint itself keeps hidden in every authored state --
    // hiding or showing them leaves a sphere swinging under the drone or an invisible overlap
    // behind. The MESH ASSET is the identity, not the component name: a Blueprint variable rename
    // cannot spoof it, and it is what the delivery census reads to name the visual.
    static int32_t sSkeletalMeshOff = -2;   // -2 = not resolved yet, -1 = absent (never matches)
    static int32_t sStaticMeshOff = -2;
    if (sSkeletalMeshOff == -2) {
        sSkeletalMeshOff = -1;
        if (void* cls = R::FindClass(L"SkeletalMeshComponent"))
            sSkeletalMeshOff = R::FindPropertyOffset(cls, L"SkeletalMesh");
    }
    if (sStaticMeshOff == -2) {
        sStaticMeshOff = -1;
        if (void* cls = R::FindClass(L"StaticMeshComponent"))
            sStaticMeshOff = R::FindPropertyOffset(cls, L"StaticMesh");
    }
    void* const skeletalBase = R::FindClass(L"SkeletalMeshComponent");
    void* const staticBase = R::FindClass(L"StaticMeshComponent");
    void* const bases[] = { skeletalBase, staticBase };
    if (sSkeletalMeshOff < 0 && sStaticMeshOff < 0) return false;
    for (const auto& child : R::ChildObjectsOf(drone)) {
        if (!child.object || !R::IsLive(child.object)) continue;
        void* const cls = R::ClassOf(child.object);
        if (!R::IsDescendantOfAny(cls, bases, 2)) continue;
        const bool isSkeletal = skeletalBase && R::IsDescendantOfAny(cls, &skeletalBase, 1);
        const int32_t off = isSkeletal ? sSkeletalMeshOff : sStaticMeshOff;
        if (off < 0) continue;
        void* const mesh = *reinterpret_cast<void* const*>(
            reinterpret_cast<const char*>(child.object) + off);
        if (!mesh || !R::IsLive(mesh)) continue;
        std::wstring meshName = R::ToString(R::NameOf(mesh));
        for (auto& ch : meshName)
            if (ch >= L'A' && ch <= L'Z') ch = static_cast<wchar_t>(ch - L'A' + L'a');
        if (meshName.find(L"dronesack") == std::wstring::npos) continue;
        E::SetComponentVisible(child.object, on, /*propagate=*/true);
        return true;
    }
    return false;
}

bool CanDropSack(void* drone) {
    if (!drone || !EnsureGateFieldsResolved()) return false;
    const char* const bytes = reinterpret_cast<const char*>(drone);
    const bool parked = *reinterpret_cast<const bool*>(bytes + g_canTakeOffOff);
    const bool blueprintHasSack = *reinterpret_cast<const bool*>(bytes + g_hasSackOff);
    void* const sack = FindSack(drone);
    // Let the authoritative Blueprint complete its own drop transition when its interaction gate
    // says the sack is mounted, even if the actor-parent query is momentarily stale/unavailable.
    return parked && (blueprintHasSack || (sack && IsSackAttached(drone, sack)));
}

void NormalizeDroppedSack(void* drone) {
    if (!drone || !R::IsLive(drone) || !EnsureGateFieldsResolved() || !EnsureSackFieldsResolved()) return;
    // A stale mount can coexist with the actor the Blueprint just dropped. Detach every remaining
    // drone-owned sack, but never detach an actor with a different live parent (a player hold).
    void* except = nullptr;
    for (int i = 0; i < 8; ++i) {
        void* const sack = FindAttachedSack(drone, except);
        if (!sack) break;
        except = sack;
        void* parent = nullptr;
        const bool parentRead = E::GetAttachParentActor(sack, parent);
        const bool taken = *reinterpret_cast<const bool*>(
            reinterpret_cast<const char*>(sack) + g_sackTakenOff);
        bool released = false;
        if (parentRead && parent == drone) released = E::DetachActorFromParent(sack);
        else if (!parentRead && taken) released = E::DetachActorFromParent(sack);
        else if (parentRead && parent == nullptr) released = true;
        if (!released && (!parentRead || parent == drone))
            UE_LOGW("drone: dropSack post-hook could not detach sack %p", sack);
        WriteSackTaken(sack, false);
        E::SetActorEnableCollision(sack, true);
        E::SetActorRootCollisionEnabled(sack, 3); // QueryAndPhysics
        if (released) E::SetActorSimulatePhysics(sack, true);
        UE_LOGI("drone: normalized dropped sack actor %p (released=%d)", sack, released ? 1 : 0);
    }
    // The Blueprint can leave this gate true after the sack actor leaves the socket. The host is
    // authoritative for the completed drop, so clear only this interaction prerequisite.
    *reinterpret_cast<bool*>(reinterpret_cast<char*>(drone) + g_hasSackOff) = false;
    // Do not let the pre-drop actor be reused as the authoritative post-drop sack if the Blueprint
    // destroyed/replaced it or cleared its inventory binding. The normal prop lane/key census can
    // discover the resulting world actor independently on the next host read.
    g_sackActor.Reset();
}

bool DropSack(void* drone) {
    if (!drone || !R::IsLive(drone)) return false;
    void* const fn = R::FindDispatchFunctionCached(R::ClassOf(drone), L"dropSack");
    if (!fn) return false;
    ParamFrame frame(fn);
    return frame.valid() && Call(drone, frame);
}

void WriteGateFields(void* drone, bool canTakeOff, bool hasSack) {
    if (!drone || !EnsureGateFieldsResolved()) return;
    // The client's suppressed tick never sets these, so a parked mirror drone reads "in motion" +
    // shows no action options. Mirror the host's values so the same interaction options are exposed.
    *reinterpret_cast<bool*>(reinterpret_cast<char*>(drone) + g_canTakeOffOff) = canTakeOff;
    *reinterpret_cast<bool*>(reinterpret_cast<char*>(drone) + g_hasSackOff) = hasSack;
}

bool SetContainer(void* drone, void* container) {
    if (!drone || !container || !R::IsLive(drone) || !R::IsLive(container) ||
        !EnsureGateFieldsResolved()) return false;
    void* const containerClass = R::FindClass(L"prop_inventoryContainer_drone_C");
    void* const actualClass = R::ClassOf(container);
    if (!containerClass || !actualClass || !R::IsDescendantOfAny(actualClass, &containerClass, 1)) return false;

    void** const slot = reinterpret_cast<void**>(reinterpret_cast<char*>(drone) + g_containerOff);
    if (*slot == container) return true;
    void* const previous = *slot;
    *slot = container;
    UE_LOGI("drone: bound shared sack container @0x%04X %p -> %p", g_containerOff, previous, container);
    return true;
}

void* ContainerOf(void* drone) {
    if (!drone || !EnsureGateFieldsResolved()) return nullptr;
    void* const c = *reinterpret_cast<void* const*>(reinterpret_cast<const char*>(drone) + g_containerOff);
    return c && R::IsLive(c) ? c : nullptr;
}

void* OfSellLocation(void* sellLocation) {
    static R::InstanceOffset s_droneOff{L"drone"};
    if (!sellLocation) return nullptr;
    const int32_t off = s_droneOff.Of(sellLocation);
    if (off < 0) return nullptr;
    void* const d = *reinterpret_cast<void* const*>(static_cast<const uint8_t*>(sellLocation) + off);
    return d && R::IsLive(d) ? d : nullptr;
}

}  // namespace ue_wrap::drone
