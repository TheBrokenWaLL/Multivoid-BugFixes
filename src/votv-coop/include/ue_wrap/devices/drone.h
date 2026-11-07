// ue_wrap/devices/drone.h -- standalone engine access for the VOTV delivery DRONE (Adrone_C).
// Principle-7 engine-wrapper layer (NO network/coop state). coop::drone_sync drives the
// host-authoritative pose mirror through here.
//
// Adrone_C : AActor (direct) is the SINGLETON delivery drone (one per save, world-placed; NOT the
// player-piloted recon Aprop_rdrone_C). Its flight is a per-tick BP float integrator in ReceiveTick
// (Damping/Speed/Pitch/vecForce vs a world Spline) -- reproducing it bit-exact on a remote is fragile,
// so we HOST-AUTHORITATIVELY stream the resolved actor TRANSFORM and the client mirrors it kinematically
// (SuppressTick stops the client drone's own ReceiveTick so it can't fly on its own + fight the stream --
// the npc/clump mirror discipline). The drone flies by forces on its root sphere (`coll`), which carries
// the actor, so actor-level TryGetActorLocation/SetActorLocation is the transform.
//
// Identity = SINGLETON (the world singleton's drone_C; both peers load the same placed drone). It has
// no top-level Key (its key lives inside Data) -- but a singleton needs no key. State we read: Active
// (dormant<->flying).

#pragma once

#include "ue_wrap/core/types.h"  // FVector, FRotator

namespace ue_wrap::drone {

// Resolve Adrone_C + the Active offset. Idempotent; true once resolved. Game thread.
bool EnsureResolved();

// The singleton delivery drone actor, or nullptr if not present: the world singleton's drone_C
// (world_singleton::Find, answered by the object index, never an array walk). Game thread.
void* Find();

// Active -- TRUE while the drone is flying a delivery (false = dormant/parked).
bool IsActive(void* drone);

// The drone's world transform (loc + full rotation -- it leans and pitches in flight), read from memory:
// its root component's relative location and rotation, which are its world ones while the root hangs
// from nothing -- always, for the delivery drone, which moves itself and carries the sack, never the
// other way round. An attached root is read through the engine instead. False on null or an unread
// pose. Game thread.
bool ReadPose(void* drone, FVector& loc, FRotator& rot);

// CLIENT mirror: snap the drone to the streamed transform. SetActorLocation + SetActorRotation.
bool DriveMirror(void* drone, const FVector& loc, const FRotator& rot);

// CLIENT mirror: stop the drone's own ReceiveTick (its flight integrator) so it can't fly on its
// own + fight the host stream -- the client drone is ALWAYS a mirror. Idempotent at the engine
// level. Game thread.
void SuppressTick(void* drone);

// Restore the drone's ReceiveTick (on disconnect, so single-player flight works again). Game thread.
void RestoreTick(void* drone);

// ---- FX mirroring ----------------------------------------------------------------------------
// The CLIENT suppresses the drone's ReceiveTick, which also kills the tick-driven FX: the
// rotor-wash DUST (eff_droneDust) and the delivery "items ready" alarm cue (audio_alarm +
// light_alarm). The drone's tick sphere-traces 2000uu down each tick and
// (1) SetActive(hit, false) on an IsActive!=hit edge,
// (2) K2_SetWorldLocation's eff_droneDust to the ground hit -- the component is bAbsoluteLocation
// with NO relative offset, so unmoved it sits at WORLD ORIGIN (invisible: frustum-culled fixed
// bounds), and (3) SetFloatParameter('dust', 1 - dist/2000) (a [0,1] RateScale). The PS is
// EmitterLoops=1/20s -- it self-completes and needs the per-tick IsActive!=want re-arm. So the
// HOST streams the FX bits + the dust component's world location in DroneState (about 20 Hz while
// Active or the dust is on, and as they change) and the CLIENT replays the BP's exact three calls per packet (ApplyDustMirror). The
// alarm cue fires on the canTakeOff false->true edge (drone within 25cm of its drop).

// stateBits packing (DroneStatePayload.stateBits): bit0 = dust active, bit1 = canTakeOff (arrived /
// the interaction gate), bit2 = hasSack (a physically attached sack), bit3 = leaveAfter5min,
// bit4 = sack actor present, bit5 = sack physically attached, bit6 = sack world pose valid,
// bit7 = sack root simulates physics.
inline constexpr uint8_t kFxDust     = 0x01;
inline constexpr uint8_t kFxArrived  = 0x02;
inline constexpr uint8_t kFxHasSack  = 0x04;
inline constexpr uint8_t kFxLeaveAfter5min = 0x08;
inline constexpr uint8_t kFxSackPresent = 0x10;  // the prop_dronesack_C actor exists on this peer
inline constexpr uint8_t kFxSackAttached = 0x20; // the sack actor is physically attached to the drone
inline constexpr uint8_t kFxSackPose = 0x40;     // the sack's world transform is valid in the packet
inline constexpr uint8_t kFxSackSimulating = 0x80; // host sack root currently simulates physics

// Drone's leave mode: true means leave after five minutes, false means wait indefinitely.
bool ReadLeaveAfter5min(void* drone, bool& out);
bool WriteLeaveAfter5min(void* drone, bool value);

// Find the drone's sack actor. Prefer an actor physically attached to the drone even when its
// Blueprint container pointer is stale; otherwise match the shared drone_InventoryContainer.
// Game thread.
void* FindSack(void* drone);

// Find another sack physically mounted on this drone, or left with an orphaned takenByDrone marker
// and no actor parent; the latter is a cleanup candidate only, not proof of gameplay attachment.
// `except` can skip a previously returned actor so stale local copies can be retired one by one. GT.
void* FindAttachedSack(void* drone, void* except = nullptr);

// Rebind a sack's opener field to the exact inventory actor used by the drone. This repairs the
// BeginPlay/key lookup on an already-existing or adopted sack so openPropInv remains usable.
// Game thread.
bool SetSackContainer(void* drone, void* sack);

// Read the sack's attachment state. The physical attach parent wins whenever the engine can
// read it, so a stale takenByDrone flag cannot leave a phantom drone interaction enabled. Game thread.
bool IsSackAttached(void* drone, void* sack);

// CLIENT mirror: apply the host's state to the EXACT sack actor resolved by the host element id/key.
// The caller must not pass an actor found only by the shared inventory-container pointer, since
// multiple local/remote sack actors can reference that same container during a drop transition.
// Returns false when a present actor is not available yet; retry on a later tick. Game thread.
bool ApplySackMirror(void* drone, void* sack, bool present, bool attached, bool simulatePhysics,
                     bool poseValid, const FVector& loc, const FRotator& rot);

// HOST: read the synced drone state into the packed stateBits byte:
// FX, arrival gate, leaveAfter5min and sack actor state. A sack that exists but is not physically
// attached does not satisfy the interaction gate. Game thread.
uint8_t ReadFxBits(void* drone);

// HOST: read eff_droneDust's world location (the BP pins it to its ground-trace hit per tick) for
// the DroneState stream. False if the FX refs / the component are unavailable. Game thread.
bool ReadDustAnchor(void* drone, FVector& out);

// CLIENT mirror: replay the BP's per-tick dust update from the streamed state -- SetActive on an
// IsActive!=on edge (also re-arms the self-completing 20s system), pin the component to `anchor`,
// write the 'dust' intensity (1 - dist/2000). Call per received DroneState packet. Game thread.
void ApplyDustMirror(void* drone, bool on, const FVector& anchor);

// CLIENT mirror: play the delivery "items ready" alarm cue once (audio_alarm.Activate). Call on the
// canTakeOff rising edge. Game thread.
void PlayArrivalCue(void* drone);

// CLIENT mirror: show/hide the arrival signal light (light_alarm). On while parked-with-cargo. GT.
void SetSignalLight(void* drone, bool on);

// ---- the drone's own sack visual ------------------------------------------------------------------
// The sack the drone carries while parked is not the prop actor: it is the drone BP's own apparatus
// (sack / sackPhys / sackHitbox / sackConstraint / sackInteract over the dronesack mesh), which
// dropSack hides in the same breath as it spawns the ground prop. A client cancels that body, so its
// copy keeps the apparatus up after every drop -- a visible duplicate with no actor behind it. The
// host's word that means "a sack is aboard" is hasSack (the mounted state has no actor at all), so
// this takes the visual from that word instead, as the dust and signal light take theirs.

// Hide/show the drone's own sack apparatus: every SceneComponent the drone owns whose name carries
// "sack", plus a drone-owned sack child actor (a ChildActorComponent's own actor is not hidden by
// hiding the component, so the actor itself is hidden as well). False while nothing matches -- the
// drone's construction script may not have instantiated the apparatus yet, and the caller retries.
// Game thread.
bool SetSackVisual(void* drone, bool on);

// CLIENT mirror: write the interaction-gate fields (canTakeOff, hasSack) onto the
// mirror drone so a PARKED drone is interactable (else the gate prints "drone is in motion" + offers
// no options -- the suppressed tick never sets them). Game thread.
void WriteGateFields(void* drone, bool canTakeOff, bool hasSack);

// Whether the drone is parked with its sack physically attached. Does not trust a stale hasSack
// Blueprint byte and does not depend on the optional FX components. Game thread.
bool CanDropSack(void* drone);
// HOST post-drop reconciliation: release any stale drone-owned sack actors, restore their query
// collision and clear the Blueprint's stale hasSack interaction gate. Game thread.
void NormalizeDroppedSack(void* drone);

// Run the drone's own dropSack Blueprint verb. The multiplayer intent lane calls this on the host
// after checking the sender's reach and the drone's state. Game thread.
bool DropSack(void* drone);

// Point the drone at the exact container also opened by its sack. False until the keyed actor is
// available or when it is not a drone inventory container. Game thread.
bool SetContainer(void* drone, void* container);

// The container (Aprop_inventoryContainer_drone_C) the drone's `container` field names, or null when the field
// is null or dead. Game thread.
void* ContainerOf(void* drone);

// The drone a droneSellLocation_C instance names through its `drone` field, or null when it is null or dead.
// Game thread.
void* OfSellLocation(void* sellLocation);

}  // namespace ue_wrap::drone
