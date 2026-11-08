// coop/interactables/drone_call_intent.h -- the garage console and drone sack, as client intents
// the host executes. Overview: docs/devices.md. Gameplay/network layer (principle 7); the
// engine is reached through ue_wrap::drone.
//
// The console's one action option dispatches on what the presser is looking at: its keyboard runs
// drone.triggerFly(console), and its other face toggles leaveAfter5min. The client sends those
// presses to the host. The same lane carries a request to remove the drone sack: a client cancels
// its local dropSack body, and the host checks reach/state before running that body once. The
// ordinary prop lanes carry the resulting sack to every peer.

#pragma once

#include <cstdint>

namespace coop::net {
class Session;
struct DroneFlyIntentPayload;
}  // namespace coop::net

namespace coop::drone_call_intent {

// Watch the console action and drone sack-drop verbs (client) and cache the session. Idempotent;
// retried until their names resolve. Game thread.
void Install(coop::net::Session* session);

// Per pump tick: keep the gates enabled for this session and run queued drone intents under the
// per-peer budget (host). Game thread.
void Tick(coop::net::Session& session);

// HOST: a client asks for a console press or sack drop. Queued here and performed from Tick under the rate limit.
void OnDroneFlyIntent(coop::net::Session& session, const coop::net::DroneFlyIntentPayload& payload,
                      uint8_t senderSlot);

// Session teardown: the counters are dropped. A sender's queue, its bucket and its said-once latch live in its
// per-slot state and go with its occupant, so the next occupant of the slot starts clean.
void OnDisconnect();

// The presses (either face) this client has sent the host (the drone call drill's proof that a press
// reached the lane's gate). Game thread.
uint64_t SentCount();

}  // namespace coop::drone_call_intent
