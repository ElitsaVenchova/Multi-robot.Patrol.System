package bg.uni.sofia.fmi.simulator.behavior.communication.messages;

import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;

import java.util.Objects;

/**
 * Snapshot of the state currently needed for encounter allocation.
 * PatrolSection protects its mutable Position endpoints with defensive copies.
 * Transport and negotiation metadata belong outside this state message.
 */
public record EncounterStateMessage(long senderId, double maxSpeed, PatrolSection patrolSection) {
    public EncounterStateMessage {
        Objects.requireNonNull(patrolSection, "Patrol section");
    }
}