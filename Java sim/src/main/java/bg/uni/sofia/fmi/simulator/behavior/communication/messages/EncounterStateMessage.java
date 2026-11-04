package bg.uni.sofia.fmi.simulator.behavior.communication.messages;

import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;

import java.util.Objects;

/**
 * Addressed only to the selected peer. Reciprocal state messages establish a pair;
 * there is no separate selection/acceptance payload. Section endpoints are defensive copies.
 */
public record EncounterStateMessage(long senderId, double maxSpeed, PatrolSection patrolSection)
        implements CommunicationMessage {
    public EncounterStateMessage {
        Objects.requireNonNull(patrolSection, "Patrol section");
        if (!Double.isFinite(maxSpeed) || maxSpeed <= 0) {
            throw new IllegalArgumentException("Maximum speed must be finite and positive");
        }
    }
}