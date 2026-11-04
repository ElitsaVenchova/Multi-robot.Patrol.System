package bg.uni.sofia.fmi.simulator.behavior.communication.messages;

/**
 * Discovery data sent only to locally LiDAR-detected peers. Displacement is the signed
 * movement intended for this tick, before an encounter response or obstacle avoidance.
 * The counter advances only after a committed encounter. rangeExitObserved is specific
 * to the recipient: the sender observed a range exit since their last committed encounter.
 * Identity and observation time are supplied by the envelope.
 */
public record EncounterMotionMessage(double intendedDisplacement, long completedEncounterCount,
                                     boolean rangeExitObserved) implements CommunicationMessage {
    public EncounterMotionMessage {
        if (!Double.isFinite(intendedDisplacement)) {
            throw new IllegalArgumentException("Intended displacement must be finite");
        }
        if (completedEncounterCount < 0) {
            throw new IllegalArgumentException("Encounter counter must be non-negative");
        }
    }
}