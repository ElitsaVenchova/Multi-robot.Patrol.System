package bg.uni.sofia.fmi.simulator.behavior.communication.messages;

/**
 * An immutable message payload. Implementations must defensively copy mutable values.
 * Addressing, delivery time and transport phases belong to the envelope.
 */
public interface CommunicationMessage { }