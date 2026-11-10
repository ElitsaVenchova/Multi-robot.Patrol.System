package bg.uni.sofia.fmi.simulator.behavior.communication.messages;

/**
 * An immutable message payload. Implementations must defensively copy mutable values.
 * Адресирането, времето за доставка и етапите на транспортиране са част от обвивката.
 */
public interface CommunicationMessage { }