package bg.uni.sofia.fmi.simulator.behavior.communication;

import bg.uni.sofia.fmi.simulator.domain.Bot;

/**
 * Robot-local message processing, separate from transport and patrol execution.
 * Simulator rounds replace event callbacks for now; handlers never coordinate other robots.
 */
public interface CommunicationParticipant {
    void onCommunicationRound(Bot bot, int currentTime, CommunicationController.Round round,
                              CommunicationController communication);
}