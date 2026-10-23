package bg.uni.sofia.fmi.simulator.domain.enums;

/** Controls when a previously completed encounter may be repeated. */
public enum EncounterRearmPolicy {
    EXIT_EITHER_RANGE,
    AFTER_OTHER_ENCOUNTER
}