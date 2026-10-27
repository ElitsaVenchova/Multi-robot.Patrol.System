package bg.uni.sofia.fmi.simulator.results;

import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import java.util.Map;

/** One-minute state counts and physical battery throughput for one robot. */
public record RobotMetrics(long id, String name, String type, Map<BotState, Long> stateMinutes,
        double chargedWh, double dischargedWh, double equivalentDischargeCycles, double batteryWearEur) {
    public RobotMetrics { stateMinutes = Map.copyOf(stateMinutes); }
    public long patrolMinutes() { return stateMinutes.getOrDefault(BotState.PATROLLING, 0L); }
    public long totalMinutes() { return stateMinutes.values().stream().mapToLong(Long::longValue).sum(); }
}
