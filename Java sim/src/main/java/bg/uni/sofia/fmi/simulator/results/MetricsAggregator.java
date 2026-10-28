package bg.uni.sofia.fmi.simulator.results;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.experiments.ExperimentRunner;
import bg.uni.sofia.fmi.simulator.util.MathUtils;

import java.util.*;

public class MetricsAggregator {
    public record Statistic(int count, double mean, double std) { }
    public record ExperimentSummary(ExperimentRunner.PreparedRun context,
                                    int runs, Map<String, Statistic> statistics) { }

    /** Per-run statistics; undefined values are excluded independently for each metric. */
    public static Statistic statistic(List<Double> values) {
        double[] finite = values.stream().mapToDouble(Double::doubleValue).filter(Double::isFinite).toArray();
        if (finite.length == 0) return new Statistic(0, Double.NaN, Double.NaN);
        double mean = Arrays.stream(finite).average().orElseThrow();
        double variance = Arrays.stream(finite).map(v -> (v - mean) * (v - mean)).average().orElseThrow();
        return new Statistic(finite.length, mean, Math.sqrt(variance));
    }

    public static Map<String, Double> experimentValues(SimulationMetrics m) {
        Map<String, Double> v = new LinkedHashMap<>();
        v.put("capture_coefficient", m.getCaptureCoefficient());
        v.put("activity_coefficient", m.getActivityCoefficient());
        v.put("detection_fraction_generated", m.getTotalAttacks() == 0 ? Double.NaN : (double) m.getInterceptedAttacks() / m.getTotalAttacks());
        v.put("detection_time_min", m.getAverageDetectionTime());
        v.put("generated_attacks", (double) m.getTotalAttacks());
        v.put("intercepted_attacks", (double) m.getInterceptedAttacks());
        v.put("missed_attacks", (double) m.getMissedAttacks());
        v.put("active_attacks", (double) m.getActiveAttacks());
        v.put("resolved_attacks", (double) m.getInterceptedAttacks() + m.getMissedAttacks());
        v.put("robot_acquisition_eur", m.getRobotAcquisitionCost());
        v.put("station_acquisition_eur", m.getStationAcquisitionCost());
        v.put("energy_eur", m.getEnergyCost());
        v.put("battery_replacement_eur", m.getBatteryReplacementCost());
        v.put("tco_eur", m.getTco());
        v.put("charged_wh", m.getRobots().stream().mapToDouble(RobotMetrics::chargedWh).sum());
        v.put("discharged_wh", m.getRobots().stream().mapToDouble(RobotMetrics::dischargedWh).sum());
        v.put("equivalent_discharge_cycles", m.getRobots().stream().mapToDouble(RobotMetrics::equivalentDischargeCycles).sum());
        for (var state : BotState.values())
            v.put(state.name().toLowerCase(Locale.ROOT) + "_min", (double) m.getRobots().stream()
                    .mapToLong(r -> r.stateMinutes().getOrDefault(state, 0L)).sum());
        return v;
    }

    public List<ExperimentSummary> aggregateExperiments(List<ExperimentRunner.CompletedRun> runs) {
        Map<String, List<ExperimentRunner.CompletedRun>> groups = new LinkedHashMap<>();
        for (var run : runs) {
            var c = run.context();
            var config = c.configuration(); config.getSimulation().setSeed(0L);
            String key = List.of(c.experimentId(), c.designId(), c.type(), c.system(), c.pointId(), c.resourceScenario()).toString()
                    + ConfigLoader.toYaml(config);
            groups.computeIfAbsent(key, k -> new ArrayList<>()).add(run);
        }
        List<ExperimentSummary> summaries = new ArrayList<>();
        for (var group : groups.values()) {
            Map<String, Statistic> stats = new LinkedHashMap<>();
            var values = group.stream().map(r -> experimentValues(r.metrics())).toList();
            for (String name : values.get(0).keySet())
                stats.put(name, statistic(values.stream().map(v -> v.get(name)).toList()));
            summaries.add(new ExperimentSummary(group.get(0).context(), group.size(), Collections.unmodifiableMap(stats)));
        }
        return List.copyOf(summaries);
    }

    public AggregatedMetrics aggregate(List<SimulationMetrics> metricsList) {
        AggregatedMetrics result = new AggregatedMetrics();
        int n = metricsList.size();
        result.setRuns(n);
        if (n == 0) return result;
        List<Double> successRates = new ArrayList<>(n);
        List<Double> detectionTimes = new ArrayList<>(n);
        for (SimulationMetrics m : metricsList) {
            successRates.add(m.getSuccessRate());
            detectionTimes.add(m.getAverageDetectionTime());
        }
        // ---- Means ----
        double meanSuccess = MathUtils.mean(successRates);
        double meanDetection = MathUtils.mean(detectionTimes);
        result.setMeanSuccessRate(meanSuccess);
        result.setMeanDetectionTime(meanDetection);
        // ---- Standard deviation ----
        double stdSuccess = MathUtils.std(successRates, meanSuccess);
        double stdDetection = MathUtils.std(detectionTimes, meanDetection);
        result.setStdSuccessRate(stdSuccess);
        result.setStdDetectionTime(stdDetection);
        return result;
    }
}
