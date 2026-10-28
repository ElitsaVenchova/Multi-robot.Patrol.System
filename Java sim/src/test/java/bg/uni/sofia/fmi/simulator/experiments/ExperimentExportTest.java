package bg.uni.sofia.fmi.simulator.experiments;

import bg.uni.sofia.fmi.simulator.config.*;
import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.results.*;
import org.junit.*;
import org.junit.rules.TemporaryFolder;
import java.nio.file.*;
import java.util.*;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import static org.junit.Assert.*;

public class ExperimentExportTest {
    @Rule public TemporaryFolder folder = new TemporaryFolder();

    @Test public void snapshotReplayPreservesAllMetricsAndRobotAccounting() throws Exception {
        var plan = shortPlan(40);
        var base = new ConfigLoader().load(plan.getBaseConfig());
        base.getPerimeter().setSize(20);
        base.getSimulation().setChargeThreshold(1);
        base.getAttackModel().setDuration(3);
        base.getPatrolModel().setDeviationProbability(0.25);
        base.getPatrolModel().setMaxDeviationDuration(3);
        var model = new RobotModelLoader().load("turtlebot_pi");
        model.getBattery().setBatteryCapacity(2);
        model.getBattery().setCycleLife(1000.0);
        model.getBattery().setReplacementCostEur(20.0);
        model.setName("embedded, \"robot\"\nfixture");
        base.getRobotModels().put("turtlebot_pi", model);
        var station = new ChargingStationConfig(); station.setModel("cheap_station"); station.setX(5);
        base.setChargingStations(List.of(station));
        Files.writeString(Path.of(plan.getBaseConfig()), ConfigLoader.toYaml(base));
        plan.getDesigns().get(0).getPoints().get(0).setAttackProbability(0.4);
        List<SimulationMetrics> originals = new ArrayList<>();
        var runner = new ExperimentRunner(config -> {
            var metrics = new SimulationRunner().runWithResult(config, false);
            originals.add(metrics);
            return metrics;
        });
        Path batch = runner.runBatch(plan, folder.getRoot().toPath());
        var rows = readCsv(batch.resolve("experiment_runs.csv"));
        var robotRows = readCsv(batch.resolve("experiment_robot_states.csv"));
        assertEquals(2, rows.size());
        assertEquals(4, robotRows.size());
        for (int i = 0; i < rows.size(); i++) {
            var row = rows.get(i);
            Path snapshot = batch.resolve(row.get("configuration_path"));
            String hash = HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(Files.readAllBytes(snapshot)));
            assertEquals(hash, row.get("configuration_sha256"));
            var config = new ConfigLoader().load(snapshot.toString());
            assertEquals(42L + i, (long) config.getSimulation().getSeed());
            assertEquals(2, config.getRobotModels().get("turtlebot_pi").getBattery().getBatteryCapacity(), 0);
            var replay = new SimulationRunner().runWithResult(config, false);
            assertEquals(MetricsAggregator.experimentValues(originals.get(i)), MetricsAggregator.experimentValues(replay));
            assertEquals(originals.get(i).getRobots(), replay.getRobots());
            assertTrue("Fixture must exercise charging", replay.getRobots().stream().mapToDouble(RobotMetrics::chargedWh).sum() > 0);
            for (var robot : replay.getRobots()) {
                assertEquals(40, robot.totalMinutes());
                assertEquals("embedded, \"robot\"\nfixture", robot.name());
                var exported = robotRows.stream().filter(r -> r.get("run_index").equals(row.get("run_index"))
                        && r.get("robot_id").equals(Long.toString(robot.id()))).findFirst().orElseThrow();
                assertEquals(robot.name(), exported.get("robot_name"));
                assertEquals("turtlebot_pi", exported.get("robot_model"));
                assertEquals(row.get("seed"), exported.get("seed"));
                assertNumber(exported, "total_min", 40);
                assertNumber(exported, "charged_wh", robot.chargedWh());
                assertNumber(exported, "discharged_wh", robot.dischargedWh());
                assertNumber(exported, "equivalent_discharge_cycles", robot.equivalentDischargeCycles());
                assertNumber(exported, "battery_wear_eur", robot.batteryWearEur());
                for (var state : BotState.values()) assertNumber(exported,
                        state.name().toLowerCase(Locale.ROOT) + "_min", robot.stateMinutes().getOrDefault(state, 0L));
            }
            for (var metric : MetricsAggregator.experimentValues(replay).entrySet())
                assertNumber(row, metric.getKey(), metric.getValue());
        }
    }

    @Test public void exportedColumnsRetainKnownValuesAndUndefinedStatistics() throws Exception {
        var plan = shortPlan(8);
        var prepared = new ExperimentRunner().prepare(plan);
        var first = new SimulationMetrics();
        first.setTotalAttacks(10); first.setInterceptedAttacks(2); first.setMissedAttacks(3); first.setActiveAttacks(5);
        first.setCaptureCoefficient(0.4); first.setAverageDetectionTime(3); first.setActivityCoefficient(3);
        first.setRobotAcquisitionCost(2000); first.setStationAcquisitionCost(1000);
        first.setEnergyCost(0.0075); first.setBatteryReplacementCost(0.15);
        first.setRobots(List.of(
                new RobotMetrics(1, "first, \"quoted\"\nrobot", "GROUND", Map.of(BotState.PATROLLING, 6L,
                        BotState.WAITING_FOR_CHARGE, 2L), 10, 25, 0.25, 0.05),
                new RobotMetrics(2, "second", "GROUND", Map.of(BotState.PATROLLING, 6L,
                        BotState.GOING_TO_CHARGE, 2L), 20, 50, 0.5, 0.1)));
        var second = new SimulationMetrics(); second.setAverageDetectionTime(Double.NaN);
        second.setRobotAcquisitionCost(2000); second.setStationAcquisitionCost(1000);
        second.setRobots(List.of(
                new RobotMetrics(1, "first", "GROUND", Map.of(BotState.WAITING_FOR_CHARGE, 8L), 0, 0, 0, Double.NaN),
                new RobotMetrics(2, "second", "GROUND", Map.of(BotState.WAITING_FOR_CHARGE, 8L), 0, 0, 0, Double.NaN)));
        var results = List.of(new ExperimentRunner.CompletedRun(prepared.get(0), first),
                new ExperimentRunner.CompletedRun(prepared.get(1), second));
        Path batch = folder.getRoot().toPath().resolve("known_export");
        try (var output = new ResultExporter.ExperimentOutput(batch, plan)) {
            for (var result : results) output.write(result);
            output.finish(new MetricsAggregator().aggregateExperiments(results));
        }
        var rows = readCsv(batch.resolve("experiment_runs.csv"));
        Map<String, Double> expected = Map.ofEntries(
                Map.entry("capture_coefficient", 0.4), Map.entry("activity_coefficient", 3.0),
                Map.entry("detection_fraction_generated", 0.2), Map.entry("detection_time_min", 3.0),
                Map.entry("generated_attacks", 10.0), Map.entry("intercepted_attacks", 2.0),
                Map.entry("missed_attacks", 3.0), Map.entry("active_attacks", 5.0), Map.entry("resolved_attacks", 5.0),
                Map.entry("robot_acquisition_eur", 2000.0), Map.entry("station_acquisition_eur", 1000.0),
                Map.entry("energy_eur", 0.0075), Map.entry("battery_replacement_eur", 0.15),
                Map.entry("tco_eur", 3000.1575), Map.entry("charged_wh", 30.0), Map.entry("discharged_wh", 75.0),
                Map.entry("equivalent_discharge_cycles", 0.75), Map.entry("waiting_for_charge_min", 2.0),
                Map.entry("patrolling_min", 12.0), Map.entry("going_to_patrol_min", 0.0),
                Map.entry("going_to_charge_min", 2.0), Map.entry("charging_min", 0.0), Map.entry("error_min", 0.0));
        for (var metric : expected.entrySet()) assertNumber(rows.get(0), metric.getKey(), metric.getValue());
        assertEquals("1", rows.get(0).get("schema_version"));
        assertEquals("test", rows.get(0).get("experiment_id"));
        assertEquals("42", rows.get(0).get("seed")); assertEquals("43", rows.get(1).get("seed"));
        assertEquals("8", rows.get(0).get("duration_min"));
        for (String metric : List.of("capture_coefficient", "activity_coefficient", "detection_fraction_generated",
                "detection_time_min", "energy_eur", "battery_replacement_eur", "tco_eur")) assertNumber(rows.get(1), metric, Double.NaN);
        var robotRows = readCsv(batch.resolve("experiment_robot_states.csv"));
        assertEquals("first, \"quoted\"\nrobot", robotRows.get(0).get("robot_name"));
        assertNumber(robotRows.get(0), "charged_wh", 10);
        assertNumber(robotRows.get(0), "battery_wear_eur", 0.05);
        var summary = readCsv(batch.resolve("experiment_summary.csv")).get(0);
        assertNumber(summary, "runs", 2);
        assertNumber(summary, "valid_capture_coefficient_runs", 1);
        assertNumber(summary, "mean_capture_coefficient", 0.4);
        assertNumber(summary, "std_capture_coefficient", 0);
        assertNumber(summary, "valid_detection_time_min_runs", 1);
        assertNumber(summary, "mean_detection_time_min", 3);
        assertNumber(summary, "valid_tco_eur_runs", 1);
        assertNumber(summary, "mean_tco_eur", 3000.1575);
        assertNumber(summary, "std_tco_eur", 0);
        assertNumber(summary, "valid_generated_attacks_runs", 2);
        assertNumber(summary, "mean_generated_attacks", 5);
        assertNumber(summary, "std_generated_attacks", 5);
        assertEquals("COMPLETE", Files.readString(batch.resolve("status.txt")).strip());
    }

    @Test public void captureAggregationUsesRunRatiosRatherThanPooledEvents() {
        var prepared = new ExperimentRunner().prepare(ExperimentRunnerTest.plan());
        var a = new SimulationMetrics(); a.setTotalAttacks(1); a.setInterceptedAttacks(1); a.setCaptureCoefficient(1);
        var b = new SimulationMetrics(); b.setTotalAttacks(9); b.setMissedAttacks(9); b.setCaptureCoefficient(0);
        var summaries = new MetricsAggregator().aggregateExperiments(List.of(
                new ExperimentRunner.CompletedRun(prepared.get(0), a), new ExperimentRunner.CompletedRun(prepared.get(1), b)));
        assertEquals(1, summaries.size());
        var capture = summaries.get(0).statistics().get("capture_coefficient");
        assertEquals(2, capture.count()); assertEquals(0.5, capture.mean(), 0); assertEquals(0.5, capture.std(), 0);
        // Pooling would report 1 / (1 + 9) = 0.1 instead.
        var undefined = summaries.get(0).statistics().get("energy_eur");
        assertEquals(0, undefined.count()); assertTrue(Double.isNaN(undefined.mean())); assertTrue(Double.isNaN(undefined.std()));
    }

    private ExperimentConfig shortPlan(int duration) throws Exception {
        var plan = ExperimentRunnerTest.plan();
        plan.setEnergyPriceEurPerKwh(0.25);
        plan.getDesigns().get(0).setPoints(List.of(plan.getDesigns().get(0).getPoints().get(0)));
        var base = new ConfigLoader().load(plan.getBaseConfig()); base.getSimulation().setDuration(duration);
        Path path = folder.newFile("base.yaml").toPath();
        Files.writeString(path, ConfigLoader.toYaml(base)); plan.setBaseConfig(path.toString());
        return plan;
    }

    private static void assertNumber(Map<String, String> row, String column, double expected) {
        assertTrue("Missing column " + column, row.containsKey(column));
        if (Double.isNaN(expected)) assertEquals(column, "", row.get(column));
        else assertEquals(column, expected, Double.parseDouble(row.get(column)), 1e-10);
    }

    /** Read quoted CSV, including escaped quotes, embedded newlines, and empty fields. */
    private static List<Map<String, String>> readCsv(Path path) throws Exception {
        String csv = Files.readString(path, StandardCharsets.UTF_8);
        List<List<String>> records = new ArrayList<>();
        List<String> record = new ArrayList<>();
        StringBuilder field = new StringBuilder();
        boolean quoted = false;
        for (int i = 0; i < csv.length(); i++) {
            char c = csv.charAt(i);
            if (c == '"') {
                if (quoted && i + 1 < csv.length() && csv.charAt(i + 1) == '"') { field.append('"'); i++; }
                else quoted = !quoted;
            } else if (!quoted && (c == ',' || c == '\n')) {
                record.add(field.toString()); field.setLength(0);
                if (c == '\n') { records.add(record); record = new ArrayList<>(); }
            } else if (quoted || c != '\r') field.append(c);
        }
        assertFalse("Unclosed CSV quote", quoted);
        if (!record.isEmpty() || field.length() > 0) { record.add(field.toString()); records.add(record); }
        List<String> header = records.remove(0);
        assertEquals("Duplicate CSV columns", header.size(), new HashSet<>(header).size());
        List<Map<String, String>> rows = new ArrayList<>();
        for (var values : records) {
            assertEquals("CSV row width", header.size(), values.size());
            Map<String, String> row = new LinkedHashMap<>();
            for (int i = 0; i < header.size(); i++) row.put(header.get(i), values.get(i));
            rows.add(row);
        }
        return rows;
    }

    @Test public void batchHasReplayableSnapshotsCostsAndRefusesOverwrite() throws Exception {
        var plan = ExperimentRunnerTest.plan();
        plan.setEnergyPriceEurPerKwh(0.25);
        // Preserve the configured duration in a short base file for the integration test.
        var base = new ConfigLoader().load(plan.getBaseConfig()); base.getSimulation().setDuration(12);
        var path = folder.getRoot().toPath().resolve("base.yaml"); Files.writeString(path, ConfigLoader.toYaml(base)); plan.setBaseConfig(path.toString());
        var batch = new ExperimentRunner().runBatch(plan, folder.getRoot().toPath());
        assertEquals("COMPLETE", Files.readString(batch.resolve("status.txt")).strip());
        assertEquals(5, Files.readAllLines(batch.resolve("experiment_runs.csv")).size());
        assertEquals(3, Files.readAllLines(batch.resolve("experiment_summary.csv")).size());
        var snapshot = batch.resolve("configs/ground/p1/run-1.yaml");
        var replay = new SimulationRunner().runWithResult(new ConfigLoader().load(snapshot.toString()), false);
        assertEquals(12, replay.getTotalAttacks()); assertEquals(2, replay.getRobots().size());
        assertEquals(43L, (long) new ConfigLoader().load(snapshot.toString()).getSimulation().getSeed());
        assertTrue(Files.readString(batch.resolve("experiment_runs.csv")).contains(ResultExporter.ExperimentOutput.sha256(Files.readString(snapshot))));
        try { new ExperimentRunner().runBatch(plan, folder.getRoot().toPath()); fail("Must refuse existing batch"); }
        catch (IllegalArgumentException expected) { assertTrue(expected.getMessage().contains("Cannot write")); }
    }

    @Test public void aggregationExcludesUndefinedAndSeparatesMismatchedParameters() {
        var prepared = new ExperimentRunner().prepare(ExperimentRunnerTest.plan());
        SimulationMetrics a = new SimulationMetrics(), b = new SimulationMetrics();
        a.setCaptureCoefficient(0.25); b.setCaptureCoefficient(Double.NaN);
        var results = List.of(new ExperimentRunner.CompletedRun(prepared.get(0), a),
                new ExperimentRunner.CompletedRun(prepared.get(1), b), new ExperimentRunner.CompletedRun(prepared.get(2), a));
        var summaries = new MetricsAggregator().aggregateExperiments(results);
        assertEquals(2, summaries.size()); assertEquals(2, summaries.get(0).runs());
        var capture = summaries.get(0).statistics().get("capture_coefficient");
        assertEquals(1, capture.count()); assertEquals(0.25, capture.mean(), 0); assertEquals(0, capture.std(), 0);
    }

    @Test public void allSystemsHaveTheExpectedCompositionAndConsistentTco() {
        var plan = ConfigLoader.read("configs/experiments/illustrative.yaml", ExperimentConfig.class);
        var prepared = new ExperimentRunner().prepare(plan);
        assertEquals(90, prepared.size());
        Set<ExperimentConfig.SystemType> systems = new HashSet<>();
        for (var run : prepared) if (run.type() == ExperimentConfig.Type.TCO && systems.add(run.system())) {
            var config = run.configuration(); config.getSimulation().setDuration(10);
            var metrics = new SimulationRunner().runWithResult(config, false);
            assertEquals(config.getRobots().stream().mapToInt(RobotConfig::getCount).sum(), metrics.getRobots().size());
            double robotCost = config.getRobots().stream().mapToDouble(r -> r.getCount() * config.getRobotModels().get(r.getModel()).getPrice()).sum();
            assertEquals(robotCost, metrics.getRobotAcquisitionCost(), 0);
            double wear = metrics.getRobots().stream().mapToDouble(RobotMetrics::batteryWearEur).sum();
            assertEquals(wear, metrics.getBatteryReplacementCost(), 0);
            assertEquals(robotCost + metrics.getStationAcquisitionCost() + metrics.getEnergyCost() + wear, metrics.getTco(), 1e-12);
        }
        assertEquals(3, systems.size());
    }
}
