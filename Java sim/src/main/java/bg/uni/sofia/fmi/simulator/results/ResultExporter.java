package bg.uni.sofia.fmi.simulator.results;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.config.RobotConfig;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.experiments.ExperimentRunner;
import bg.uni.sofia.fmi.simulator.util.FileUtils;

// This class will handle exporting the results to a CSV file
// It will create a file "simulation_results.csv" in the "outputs/results" directory
public class ResultExporter {
    private static final String OUTPUT_DIR = "outputs/results/";
    private static final String FILE_NAME = "simulation_results.csv";

    public void export(SimulationMetrics metrics) {
        FileUtils.ensureDirectoryExists(OUTPUT_DIR);

        File file = new File(OUTPUT_DIR + FILE_NAME);
        boolean fileExists = file.exists();

        try (FileWriter writer = new FileWriter(file, true)) {

            // Write header only once
            if (!fileExists) {
                writer.append("total_attacks,intercepted,missed,success_rate,avg_detection_time\n");
            }

            // Write data row
            writer.append(String.valueOf(metrics.getTotalAttacks())).append(",");
            writer.append(String.valueOf(metrics.getInterceptedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getMissedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getAverageDetectionTime())).append("\n");

        } catch (IOException e) {
            throw new RuntimeException("Error writing results file", e);
        }
    }

    public void exportWithContext(SimulationMetrics metrics, SimulationConfig config) {

        FileUtils.ensureDirectoryExists(OUTPUT_DIR);

        File file = new File(OUTPUT_DIR + FILE_NAME);
        boolean fileExists = file.exists();

        try (FileWriter writer = new FileWriter(file, true)) {

            if (!fileExists) {
                writer.append("patrol_model,patrol_deviation_probability,patrol_deviation_duration,");
                writer.append("attack_model,attack_lambda,attack_duration,");
                writer.append("total_attacks,intercepted,missed,success_rate,avg_detection_time,avg_response_time\n");
            }

            writer.append(String.valueOf(config.getPatrolModel().getModel())).append(",");
            writer.append(String.valueOf(config.getPatrolModel().getDeviationProbability())).append(",");
            writer.append(String.valueOf(config.getPatrolModel().getMaxDeviationDuration())).append(",");

            writer.append(String.valueOf(config.getAttackModel().getModel())).append(",");
            writer.append(String.valueOf(config.getAttackModel().getLambda())).append(",");
            writer.append(String.valueOf(config.getAttackModel().getDuration())).append(",");

            writer.append(String.valueOf(metrics.getTotalAttacks())).append(",");
            writer.append(String.valueOf(metrics.getInterceptedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getMissedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getAverageDetectionTime())).append(",");
            writer.append(String.valueOf(metrics.getAverageResponseTime())).append("\n");

        } catch (IOException e) {
            throw new RuntimeException("Error writing results file", e);
        }
    }

    public void exportAggregated(AggregatedMetrics metrics,
            String strategy,
            double lambda) {

        FileUtils.ensureDirectoryExists(OUTPUT_DIR);

        File file = new File(OUTPUT_DIR + "aggregated_results.csv");
        boolean fileExists = file.exists();

        try (FileWriter writer = new FileWriter(file, true)) {

            if (!fileExists) {
                writer.append("strategy,lambda,runs,mean_success,std_success,mean_detection,std_detection\n");
            }

            writer.append(strategy).append(",");
            writer.append(String.valueOf(lambda)).append(",");
            writer.append(String.valueOf(metrics.getRuns())).append(",");
            writer.append(String.valueOf(metrics.getMeanSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getStdSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getMeanDetectionTime())).append(",");
            writer.append(String.valueOf(metrics.getStdDetectionTime())).append("\n");

        } catch (IOException e) {
            throw new RuntimeException("Error writing aggregated results", e);
        }
    }

    /** Отделните пакетни файлове никога не добавят данни към съществуващия CSV файл,
     предназначени за еднократно изпълнение. */
    public static final class ExperimentOutput implements AutoCloseable {
        private final Path directory;
        private final BufferedWriter runs, robots;
        private boolean finished;
        private boolean runHeader, robotHeader;
        /** Package-private seam for deterministic I/O-failure tests. */
        @FunctionalInterface interface WriterOpener {
            BufferedWriter open(Path path) throws IOException;
        }
        public ExperimentOutput(Path directory, Object plan) throws IOException {
            this(directory, plan, Files::newBufferedWriter);
        }
        ExperimentOutput(Path directory, Object plan, WriterOpener opener) throws IOException {
            this.directory = directory;
            Files.createDirectories(directory.getParent());
            Files.createDirectory(directory); // Refuse overwrite, including incomplete batches.
            BufferedWriter openedRuns = null;
            try {
                Files.writeString(directory.resolve("experiment.yaml"), ConfigLoader.toYaml(plan));
                Files.writeString(directory.resolve("status.txt"), "RUNNING\n");
                Files.writeString(directory.resolve("runtime.txt"), "java=" + System.getProperty("java.version")
                        + "\nschema=1\ntick_minutes=1\ncharging_efficiency=1\nstatistic=per_run_population_standard_deviation\n"
                        + "seed_design=shared_seed_not_aligned_random_streams\n");
                archiveImplementation();
                openedRuns = opener.open(directory.resolve("experiment_runs.csv"));
                robots = opener.open(directory.resolve("experiment_robot_states.csv"));
                runs = openedRuns;
            } catch (IOException | RuntimeException | Error failure) {
                if (openedRuns != null) {
                    try { openedRuns.close(); }
                    catch (IOException | RuntimeException | Error cleanup) {
                        if (cleanup != failure) failure.addSuppressed(cleanup);
                    }
                }
                recordFailure(failure);
                throw failure;
            }
        }

        private void archiveImplementation() throws IOException {
            Path archive = null;
            if (!Files.isDirectory(Path.of("src/main/java"))){
                archive = directory.resolve("implementation.zip");
            }
            assert archive != null;
            try (ZipOutputStream zip = new ZipOutputStream(Files.newOutputStream(archive))) {
                List<Path> paths = new ArrayList<>();
                for (String root : List.of("src", "scripts", "docs")) {
                    Path pathOfRoot = Path.of(root);
                    if (!Files.isDirectory(pathOfRoot)) continue;
                    try (Stream<Path> walk = Files.walk(pathOfRoot)) {
                        walk.filter(Files::isRegularFile)
                                .filter(p -> p.toString().matches(".*\\.(java|yaml|py|txt|md)$")).forEach(paths::add);
                    }
                }
                paths.add(Path.of("pom.xml"));
                paths.sort(Comparator.comparing(Path::toString));
                for (var path : paths) {
                    var entry = new ZipEntry(path.toString().replace('\\', '/')); entry.setTime(0);
                    zip.putNextEntry(entry); Files.copy(path, zip); zip.closeEntry();
                }
            }
        }

        public void write(ExperimentRunner.CompletedRun result) throws IOException {
            var c = result.context();
            String relative = "configs/" + c.designId() + "/" + c.pointId() + "/run-" + c.repetition() + ".yaml";
            var path = directory.resolve(relative);
            Files.createDirectories(path.getParent());
            Files.writeString(path, c.configurationYaml());
            var row = context(c);
            row.put("run_index", c.repetition()); row.put("seed", c.seed());
            row.put("configuration_path", relative); row.put("configuration_sha256", sha256(c.configurationYaml()));
            row.putAll(MetricsAggregator.experimentValues(result.metrics()));
            if (!runHeader) { csv(runs, row.keySet()); runHeader = true; }
            csv(runs, row.values()); runs.flush();
            var config = c.configuration();
            List<String> modelNames = new ArrayList<>();
            for (var r : config.getRobots()) for (int i = 0; i < r.getCount(); i++) modelNames.add(r.getModel());
            int index = 0;
            for (var robot : result.metrics().getRobots()) {
                var robotRow = new LinkedHashMap<String, Object>();
                robotRow.put("experiment_id", c.experimentId()); robotRow.put("design_id", c.designId());
                robotRow.put("point_id", c.pointId()); robotRow.put("run_index", c.repetition()); robotRow.put("seed", c.seed());
                robotRow.put("robot_id", robot.id()); robotRow.put("robot_name", robot.name()); robotRow.put("robot_type", robot.type());
                robotRow.put("robot_model", modelNames.get(index++));
                for (var state : BotState.values())
                    robotRow.put(state.name().toLowerCase(Locale.ROOT) + "_min", robot.stateMinutes().getOrDefault(state, 0L));
                robotRow.put("total_min", robot.totalMinutes());
                robotRow.put("charged_wh", robot.chargedWh()); robotRow.put("discharged_wh", robot.dischargedWh());
                robotRow.put("equivalent_discharge_cycles", robot.equivalentDischargeCycles()); robotRow.put("battery_wear_eur", robot.batteryWearEur());
                if (!robotHeader) { csv(robots, robotRow.keySet()); robotHeader = true; }
                csv(robots, robotRow.values());
            }
            robots.flush();
        }

        public void finish(List<MetricsAggregator.ExperimentSummary> summaries) throws IOException {
            try (var writer = Files.newBufferedWriter(directory.resolve("experiment_summary.csv"))) {
                boolean header = false;
                for (var summary : summaries) {
                    var row = context(summary.context()); row.put("runs", summary.runs());
                    for (var item : summary.statistics().entrySet()) {
                        row.put("valid_" + item.getKey() + "_runs", item.getValue().count());
                        row.put("mean_" + item.getKey(), item.getValue().mean());
                        row.put("std_" + item.getKey(), item.getValue().std());
                    }
                    if (!header) { csv(writer, row.keySet()); header = true; }
                    csv(writer, row.values());
                }
            }
            finished = true;
        }
        private static LinkedHashMap<String, Object> context(ExperimentRunner.PreparedRun r) {
            var config = r.configuration();
            var row = new LinkedHashMap<String, Object>();
            row.put("schema_version", 1); row.put("experiment_id", r.experimentId()); row.put("design_id", r.designId());
            row.put("experiment_type", r.type()); row.put("system_type", r.system()); row.put("point_id", r.pointId());
            row.put("resource_scenario", r.resourceScenario()); row.put("master_seed", r.masterSeed());
            row.put("patrol_strategy", config.getPatrolModel().getModel());
            row.put("patrol_deviation_probability", config.getPatrolModel().getDeviationProbability());
            row.put("patrol_max_deviation_duration_min", config.getPatrolModel().getMaxDeviationDuration());
            row.put("encounter_rearm_policy", config.getPatrolModel().getEncounterRearmPolicy());
            row.put("charge_threshold_wh", config.getSimulation().getChargeThreshold());
            row.put("robot_configuration", config.getRobots().stream().map(b -> b.getModel() + ":" + b.getCount()).collect(Collectors.joining(";")));
            row.put("station_configuration", config.getChargingStations().stream().map(s -> s.getModel() + "@" + s.getX() + ":" + s.getY()).collect(Collectors.joining(";")));
            row.put("battery_configuration", config.getRobots().stream().map(b -> {
                var battery = config.getRobotModels().get(b.getModel()).getBattery();
                return b.getModel() + ":capacityWh=" + battery.getBatteryCapacity() + ":cycleLife=" + battery.getCycleLife()
                        + ":replacementEur=" + battery.getReplacementCostEur();
            }).collect(Collectors.joining(";")));
            row.put("station_model_configuration", config.getChargingStations().stream().map(s -> {
                var model = config.getStationModels().get(s.getModel());
                return s.getModel() + ":slots=" + model.getSlots() + ":powerWPerSlot=" + model.getPower() + ":priceEur=" + model.getPrice();
            }).collect(Collectors.joining(";")));
            row.put("robot_count", config.getRobots().stream().mapToInt(RobotConfig::getCount).sum());
            row.put("patrol_density_robots_per_m", config.getRobots().stream().mapToInt(RobotConfig::getCount).sum() / (double) config.getPerimeter().getSize());
            row.put("attack_model", config.getAttackModel().getModel());
            row.put("attack_probability_per_tick", config.getAttackModel().getLambda());
            row.put("attack_duration_min", config.getAttackModel().getDuration());
            row.put("duration_min", config.getSimulation().getDuration()); row.put("perimeter_length_m", config.getPerimeter().getSize());
            row.put("perimeter_type", config.getPerimeter().getType()); row.put("energy_price_eur_per_kwh", config.getEnergyPriceEurPerKwh());
            row.put("cost_model", "acquisition_charged_energy_discharge_cycle_wear_v1");
            row.put("capture_definition", "intercepted_over_resolved");
            return row;
        }
        public static String sha256(String value) {
            try {
                return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256")
                        .digest(value.getBytes(StandardCharsets.UTF_8)));
            } catch (NoSuchAlgorithmException e) { throw new IllegalStateException(e); }
        }
        /** RFC 4180 quoting; undefined floating-point values are empty fields. */
        private static void csv(Writer writer, Collection<?> fields) throws IOException {
            List<String> values = new ArrayList<>();
            for (Object field : fields) {
                String text = field == null || (field instanceof Double && !Double.isFinite((Double) field)) ? "" : field.toString();
                values.add("\"" + text.replace("\"", "\"\"") + "\"");
            }
            writer.write(String.join(",", values) + "\n");
        }
        @Override public void close() throws IOException {
            try {
                Files.writeString(directory.resolve("status.txt"), finished ? "COMPLETE\n" : "FAILED\n");
            } catch (IOException | RuntimeException | Error failure) {
                recordFailure(failure);
                throw failure;
            }
        }
        private void recordFailure(Throwable failure) {
            finished = false; // A later cleanup call must not publish a failed batch as complete.
            try { Files.writeString(directory.resolve("status.txt"), "FAILED\n"); }
            catch (IOException | RuntimeException | Error statusFailure) {
                if (statusFailure != failure) failure.addSuppressed(statusFailure);
            }
        }
    }
}
