package bg.uni.sofia.fmi.simulator.experiments;

import bg.uni.sofia.fmi.simulator.config.*;
import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;
import java.util.*;
import java.util.function.Function;

/** Sequential execution is required by static world and random-provider access. */
public class ExperimentRunner {
    public record PreparedRun(String experimentId, String designId, ExperimentConfig.Type type,
            ExperimentConfig.SystemType system, String pointId, String resourceScenario,
            int repetition, long masterSeed, long seed, String configurationYaml) {
        public SimulationConfig configuration() {
            return new org.yaml.snakeyaml.Yaml().loadAs(configurationYaml, SimulationConfig.class);
        }
    }
    public record CompletedRun(PreparedRun context, SimulationMetrics metrics) { }
    private final Function<SimulationConfig, SimulationMetrics> simulate;
    public ExperimentRunner() { this(config -> new SimulationRunner().runWithResult(config, false)); }
    public ExperimentRunner(Function<SimulationConfig, SimulationMetrics> simulate) { this.simulate = simulate; }

    public List<PreparedRun> prepare(ExperimentConfig plan) {
        require(plan != null && identifier(plan.getExperimentId()), "Invalid experimentId");
        require(plan.getRepetitions() > 0, "repetitions must be positive");
        SimulationConfig base = new ConfigLoader().load(plan.getBaseConfig());
        require("AsyncPatrol".equals(base.getPatrolModel().getModel()),
                "Unsupported experiment patrol " + base.getPatrolModel().getModel()
                        + ". Experiments currently support AsyncPatrol only; EncounterPatrol negotiation/rearm"
                        + " and the other patrol models are incomplete.");
        require(base.getSimulation().getSeed() != null, "simulation.seed is required");
        require(plan.getDesigns() != null && !plan.getDesigns().isEmpty(), "No experiment designs");
        List<PreparedRun> result = new ArrayList<>();
        Set<String> ids = new HashSet<>();
        Set<String> designKinds = new HashSet<>();
        for (var design : plan.getDesigns()) {
            require(identifier(design.getId()) && ids.add(design.getId()), "Invalid/duplicate design id");
            require(design.getType() != null && design.getSystem() != null, "Missing design type/system");
            require(designKinds.add(design.getType() + ":" + design.getSystem()), "Duplicate experiment type/system design");
            require(design.getPoints() != null && !design.getPoints().isEmpty(), "Design has no points");
            Set<String> points = new HashSet<>();
            Map<String, String> controls = new HashMap<>();
            Double fixedProbability = null;
            for (var point : design.getPoints()) {
                require(identifier(point.getId()) && points.add(point.getId()), "Invalid/duplicate point id");
                require(point.getResourceScenario() != null && !point.getResourceScenario().isBlank(), "Missing resourceScenario");
                SimulationConfig config = ConfigLoader.copy(base);
                if (plan.getEnergyPriceEurPerKwh() != null) config.setEnergyPriceEurPerKwh(plan.getEnergyPriceEurPerKwh());
                if (point.getRobots() != null) config.setRobots(point.getRobots());
                if (point.getChargingStations() != null) config.setChargingStations(point.getChargingStations());
                if (point.getAttackProbability() != null) config.getAttackModel().setLambda(point.getAttackProbability());
                require("PoissonAttack".equals(config.getAttackModel().getModel()), "Experiments currently support PoissonAttack");
                Double p = config.getAttackModel().getLambda();
                require(p != null && Double.isFinite(p) && p >= 0 && p <= 1, "attack probability must be in [0,1]");
                resolveModels(config, design.getSystem());
                SchemaValidator.validate(config);
                if (design.getType() != ExperimentConfig.Type.ATTACK_DENSITY) {
                    if (fixedProbability == null) fixedProbability = p;
                    require(fixedProbability.equals(p), "Patrol density and TCO designs require fixed attack probability");
                }
                if (design.getType() != ExperimentConfig.Type.TCO) {
                    var controlled = ConfigLoader.copy(config); controlled.getSimulation().setSeed(0L);
                    String controlGroup = "attack";
                    if (design.getType() == ExperimentConfig.Type.ATTACK_DENSITY) controlled.getAttackModel().setLambda(0.0);
                    else {
                        controlGroup = point.getResourceScenario();
                        int divisor = controlled.getRobots().stream().mapToInt(RobotConfig::getCount).reduce(0, ExperimentRunner::gcd);
                        for (var robot : controlled.getRobots()) robot.setCount(robot.getCount() / divisor);
                    }
                    String signature = ConfigLoader.toYaml(controlled);
                    String previous = controls.putIfAbsent(controlGroup, signature);
                    require(previous == null || previous.equals(signature), "Uncontrolled configuration changes in " + design.getId() + "/" + controlGroup);
                }
                if (design.getType() == ExperimentConfig.Type.TCO) {
                    require(config.getEnergyPriceEurPerKwh() != null, "TCO requires energyPriceEurPerKwh");
                    for (var robot : config.getRobots()) {
                        var battery = config.getRobotModels().get(robot.getModel()).getBattery();
                        require(battery.getCycleLife() != null && battery.getReplacementCostEur() != null,
                                "TCO requires battery cycleLife and replacementCostEur for " + robot.getModel());
                    }
                }
                long master = base.getSimulation().getSeed();
                for (int i = 0; i < plan.getRepetitions(); i++) {
                    long seed = Math.addExact(master, i);
                    config.getSimulation().setSeed(seed);
                    result.add(new PreparedRun(plan.getExperimentId(), design.getId(), design.getType(), design.getSystem(),
                            point.getId(), point.getResourceScenario(), i, master, seed, ConfigLoader.toYaml(config)));
                }
            }
        }
        return List.copyOf(result);
    }
    private static void resolveModels(SimulationConfig config, ExperimentConfig.SystemType system) {
        boolean ground = false, drone = false;
        for (var robot : config.getRobots()) {
            require(robot.getCount() > 0, "Robot counts must be positive");
            RobotModelConfig model = config.getRobotModels().get(robot.getModel());
            if (model == null) model = new RobotModelLoader().load(robot.getModel());
            SchemaValidator.validateRobotModel(model);
            config.getRobotModels().put(robot.getModel(), model);
            ground |= "GROUND".equalsIgnoreCase(model.getType());
            drone |= "DRONE".equalsIgnoreCase(model.getType());
            require("GROUND".equalsIgnoreCase(model.getType()) || "DRONE".equalsIgnoreCase(model.getType()), "Unsupported robot type");
        }
        require(switch (system) { case TURTLEBOT -> ground && !drone; case DRONE -> drone && !ground; case HYBRID -> ground && drone; }, "Robot composition does not match " + system);
        if (config.getChargingStations() == null) config.setChargingStations(new ArrayList<>());
        for (var station : config.getChargingStations()) {
            if (!config.getStationModels().containsKey(station.getModel()))
                config.getStationModels().put(station.getModel(), new ChargingStationModelLoader().load(station.getModel()));
            SchemaValidator.validateStationModel(config.getStationModels().get(station.getModel()));
        }
    }
    public List<CompletedRun> execute(ExperimentConfig plan) {
        List<CompletedRun> result = new ArrayList<>();
        for (var run : prepare(plan)) result.add(new CompletedRun(run, simulate.apply(run.configuration())));
        return List.copyOf(result);
    }
    public void runExperiments(String path) { runBatch(ConfigLoader.read(path, ExperimentConfig.class), java.nio.file.Path.of("outputs/results")); }
    public java.nio.file.Path runBatch(ExperimentConfig plan, java.nio.file.Path root) {
        var prepared = prepare(plan); // Validate all points before creating output or running simulations.
        var directory = root.resolve(plan.getExperimentId());
        try (var output = new bg.uni.sofia.fmi.simulator.results.ResultExporter.ExperimentOutput(directory, plan)) {
            java.util.List<CompletedRun> completed = new java.util.ArrayList<>();
            for (var run : prepared) {
                var result = new CompletedRun(run, simulate.apply(run.configuration()));
                output.write(result); completed.add(result);
            }
            output.finish(new bg.uni.sofia.fmi.simulator.results.MetricsAggregator().aggregateExperiments(completed));
        } catch (java.io.IOException e) { throw new IllegalArgumentException("Cannot write experiment batch " + directory, e); }
        System.out.println("Completed experiment batch: " + directory);
        return directory;
    }
    private static boolean identifier(String value) { return value != null && value.matches("[A-Za-z0-9_-]+"); }
    private static int gcd(int a, int b) { return b == 0 ? a : gcd(b, a % b); }
    private static void require(boolean valid, String message) { if (!valid) throw new IllegalArgumentException(message); }
}
