package bg.uni.sofia.fmi.simulator.experiments;

import bg.uni.sofia.fmi.simulator.config.*;
import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.results.MetricsAggregator;
import bg.uni.sofia.fmi.simulator.results.ResultExporter;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;
import bg.uni.sofia.fmi.simulator.util.MathUtils;
import org.yaml.snakeyaml.Yaml;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import java.util.logging.Logger;

/** Sequential execution is required by static world and random-provider access. */
public class ExperimentRunner {
    Logger logger = Logger.getLogger(getClass().getName());

    //Това се вика от main за изпълнение
    public void runExperiments(String path) {
        runBatch(ConfigLoader.read(path, ExperimentConfig.class), Path.of("outputs/results"));
    }
    //Изпълнява експериментите в пакет
    public Path runBatch(ExperimentConfig plan, Path root) {
        List<ExperimentRunner.PreparedRun> prepared = prepare(plan); // Генерира различните конфигурации за експеримента
        Path directory = root.resolve(plan.getExperimentId());
        try (ResultExporter.ExperimentOutput output = new ResultExporter.ExperimentOutput(directory, plan)) {
            List<CompletedRun> completed = new ArrayList<>();
            for (ExperimentRunner.PreparedRun run : prepared) {
                //Изпълняване на симулацията с подготвеното изпълнение
                ExperimentRunner.CompletedRun result = new CompletedRun(run, new SimulationRunner().runWithResult(run.configuration(), false));
                output.write(result);
                completed.add(result);
            }
            output.finish(new MetricsAggregator().aggregateExperiments(completed));
        } catch (IOException e) { throw new IllegalArgumentException("Cannot write experiment batch " + directory, e); }
        logger.fine("Completed experiment batch: " + directory);
        return directory;
    }
    //Генерира различните конфигурации за експеримента
    public List<PreparedRun> prepare(ExperimentConfig plan) {
        SimulationConfig base = new ConfigLoader().load(plan.getBaseConfig());
        List<PreparedRun> result = new ArrayList<>();
        for (ExperimentConfig.Design design : plan.getDesigns()) {
            Double fixedProbability = null;
            for (ExperimentConfig.Point point : design.getPoints()) {
                SimulationConfig config = ConfigLoader.copy(base);
                if (plan.getEnergyPriceEurPerKwh() != null) config.setEnergyPriceEurPerKwh(plan.getEnergyPriceEurPerKwh());
                if (point.getRobots() != null) config.setRobots(point.getRobots());
                if (point.getChargingStations() != null) config.setChargingStations(point.getChargingStations());
                if (point.getAttackProbability() != null) config.getAttackModel().setLambda(point.getAttackProbability());
                Double p = config.getAttackModel().getLambda();
                //Зарежда конфигурациите на роботите
                for (RobotConfig robot : config.getRobots()) {
                    RobotModelConfig model = config.getRobotModels().get(robot.getModel());
                    if (model == null) model = new RobotModelLoader().load(robot.getModel());
                    SchemaValidator.validateRobotModel(model);
                    config.getRobotModels().put(robot.getModel(), model);
                }
                //Зарежда конфигурациите на станциите
                if (config.getChargingStations() == null) config.setChargingStations(new ArrayList<>());
                for (ChargingStationConfig station : config.getChargingStations()) {
                    if (!config.getStationModels().containsKey(station.getModel()))
                        config.getStationModels().put(station.getModel(), new ChargingStationModelLoader().load(station.getModel()));
                    SchemaValidator.validateStationModel(config.getStationModels().get(station.getModel()));
                }
                SchemaValidator.validate(config);
                if (design.getType() != ExperimentConfig.Type.ATTACK_DENSITY) {
                    if (fixedProbability == null) fixedProbability = p;
                }
                if (design.getType() != ExperimentConfig.Type.TCO) {
                    SimulationConfig controlled = ConfigLoader.copy(config); controlled.getSimulation().setSeed(0L);
                    if (design.getType() == ExperimentConfig.Type.ATTACK_DENSITY) controlled.getAttackModel().setLambda(0.0);
                    else {
                        int divisor = controlled.getRobots().stream().mapToInt(RobotConfig::getCount).reduce(0, MathUtils::gcd);
                        for (RobotConfig robot : controlled.getRobots()) robot.setCount(robot.getCount() / divisor);
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

    public record PreparedRun(String experimentId, String designId, ExperimentConfig.Type type,
                              ExperimentConfig.SystemType system, String pointId, String resourceScenario,
                              int repetition, long masterSeed, long seed, String configurationYaml) {
        public SimulationConfig configuration() {
            return new Yaml().loadAs(configurationYaml, SimulationConfig.class);
        }
    }
    public record CompletedRun(PreparedRun context, SimulationMetrics metrics) { }
}
