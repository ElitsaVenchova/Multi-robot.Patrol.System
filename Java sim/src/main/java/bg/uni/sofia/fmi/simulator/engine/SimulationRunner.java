package bg.uni.sofia.fmi.simulator.engine;

import java.util.List;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;
import bg.uni.sofia.fmi.simulator.factory.DomainFactory;
import bg.uni.sofia.fmi.simulator.factory.StrategyFactory;
import bg.uni.sofia.fmi.simulator.results.MetricsCalculator;
import bg.uni.sofia.fmi.simulator.results.ResultExporter;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;
import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

/**
 * Main class responsible for running the simulation.
 * - loads configuration
 * - creates world and strategies
 * - runs the simulation loop
 * - collects and prints results
 */
public class SimulationRunner {

    public void run(String configPath) {
        // 1️⃣ Load config
        ConfigLoader loader = new ConfigLoader();
        SimulationConfig config = loader.load(configPath);
        RandomProvider.setSeed(config.getSimulation().getSeed());

        // 2️⃣ Create world
        World world = DomainFactory.createWorld(config);

        // 3️⃣ Create strategies
        PatrolModel patrolModel = StrategyFactory.createPatrol(config.getPatrolModel());
        LoadModel attackModel = StrategyFactory.createAttack(config.getAttackModel());

        // 4️⃣ Initialize strategies
        patrolModel.initialize(world.getBots(), world);

        int duration = config.getSimulation().getDuration();

        // 5️⃣ Simulation loop
        for (int t = 0; t < duration; t++) {

            // Generate attacks
            List<Attack> newAttacks = attackModel.generateAttacks(world, t);
            for (Attack attack : newAttacks) {
                world.addAttack(attack);
            }

            // Execute patrol behavior
            for (Bot bot : world.getBots()) {
                bot.update(world, t);
            }

            // Update world (movement + detection)
            world.tick(t); // ✅

            // Parallerization (optional)
            // SimulationMode mode = SimulationMode.PARALLEL;
            // ParallelExecutor executor = new
            // ParallelExecutor(Runtime.getRuntime().availableProcessors());
            // ParallelPatrolExecutor patrolExecutor = new
            // ParallelPatrolExecutor(executor.getExecutor());
            // ParallelDetectionEngine detectionEngine = new
            // ParallelDetectionEngine(executor.getExecutor());
            // // Movement
            // patrolExecutor.execute(world.getBots());
            // // Detection
            // detectionEngine.detect(world.getBots(), world.getAttacks(), t);
            // executor.shutdown();
        }

        // 6️⃣ Results
        printResults(world);
    }

    private void printResults(World world) {
        for (Attack attack : world.getAttacks()) {
            if (attack.getStatus() == AttackStatus.INTERCEPTED) {
                attack.intercept();
            } else if (attack.getStatus() == AttackStatus.MISSED) {
                attack.miss();
            }
        }

        MetricsCalculator calculator = new MetricsCalculator();
        SimulationMetrics metrics = calculator.calculate(world);

        // Print (optional)
        System.out.println("===== Simulation Results =====");
        System.out.println("Total attacks: " + metrics.getTotalAttacks());
        System.out.println("Intercepted: " + metrics.getInterceptedAttacks());
        System.out.println("Missed: " + metrics.getMissedAttacks());
        System.out.println("Success rate: " + metrics.getSuccessRate());

        // Export
        ResultExporter exporter = new ResultExporter();
        exporter.export(metrics);
    }

    public SimulationMetrics runWithResult(SimulationConfig config) {
        RandomProvider.setSeed(config.getSimulation().getSeed());

        World world = DomainFactory.createWorld(config);

        PatrolModel patrolModel = StrategyFactory.createPatrol(config.getPatrolModel());
        LoadModel attackModel = StrategyFactory.createAttack(config.getAttackModel());

        patrolModel.initialize(world.getBots(), world);

        int duration = config.getSimulation().getDuration();

        for (int t = 0; t < duration; t++) {

            List<Attack> newAttacks = attackModel.generateAttacks(world, t);
            for (Attack attack : newAttacks) {
                world.addAttack(attack);
            }

            patrolModel.execute(world.getBots(), world);
            world.tick(t);
        }

        MetricsCalculator calculator = new MetricsCalculator();
        return calculator.calculate(world);
    }
}
