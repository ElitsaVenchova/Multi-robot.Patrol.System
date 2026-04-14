package bg.uni.sofia.fmi.simulator.engine;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.factory.DomainFactory;
import bg.uni.sofia.fmi.simulator.factory.StrategyFactory;
import bg.uni.sofia.fmi.simulator.results.MetricsCalculator;
import bg.uni.sofia.fmi.simulator.results.ResultExporter;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;
import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

/**
 * Основен клас за стартиране на симулацията.
 * - зарежда конфигурацията
 * - създава света и стратегиите
 * - изпълнява цикъла на симулацията
 * - събира и отпечатва резултатите
 */
public class SimulationRunner {
    // Метод за стартиране на симулацията с даден конфигурационен файл
    // (за бързо стартиране от main метода)
    public void run(String configPath) {
        // Зареждане на конфигурацията
        ConfigLoader loader = new ConfigLoader();
        SimulationConfig config = loader.load(configPath);
        World world = run(config);

        // След края на симулацията, събиране и отпечатване на резултатите
        printResults(world);
    }

    // Метод за стартиране на симулацията и връщане на резултатите
    // (за експерименти и анализ)
    public SimulationMetrics runWithResult(SimulationConfig config) {
        World world = run(config);
        // Събиране на резултатите
        MetricsCalculator calculator = new MetricsCalculator();
        return calculator.calculate(world);
    }

    // Метод за стартиране на симулацията и връщане на света (за по-нататъшен анализ)
    private World run(SimulationConfig config) {
        RandomProvider.setSeed(config.getSimulation().getSeed());
        // Създаване на света от конфигурацията
        World world = DomainFactory.createWorld(config);
        // Създаване на стратегиите за патрулиране и генериране на атаки
        PatrolModel patrolModel = StrategyFactory.createPatrol(config.getPatrolModel());
        patrolModel.initialize(world.getBots(), world);
        LoadModel attackModel = StrategyFactory.createAttack(config.getAttackModel());
        world.setAttackModel(attackModel);
        world.setPatrolModel(patrolModel);
        // Основен цикъл на симулацията
        int duration = config.getSimulation().getDuration();
        for (int t = 0; t < duration; t++) {
            // Обновяване на състоянието на света
            // (движение на атаки, проверка за интерцептирани и пропуснати атаки и т.н.)
            world.tick(t);

            // [TODO] Това може да се оптимизира, като се прави паралелно за всеки бот и за
            // засичането на атаки
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

        return world;
    }

    // Метод за събиране и отпечатване на резултатите от симулацията
    private void printResults(World world) {
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
}
