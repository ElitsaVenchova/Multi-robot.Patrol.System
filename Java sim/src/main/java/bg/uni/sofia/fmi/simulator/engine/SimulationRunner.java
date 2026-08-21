package bg.uni.sofia.fmi.simulator.engine;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.factory.DomainFactory;
import bg.uni.sofia.fmi.simulator.results.MetricsCalculator;
import bg.uni.sofia.fmi.simulator.results.ResultExporter;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;
import bg.uni.sofia.fmi.simulator.visualization.SimulationVisualizer;

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
    public void run(String configPath, boolean visualization) {
        // Зареждане на конфигурацията
        ConfigLoader loader = new ConfigLoader();
        SimulationConfig config = loader.load(configPath);
        World world = run(config, visualization);

        // След края на симулацията, събиране и отпечатване на резултатите
        printResults(world);
    }

    // Метод за стартиране на симулацията и връщане на резултатите
    // (за експерименти и анализ)
    public SimulationMetrics runWithResult(SimulationConfig config, boolean visualization) {
        World world = run(config, visualization);
        // Събиране на резултатите
        MetricsCalculator calculator = new MetricsCalculator();
        return calculator.calculate(world);
    }

    // Метод за стартиране на симулацията и връщане на света (за по-нататъшен анализ)
    private World run(SimulationConfig config, boolean visualization) {
        RandomProvider.setSeed(config.getSimulation().getSeed()); 
        // Създаване на света от конфигурацията
        World world = DomainFactory.createWorld(config);
        
        // Стартиране на визуализацията, ако е зададено
        if (visualization) {
            SimulationVisualizer.launch(world);
        }
        
        // Основен цикъл на симулацията
        int duration = config.getSimulation().getDuration();
        for (int t = 0; t < duration; t++) {
            // Обновяване на състоянието на света
            // (движение на атаки, проверка за интерцептирани и пропуснати атаки и т.н.)
            world.tick(t);
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
