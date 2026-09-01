package bg.uni.sofia.fmi.simulator.engine;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.factory.DomainFactory;
import bg.uni.sofia.fmi.simulator.results.MetricsCalculator;
import bg.uni.sofia.fmi.simulator.results.ResultExporter;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;
import bg.uni.sofia.fmi.simulator.visualization.SimulationVisualizer;

/**
 * Основен клас за стартиране на симулацията.
 * - зарежда конфигурацията
 * - създава света и стратегиите
 * - изпълнява цикъла на симулацията (чрез Simulator)
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
        // Ако има визуализацуя, то има отделно извикване на printResult,
        // когато симулацията приключи.
        if (!visualization) {
            printResults(world);
        }
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
        IdGenerator.reset();
        // Създаване на света от конфигурацията
        World world = DomainFactory.createWorld(config);
        int duration = config.getSimulation().getDuration();
        
        if (visualization) {
            // Визуализаторът управлява симулацията и извиква callback за отпечатване на резултатите при приключването ѝ.
            SimulationVisualizer.launch(world, config.getSimulation().getDuration(), () -> printResults(world));
        } else {
            // Без визуализация. Бързо изпълнение
            for (int t = 0; t < duration; t++) {
                world.tick(t);
            }
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
