package bg.uni.sofia.fmi.simulator.experiments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.results.AggregatedMetrics;
import bg.uni.sofia.fmi.simulator.results.MetricsAggregator;
import bg.uni.sofia.fmi.simulator.results.ResultExporter;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;

// Клас за стартиране на експерименти с различни конфигурации и събиране на резултатите
// След това с scripts/plot_result.py може да се направят диаграми на резултата от експеримента.
// В aggregated_results.csv винаги се съдържа резултата само от последният пуснат експертимент
public class ExperimentRunner {
    public void runExperiments(String configPath, boolean visualization) {
        ConfigLoader loader = new ConfigLoader();
        ResultExporter exporter = new ResultExporter();
        MetricsAggregator aggregator = new MetricsAggregator();

        // Дефиниране на различните конфигурации за експериментиране
        List<String> patrolStrategies = Arrays.asList(
                "RingPatrol",
                "CounterPhasePatrol",
                "AsyncPatrol");
        List<Double> lambdas = Arrays.asList(0.1, 0.3, 0.5);
        int runsPerConfig = 5;

        // За всяка комбинация от конфигурации, стартиране на няколко симулации и събиране на резултатите
        for (String strategy : patrolStrategies) {
            for (Double lambda : lambdas) {
                List<SimulationMetrics> results = new ArrayList<>();
                for (int run = 0; run < runsPerConfig; run++) {
                    SimulationConfig config = loader.load(configPath);
                    // Модифициране на конфигурацията според текущия експеримент
                    config.getPatrolModel().setModel(strategy);
                    config.getPatrolModel().setDeviationProbability(config.getPatrolModel().getDeviationProbability());
                    config.getPatrolModel().setMaxDeviationDuration(config.getPatrolModel().getMaxDeviationDuration());
                    // Стартиране на симулацията и събиране на резултатите
                    SimulationRunner runner = new SimulationRunner();
                    SimulationMetrics metrics = runner.runWithResult(config, visualization);
                    // Съхраняване на резултатите от текущия експеримент
                    results.add(metrics);
                }

                // Агрегиране на резултатите от всички симулации за текущата конфигурация
                AggregatedMetrics aggregated = aggregator.aggregate(results);
                // Отпечатване и експортиране на резултатите от експеримента
                exporter.exportAggregated(aggregated, strategy, lambda);
                System.out.println("Aggregated: " + strategy +
                        " | lambda=" + lambda);
            }
        }
    }
}