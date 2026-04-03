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

public class ExperimentRunner {

    private static final String BASE_CONFIG = "configs/scenarios/base.yaml";

    public void runExperiments() {

        List<String> patrolStrategies = Arrays.asList(
                "RingPatrol",
                "CounterPhasePatrol",
                "AsyncPatrol");

        List<Double> lambdas = Arrays.asList(0.1, 0.3, 0.5);

        int runsPerConfig = 5;

        ConfigLoader loader = new ConfigLoader();
        ResultExporter exporter = new ResultExporter();

        MetricsAggregator aggregator = new MetricsAggregator();

        for (String strategy : patrolStrategies) {
            for (Double lambda : lambdas) {

                List<SimulationMetrics> results = new ArrayList<>();

                for (int run = 0; run < runsPerConfig; run++) {

                    SimulationConfig config = loader.load(BASE_CONFIG);
   
                    config.getPatrolModel().setType(strategy);
                    config.getPatrolModel().setRobotsPerSection(1);
                    config.getAttackModel().setType("PoissonAttack"); // [TODO] should be in config file
                    config.getAttackModel().setLambda(lambda);
                    config.getSimulation().setSeed(System.currentTimeMillis() + run);

                    SimulationRunner runner = new SimulationRunner();
                    SimulationMetrics metrics = runner.runWithResult(config);

                    results.add(metrics);
                }

                // 👉 Aggregate
                AggregatedMetrics aggregated = aggregator.aggregate(results);

                // 👉 Export aggregated results
                exporter.exportAggregated(aggregated, strategy, lambda);

                System.out.println("Aggregated: " + strategy +
                        " | lambda=" + lambda);
            }
        }
    }
}