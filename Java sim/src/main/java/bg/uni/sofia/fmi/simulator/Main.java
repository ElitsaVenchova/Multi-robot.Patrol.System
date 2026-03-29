package bg.uni.sofia.fmi.simulator;

import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.experiments.ExperimentRunner;

public class Main {
    public static void main(String[] args) {
        System.out.println("Hello, World!");
        SimulationRunner simRunner = new SimulationRunner();
        simRunner.run("configs/scenarios/example.yaml");

        // ExperimentRunner expRunner = new ExperimentRunner();
        // expRunner.runExperiments();
    }
}
