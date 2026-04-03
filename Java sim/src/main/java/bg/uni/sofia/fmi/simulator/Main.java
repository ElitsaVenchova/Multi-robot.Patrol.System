package bg.uni.sofia.fmi.simulator;

import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.experiments.ExperimentRunner;

public class Main {
    private static String configPath = "configs/scenarios/example.yaml";
    private static String execMode = "R"; // R - run simulation, E - run experiments
    public static void main(String[] args) {
        if(args.length >= 1) {
            configPath = args[0];
        }
        if(args.length >= 2) {
            execMode = args[1];
        }

        if(execMode.equals("R")) {
            SimulationRunner simRunner = new SimulationRunner();
            simRunner.run(configPath);
        } else if(execMode.equals("E")) {
            ExperimentRunner expRunner = new ExperimentRunner();
            expRunner.runExperiments();
        } else {
            System.out.println("Invalid run mode. Please specify 'R' for simulation or 'E' for experiments.");
        }
    }
}
