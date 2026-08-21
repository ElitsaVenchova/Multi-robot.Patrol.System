package bg.uni.sofia.fmi.simulator;

import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.experiments.ExperimentRunner;

public class Main {
    private static String configPath = "configs/scenarios/example.yaml";//example.yaml
    private static String execMode = "R"; // R - run simulation, E - run experiments
    private static boolean visualization = false; // Дали да има визуализация
    public static void main(String[] args) {
        // Четене на аргументи от командния ред за конфигурационния файл и режима на изпълнение
        if(args.length >= 1) {
            execMode = args[0];
        }
        if(args.length >= 2 && args[2].equals("v")) {
            visualization = true;
        }
        if(args.length >= 3) {
            configPath = args[2];
        }
        // В зависимост от режима на изпълнение, стартиране на симулация или експерименти
        if(execMode.equals("R")) {
            SimulationRunner simRunner = new SimulationRunner();
            simRunner.run(configPath, visualization);
        } else if(execMode.equals("E")) {
            ExperimentRunner expRunner = new ExperimentRunner();
            expRunner.runExperiments(visualization);
        } else {
            System.out.println("Invalid run mode. Please specify 'R' for simulation or 'E' for experiments.");
        }
    }
}
