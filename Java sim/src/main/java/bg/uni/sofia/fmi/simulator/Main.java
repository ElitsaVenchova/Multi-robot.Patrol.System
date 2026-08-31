package bg.uni.sofia.fmi.simulator;

import bg.uni.sofia.fmi.simulator.cli.CommandLineOptions;
import bg.uni.sofia.fmi.simulator.cli.CommandLineParser;
import bg.uni.sofia.fmi.simulator.cli.ExecutionMode;
import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.experiments.ExperimentRunner;

public class Main {
    /**
     * Стартиране на Симулатора за патрулиране. Има създаден CLI paser за подаване на входни аргументи
     * за режим на изпълнение, дали да има визуализация и път до конфигурация на самата симулация.
     *   Defaults:
     *     Execution mode:     experiment
     *     Visualization:      disabled
     *     Configuration:      configs/scenarios/example.yaml
     * @param args
     *   Examples:
     *     -r
     *     -r -v
     *     -r configs/scenarios/example.yaml
     *     -e configs/scenarios/experiment.yaml
     *     -e -v configs/scenarios/experiment.yaml
     */
    public static void main(String[] args) {
        try {
            // Четене на аргументи от командния ред за конфигурационния файл и режима на изпълнение
            CommandLineOptions options = CommandLineParser.parse(args);
            // В зависимост от режима на изпълнение, стартиране на симулация или експерименти
            if (options.executionMode() == ExecutionMode.RUN) {
                new SimulationRunner().run(
                        options.configPath(),
                        options.visualization()
                );
            } else {
                new ExperimentRunner().runExperiments(
                        options.configPath(),
                        options.visualization()
                );
            }

        } catch (IllegalArgumentException e) {
            System.err.println("Error: " + e.getMessage());
            System.err.println();
            CommandLineParser.printHelp();
            System.exit(1);
        }
    }
}
