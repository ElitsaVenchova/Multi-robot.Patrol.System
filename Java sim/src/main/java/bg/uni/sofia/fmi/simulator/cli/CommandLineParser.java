package bg.uni.sofia.fmi.simulator.cli;

// Парсва аргументите от командната линия.
public class CommandLineParser {
    private static final String DEFAULT_CONFIG_PATH = "configs/scenarios/example.yaml";

    private CommandLineParser() {
    }

    // самото парсване на аргументите. долу в help се вижда какво е прието като вход
    public static CommandLineOptions parse(String[] args) {
        // Стойности по подразбиране
        ExecutionMode executionMode = ExecutionMode.RUN;
        boolean visualization = false;
        String configPath = DEFAULT_CONFIG_PATH;


        // Флагове за проверка за повторно подаване на execution mode и path
        boolean executionModeSpecified = false;
        boolean configPathSpecified = false;
        for (String arg : args) {
            switch (arg.toLowerCase()) {
                case "-r", "--run" -> {
                    if (executionModeSpecified) {
                        throw new IllegalArgumentException("Only one execution mode can be specified: -r or -e.");
                    }
                    executionMode = ExecutionMode.RUN;
                    executionModeSpecified = true;
                }
                case "-e", "--experiment" -> {
                    if (executionModeSpecified) {
                        throw new IllegalArgumentException("Only one execution mode can be specified: -r or -e.");
                    }
                    executionMode = ExecutionMode.EXPERIMENT;
                    executionModeSpecified = true;
                }
                case "-v", "--visualize" -> visualization = true;
                case "-h", "--help" -> {
                    printHelp();
                    System.exit(0);
                }
                default -> {
                    if (arg.startsWith("-")) {
                        throw new IllegalArgumentException("Unknown option: " + arg);
                    }
                    if (configPathSpecified) {
                        throw new IllegalArgumentException("Only one configuration path can be specified.");
                    }
                    configPath = arg;
                    configPathSpecified = true;
                }
            }
        }
        return new CommandLineOptions(executionMode,visualization,configPath);
    }

    public static void printHelp() {
        System.out.println("""
            
            Autonomous Perimeter Surveillance Simulator
            
            Usage:
              [options] [config-path]
            
            Options:
              -r, --run           Run simulation
              -e, --experiment    Run experiments
              -v, --visualize     Enable visualization
              -h, --help          Show this help message
            
            Defaults:
              Execution mode:     experiment
              Visualization:      disabled
              Configuration:      configs/scenarios/example.yaml
            
            Examples:
              -r
              -r -v
              -r configs/scenarios/example.yaml
              -e configs/scenarios/experiment.yaml
              -e -v configs/scenarios/experiment.yaml
            
            Notes:
              Only one execution mode (-r or -e) can be specified.
              Only one configuration path can be specified.
            """);
    }
}
