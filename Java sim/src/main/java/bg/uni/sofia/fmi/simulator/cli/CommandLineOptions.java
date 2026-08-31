package bg.uni.sofia.fmi.simulator.cli;

// Параметрите от терминала
public record CommandLineOptions(
        ExecutionMode executionMode,
        boolean visualization,
        String configPath
) {
}
