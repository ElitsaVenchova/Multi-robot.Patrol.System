package bg.uni.sofia.fmi.simulator.config;

// This class performs basic validation on the loaded config to ensure required fields are present and have valid values.
public class SchemaValidator {
    public static void validate(SimulationConfig config) {

        if (config == null) {
            throw new IllegalArgumentException("Config is null");
        }

        if (config.getSimulation() == null) {
            throw new IllegalArgumentException("Missing simulation section");
        }

        if (config.getSimulation().getDuration() <= 0) {
            throw new IllegalArgumentException("Simulation duration must be > 0");
        }

        if (config.getRobots() == null || config.getRobots().isEmpty()) {
            throw new IllegalArgumentException("At least one robot must be defined");
        }

        if (config.getPatrolModel() == null) {
            throw new IllegalArgumentException("Patrol model must be specified");
        }

        if (config.getAttackModel() == null) {
            throw new IllegalArgumentException("Attack model must be specified");
        }
    }
}
