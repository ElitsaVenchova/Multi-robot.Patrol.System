package bg.uni.sofia.fmi.simulator.config;

import java.util.List;

//This represents the main YAML structure. 
// It will be used by SnakeYAML to map the YAML file to Java objects.
public class SimulationConfig {

    private SimulationParameters simulation;
    private List<RobotConfig> robots;
    private List<ChargingStationConfig> chargingStations;
    private PatrolConfig patrolModel;
    private AttackConfig attackModel;

    public void setSimulation(SimulationParameters simulation) {
        this.simulation = simulation;
    }

    public SimulationParameters getSimulation() {
        return simulation;
    }

    public List<RobotConfig> getRobots() {
        return robots;
    }

    public void setRobots(List<RobotConfig> robots) {
        this.robots = robots;
    }

    public PatrolConfig getPatrolModel() {
        return patrolModel;
    }

    public void setPatrol(PatrolConfig patrolModel) {
        this.patrolModel = patrolModel;
    }

    public AttackConfig getAttackModel() {
        return attackModel;
    }

    public void setAttackModel(AttackConfig attackModel) {
        this.attackModel = attackModel;
    }

    public List<ChargingStationConfig> getChargingStations() {
        return chargingStations;
    }

    public void setChargingStations(List<ChargingStationConfig> chargingStations) {
        this.chargingStations = chargingStations;
    }

    // Nested class for simulation parameters
    public static class SimulationParameters {
        private int duration;
        private int perimeterSize;
        private Long seed;

        public int getDuration() {
            return duration;
        }

        public void setDuration(int duration) {
            this.duration = duration;
        }

        public int getPerimeterSize() {
            return perimeterSize;
        }

        public void setPerimeterSize(int perimeterSize) {
            this.perimeterSize = perimeterSize;
        }

        public Long getSeed() {
            return seed;
        }

        public void setSeed(Long seed) {
            this.seed = seed;
        }
    }
}