package bg.uni.sofia.fmi.simulator.config;

import java.util.List;

// Главен клас за конфигурацията на симулацията, 
// който обединява всички други конфигурационни класове.
public class SimulationConfig {
    // основни параметри на симулацията като продължителност, размер на периметъра, 
    // сийд за генератора на случайни числа и прагове за зареждане
    private SimulationParameters simulation;
    // конфигурация за различните видове роботи, които ще участват в симулацията
    private List<RobotConfig> robots;
    // конфигурация за зарядните станции в симулацията
    private List<ChargingStationConfig> chargingStations;
    private PatrolConfig patrolModel; // конфигурация за патрулирането
    private AttackConfig attackModel; // конфигурация за атаките

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
        private double chargeThreshold;

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

        public double getChargeThreshold() {
            return chargeThreshold;
        }

        public void setChargeThreshold(double chargeThreshold) {
            this.chargeThreshold = chargeThreshold;
        }
    }
}