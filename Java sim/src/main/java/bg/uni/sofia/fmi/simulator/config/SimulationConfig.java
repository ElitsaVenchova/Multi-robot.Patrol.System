package bg.uni.sofia.fmi.simulator.config;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

// Главен клас за конфигурацията на симулацията, 
// който обединява всички други конфигурационни класове.
/** Simulation duration and attack lifetime are ticks of one minute; perimeter length is meters. */
public class SimulationConfig {
    // основни параметри на симулацията като продължителност, размер на периметъра, 
    // сийд за генератора на случайни числа и прагове за зареждане
    private SimulationParameters simulation;
    // конфигурация на периметъра
    private PerimeterConfig perimeter;
    // конфигурация за различните видове роботи, които ще участват в симулацията
    private List<RobotConfig> robots;
    // конфигурация за зарядните станции в симулацията
    private List<ChargingStationConfig> chargingStations;
    private PatrolConfig patrolModel; // конфигурация за патрулирането
    private AttackConfig attackModel; // конфигурация за атаките
    private Double energyPriceEurPerKwh;

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

    public List<ChargingStationConfig> getChargingStations() { return chargingStations; }
    public void setChargingStations(List<ChargingStationConfig> chargingStations) { this.chargingStations = chargingStations; }

    public PerimeterConfig getPerimeter() { return perimeter; }
    public void setPerimeter(PerimeterConfig perimeter) {
        this.perimeter = perimeter;
    }

    public Double getEnergyPriceEurPerKwh() { return energyPriceEurPerKwh; }
    public void setEnergyPriceEurPerKwh(Double v) { energyPriceEurPerKwh = v; }
    private Map<String, RobotModelConfig> robotModels = new LinkedHashMap<>();
    private Map<String, ChargingStationModelConfig> stationModels = new LinkedHashMap<>();
    public Map<String, RobotModelConfig> getRobotModels() { return robotModels; }
    public void setRobotModels(Map<String, RobotModelConfig> v) { robotModels = v; }
    public Map<String, ChargingStationModelConfig> getStationModels() { return stationModels; }
    public void setStationModels(Map<String, ChargingStationModelConfig> v) { stationModels = v; }
    public void setPatrolModel(PatrolConfig v) { patrolModel = v; }

    // Nested class for simulation parameters
    public static class SimulationParameters {
        private int duration;
        private Long seed;
        private double chargeThreshold;

        public int getDuration() {
            return duration;
        }

        public void setDuration(int duration) {
            this.duration = duration;
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
