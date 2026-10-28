package bg.uni.sofia.fmi.simulator.experiments;

import bg.uni.sofia.fmi.simulator.config.*;
import java.util.*;

/** Explicit points avoid hidden scientific defaults. */
public class ExperimentConfig {
    public enum Type { PATROL_DENSITY, ATTACK_DENSITY, TCO }
    public enum SystemType { TURTLEBOT, DRONE, HYBRID }
    private String experimentId;
    private String baseConfig;
    private int repetitions;
    private Double energyPriceEurPerKwh;
    private List<Design> designs = new ArrayList<>();
    public String getExperimentId() { return experimentId; }
    public void setExperimentId(String v) { experimentId = v; }
    public String getBaseConfig() { return baseConfig; }
    public void setBaseConfig(String v) { baseConfig = v; }
    public int getRepetitions() { return repetitions; }
    public void setRepetitions(int v) { repetitions = v; }
    public Double getEnergyPriceEurPerKwh() { return energyPriceEurPerKwh; }
    public void setEnergyPriceEurPerKwh(Double v) { energyPriceEurPerKwh = v; }
    public List<Design> getDesigns() { return designs; }
    public void setDesigns(List<Design> v) { designs = v; }

    public static class Design {
        private String id;
        private Type type;
        private SystemType system;
        private List<Point> points = new ArrayList<>();
        public String getId() { return id; }
        public void setId(String v) { id = v; }
        public Type getType() { return type; }
        public void setType(Type v) { type = v; }
        public SystemType getSystem() { return system; }
        public void setSystem(SystemType v) { system = v; }
        public List<Point> getPoints() { return points; }
        public void setPoints(List<Point> v) { points = v; }
    }
    public static class Point {
        private String id;
        private String resourceScenario;
        private List<RobotConfig> robots;
        private List<ChargingStationConfig> chargingStations;
        private Double attackProbability;
        public String getId() { return id; }
        public void setId(String v) { id = v; }
        public String getResourceScenario() { return resourceScenario; }
        public void setResourceScenario(String v) { resourceScenario = v; }
        public List<RobotConfig> getRobots() { return robots; }
        public void setRobots(List<RobotConfig> v) { robots = v; }
        public List<ChargingStationConfig> getChargingStations() { return chargingStations; }
        public void setChargingStations(List<ChargingStationConfig> v) { chargingStations = v; }
        public Double getAttackProbability() { return attackProbability; }
        public void setAttackProbability(Double v) { attackProbability = v; }
    }
}
