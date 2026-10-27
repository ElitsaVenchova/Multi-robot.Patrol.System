package bg.uni.sofia.fmi.simulator.config;

// Клас за валидация на конфигурацията спрямо очакваната структура и изисквания
// [TODO] да се обновят накрая
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
        if (config.getPatrolModel().getEncounterRearmPolicy() == null) {
            throw new IllegalArgumentException("Encounter rearm policy cannot be null");
        }
        if (config.getPerimeter() == null || config.getPerimeter().getSize() <= 0)
            throw new IllegalArgumentException("Positive perimeter length in meters is required");
        bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType.valueOf(config.getPerimeter().getType().toUpperCase(java.util.Locale.ROOT));
        if (config.getSimulation().getSeed() == null) throw new IllegalArgumentException("simulation.seed is required");
        nonNegative(config.getSimulation().getChargeThreshold(), "chargeThreshold Wh");
        if (config.getEnergyPriceEurPerKwh() != null) nonNegative(config.getEnergyPriceEurPerKwh(), "energy tariff EUR/kWh");
        for (var robot : config.getRobots()) {
            if (robot.getModel() == null || robot.getCount() <= 0) throw new IllegalArgumentException("Positive robot counts and model names are required");
        }
        if (config.getChargingStations() == null) config.setChargingStations(new java.util.ArrayList<>());
        for (var station : config.getChargingStations()) {
            if (station.getModel() == null || !Double.isFinite(station.getX()) || station.getX() < 0
                    || station.getX() > config.getPerimeter().getSize() || !Double.isFinite(station.getY()))
                throw new IllegalArgumentException("Invalid station model/position");
        }
        Double probability = config.getAttackModel().getLambda();
        if ("PoissonAttack".equals(config.getAttackModel().getModel()) &&
                (probability == null || !Double.isFinite(probability) || probability < 0 || probability > 1))
            throw new IllegalArgumentException("PoissonAttack lambda must be a probability in [0,1] per tick");
        Integer duration = config.getAttackModel().getDuration();
        if (duration != null && duration < 0) throw new IllegalArgumentException("Attack duration cannot be negative");
        nonNegative(config.getPatrolModel().getDeviationProbability(), "deviation probability");
        if (config.getPatrolModel().getDeviationProbability() > 1 || (config.getPatrolModel().getDeviationProbability() > 0
                && config.getPatrolModel().getMaxDeviationDuration() <= 0))
            throw new IllegalArgumentException("Invalid patrol deviation parameters");
    }

    public static void validateRobotModel(RobotModelConfig model) {
        positive(model.getMaxSpeed(), "speed m/min");
        nonNegative(model.getPrice(), "robot price EUR");
        nonNegative(model.getBatteryConsumptionRate(), "movement Wh/m");
        nonNegative(model.getBaselineConsumptionWhPerMinute(), "baseline Wh/min");
        if (model.getBattery() == null || model.getLidar() == null || model.getCommunicationModule() == null)
            throw new IllegalArgumentException("Missing robot components");
        positive(model.getBattery().getBatteryCapacity(), "battery capacity Wh");
        nonNegative(model.getLidar().getLidarRange(), "LiDAR range m");
        nonNegative(model.getLidar().getLidarBatteryConsumptionRate(), "LiDAR Wh/scan");
        new bg.uni.sofia.fmi.simulator.domain.Battery(model.getBattery().getBatteryCapacity(), model.getBattery().getAverageBatteryLife())
                .setWearModel(model.getBattery().getCycleLife(), model.getBattery().getReplacementCostEur());
    }
    public static void validateStationModel(ChargingStationModelConfig model) {
        if (model.getSlots() <= 0) throw new IllegalArgumentException("Station slots must be positive");
        positive(model.getPower(), "charging power W per slot");
        nonNegative(model.getPrice(), "station price EUR");
    }
    private static void nonNegative(double v, String name) {
        if (!Double.isFinite(v) || v < 0) throw new IllegalArgumentException("Invalid " + name);
    }
    private static void positive(double v, String name) {
        if (!Double.isFinite(v) || v <= 0) throw new IllegalArgumentException("Positive " + name + " required");
    }
}
