package bg.uni.sofia.fmi.simulator.factory;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.config.ChargingStationConfig;
import bg.uni.sofia.fmi.simulator.config.ChargingStationModelConfig;
import bg.uni.sofia.fmi.simulator.config.ChargingStationModelLoader;
import bg.uni.sofia.fmi.simulator.config.RobotConfig;
import bg.uni.sofia.fmi.simulator.config.RobotModelConfig;
import bg.uni.sofia.fmi.simulator.config.RobotModelLoader;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.Battery;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Lidar;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.planning.BehaviorModule;
import bg.uni.sofia.fmi.simulator.planning.EnergyManager;
import bg.uni.sofia.fmi.simulator.planning.SimpleBehaviorController;
import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

//Фабрика за създаване на домейн обекти (Bot, ChargingStation, World) от конфигурацията.
public class DomainFactory {
    public static World createWorld(SimulationConfig config) {
        // Създаване на света от конфигурацията
        double perimeter = config.getSimulation().getPerimeterSize();
        World world = new World(perimeter);
        // Създаване на стратегиите за патрулиране
        PatrolModel patrolModel = StrategyFactory.createPatrol(config.getPatrolModel());
        patrolModel.initialize(world.getBots(), world);
        // Ботове
        List<Bot> bots = createBots(config.getRobots(), world, config.getSimulation().getChargeThreshold(), patrolModel);
        world.addBots(bots);
        // Зарядни станции
        List<ChargingStation> stations = createStations(config.getChargingStations(), world);
        world.addChargingStations(stations);
        //генериране на атаки
        LoadModel attackModel = StrategyFactory.createAttack(config.getAttackModel());
        world.setAttackModel(attackModel);
        return world;
    }
    // Създаване на ботове от конфигурацията
    public static List<Bot> createBots(List<RobotConfig> robotConfigs, World world, double energyThreshold, PatrolModel patrolModel) {
        List<Bot> bots = new ArrayList<>();
        RobotModelLoader loader = new RobotModelLoader();
        
        for (RobotConfig config : robotConfigs) {
            RobotModelConfig model = loader.load(config.getModel());
            for (int i = 0; i < config.getCount(); i++) {
                bots.add(createBot(model, world, energyThreshold, patrolModel));
            }
        }
        return bots;
    }
    //Създаване на бот от конфигурацията
    private static Bot createBot(RobotModelConfig model, World world, double energyThreshold, PatrolModel patrolModel) {
        // Начална позиция на робота
        // [TODO] Позицията може да се задава в конфигурацията или да се генерира
        // на базата на броя ботове и размера на периметъра, за да се избегне струпване
        Position position = new Position(
                RandomProvider.nextDouble() * world.getPerimeter().getSize());
        // Батерията на бота
        Battery battery = new Battery(model.getBatteryCapacity());
        // Лидарът на бота
        Lidar lidar = new Lidar(model.getLidarRange(), model.getBatteryConsumptionRate());
        RobotType type;
        // Модул за управление на енергията, който ще се използва от поведението на бота
        EnergyManager energyManager = new EnergyManager(energyThreshold);
        // Опит за конвертиране на типа на бота от конфигурацията към enum. Ако е невалиден, хвърляме грешка.
        try {
            type = RobotType.valueOf(model.getType().toUpperCase());
        } catch (Exception e) {
            throw new RuntimeException("Invalid robot type: " + model.getType());
        }
        BehaviorModule behavior = new SimpleBehaviorController(energyManager, patrolModel);
        return new Bot(position, battery, lidar, model.getMaxSpeed(), type, model.getName(), model.getFailureProbability(),
                model.getPrice(), model.getBatteryConsumptionRate(), behavior);
    }
    // Създаване на зарядни станции от конфигурацията
    public static List<ChargingStation> createStations(
            List<ChargingStationConfig> configs,
            World world) {
        List<ChargingStation> stations = new ArrayList<>();
        ChargingStationModelLoader loader = new ChargingStationModelLoader();
        for (ChargingStationConfig config : configs) {
            ChargingStationModelConfig model = loader.load(config.getModel());
            // Проверка дали позицията на станцията е в рамките на периметъра
            if (config.getX() < 0 || config.getX() > world.getPerimeter().getSize()) {
                throw new RuntimeException(
                        "Station X out of bounds. x=" + config.getX() + ", perimeter=" + world.getPerimeter());
            }
            // Създаване на позиция за станцията
            Position position = new Position(
                    config.getX(),
                    config.getY(),
                    0.0 // stations are on ground
            );

            stations.add(new ChargingStation(
                    model.getName(),
                    model.getPrice(),
                    model.getSlots(),
                    model.getPower(),
                    model.getFailureProbability(),
                    position));
        }
        return stations;
    }
}