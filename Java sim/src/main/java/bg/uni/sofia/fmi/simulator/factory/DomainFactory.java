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
import bg.uni.sofia.fmi.simulator.domain.*;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.behavior.planning.PlanningModule;
import bg.uni.sofia.fmi.simulator.behavior.energyManagment.EnergyManager;
import bg.uni.sofia.fmi.simulator.behavior.navigation.Navigation;
import bg.uni.sofia.fmi.simulator.behavior.collisionAvoidance.ObstacleAvoidance;
import bg.uni.sofia.fmi.simulator.behavior.planning.SimplePlanningController;
import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;

//Фабрика за създаване на домейн обекти (Bot, ChargingStation, World) от конфигурацията.
public class DomainFactory {
    public static World createWorld(SimulationConfig config) {
        // Създаване на света от конфигурацията
        int perimeterSize = config.getPerimeter().getSize();
        PerimeterType perimeterType = PerimeterType.valueOf(config.getPerimeter().getType().toUpperCase());
        World world = new World(new Perimeter(perimeterSize, perimeterType));
        // Ботове
        List<Bot> bots = createBots(config, world, config.getSimulation().getChargeThreshold());
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
    private static List<Bot> createBots(SimulationConfig config, World world, double energyThreshold) {
        List<Bot> bots = new ArrayList<>();
        RobotModelLoader loader = new RobotModelLoader();
        
        for (RobotConfig robotConfig : config.getRobots()) {
            RobotModelConfig model = loader.load(robotConfig.getModel());
            for (int i = 0; i < robotConfig.getCount(); i++) {
                long id = IdGenerator.nextId();
                // Създаване на стратегиите за патрулиране
                PatrolModel patrolModel = StrategyFactory.createPatrol(config.getPatrolModel());
                // Геометричната начална позиция е локална за този свят;
                // глобалният ID служи само за идентификация и логване.
                bots.add(createBot(id, bots.size() + 1, model, world, energyThreshold, patrolModel));
            }
        }
        return bots;
    }
    //Създаване на бот от конфигурацията
    private static Bot createBot(long id, int initialX, RobotModelConfig model, World world, double energyThreshold, PatrolModel patrolModel) {
        // Начална позиция на робота спрямо реда на въвеждане
        Position position = new Position(initialX);
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
        ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance();
        Navigation navigation = new Navigation(obstacleAvoidance);
        PlanningModule planningModule = new SimplePlanningController(energyManager, patrolModel, navigation, world);
        return new Bot(id, position, battery, lidar, model.getMaxSpeed(), type, model.getName(), model.getFailureProbability(),
                model.getPrice(), model.getBatteryConsumptionRate(), planningModule);
    }
    // Създаване на зарядни станции от конфигурацията
    private static List<ChargingStation> createStations(List<ChargingStationConfig> configs, World world) {
        List<ChargingStation> stations = new ArrayList<>();
        ChargingStationModelLoader loader = new ChargingStationModelLoader();
        for (ChargingStationConfig config : configs) {
            ChargingStationModelConfig model = loader.load(config.getModel());
            // Проверка дали позицията на станцията е в рамките на периметъра
            if (config.getX() < 0 || config.getX() > world.getPerimeter().getSize()) {
                throw new RuntimeException("Station X out of bounds. x=" + config.getX() + ", perimeter=" + world.getPerimeter());
            }
            // Създаване на позиция за станцията
            Position position = new Position(config.getX(), config.getY(), 0.0); // stations are on ground

            stations.add(new ChargingStation(model.getName(), model.getPrice(), model.getSlots(),
                    model.getPower(), model.getFailureProbability(), position));
        }
        return stations;
    }
}