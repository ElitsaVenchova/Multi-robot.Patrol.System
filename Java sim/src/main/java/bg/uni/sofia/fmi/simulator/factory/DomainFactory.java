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
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

/**
 * Factory class responsible for creating domain objects based on the simulation
 * configuration.
 * - building World
 * - creating Bots based on RobotConfig
 */
public class DomainFactory {

    public static World createWorld(SimulationConfig config) {

        double perimeter = config.getSimulation().getPerimeterSize();
        World world = new World(perimeter);

        // Robots
        List<Bot> bots = createBots(config.getRobots(), world);
        world.addBots(bots);

        // Charging stations
        List<ChargingStation> stations = createStations(config.getChargingStations(), world);

        world.addChargingStations(stations);

        return world;
    }

    public static List<Bot> createBots(List<RobotConfig> robotConfigs, World world) {

        List<Bot> bots = new ArrayList<>();
        RobotModelLoader loader = new RobotModelLoader();

        for (RobotConfig config : robotConfigs) {

            RobotModelConfig model = loader.load(config.getModel());

            for (int i = 0; i < config.getCount(); i++) {
                bots.add(createBot(model, world));
            }
        }

        return bots;
    }

    private static Bot createBot(RobotModelConfig model, World world) {
        // [TODO] Позицията може да се задава в конфигурацията или да се генерира
        // на базата на броя ботове и размера на периметъра, за да се избегне струпване
        Position position = new Position(
                RandomProvider.nextDouble() * world.getPerimeterSize());

        Battery battery = new Battery(
                model.getBatteryCapacity(),
                model.getBatteryConsumption());

        Lidar lidar = new Lidar(model.getLidarRange());

        return new Bot(position, battery, lidar, model.getSpeed(), model.getType(), model.getName());
    }

    public static List<ChargingStation> createStations(
            List<ChargingStationConfig> configs,
            World world) {

        List<ChargingStation> stations = new ArrayList<>();
        ChargingStationModelLoader loader = new ChargingStationModelLoader();

        for (ChargingStationConfig config : configs) {

            ChargingStationModelConfig model = loader.load(config.getModel());

            for (int i = 0; i < config.getCount(); i++) {

                Position position = new Position(
                        RandomProvider.nextDouble() * world.getPerimeterSize());

                stations.add(new ChargingStation(
                        model.getName(),
                        model.getPrice(),
                        model.getSlots(),
                        model.getPower(),
                        model.getFailureProbability(),
                        position));
            }
        }

        return stations;
    }
}