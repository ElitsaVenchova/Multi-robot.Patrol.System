package bg.uni.sofia.fmi.simulator.factory;

import bg.uni.sofia.fmi.simulator.config.RobotConfig;
import bg.uni.sofia.fmi.simulator.config.RobotModelConfig;
import bg.uni.sofia.fmi.simulator.config.RobotModelLoader;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.Battery;
import bg.uni.sofia.fmi.simulator.domain.Bot;
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

        createBots(world, config);

        return world;
    }

    private static void createBots(World world, SimulationConfig config) {
        for (RobotConfig robotConfig : config.getRobots()) {

            for (int i = 0; i < robotConfig.getCount(); i++) {

                Bot bot = createBot(robotConfig, world);

                world.addBot(bot);
            }
        }
    }

    private static Bot createBot(RobotConfig config, World world) {

        RobotModelLoader loader = new RobotModelLoader();
        RobotModelConfig model = loader.load(config.getModel());

        // [TODO] Позицията може да се задава в конфигурацията или да се генерира 
        // на базата на броя ботове и размера на периметъра, за да се избегне струпване  
        Position position = new Position(
                RandomProvider.nextDouble() * world.getPerimeterSize());

        Battery battery = new Battery(
                model.getBatteryCapacity(),
                model.getBatteryConsumption());

        Lidar lidar = new Lidar(model.getLidarRange());

        return new Bot(position, battery, lidar, model.getSpeed());
    }
}