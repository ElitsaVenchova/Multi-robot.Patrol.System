package bg.uni.sofia.fmi.simulator.factory;

import java.util.Random;

import bg.uni.sofia.fmi.simulator.config.RobotConfig;
import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.Battery;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Lidar;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

/**
 * Factory class responsible for creating domain objects based on the simulation configuration.
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

        Random random = new Random(
                config.getSimulation().getSeed() != null
                        ? config.getSimulation().getSeed()
                        : System.currentTimeMillis()
        );

        for (RobotConfig robotConfig : config.getRobots()) {

            for (int i = 0; i < robotConfig.getCount(); i++) {

                Bot bot = createBot(robotConfig, random, world);

                world.addBot(bot);
            }
        }
    }

    private static Bot createBot(RobotConfig config, Random random, World world) {

        // TODO: later load real robot models from YAML
        // For now: default values

        Position position = new Position(RandomProvider.nextDouble() * world.getPerimeterSize());

        Battery battery = new Battery(
                100.0,   // capacity
                0.1      // consumption rate
        );

        Lidar lidar = new Lidar(
                5.0      // detection range
        );

        double speed = 1.0;

        return new Bot(position, battery, lidar, speed);
    }
}