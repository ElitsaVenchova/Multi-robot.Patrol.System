package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;
import java.util.Random;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

public class AsyncPatrol implements PatrolModel {

    private Integer robotsPerSection;
    private Random random = RandomProvider.getRandom();

    public AsyncPatrol(PatrolConfig config) {
        this.robotsPerSection = config.getRobotsPerSection();
    }
  
    @Override
    public void initialize(List<Bot> bots, World world) {
        // optional random offsets
    }

    @Override
    public void execute(List<Bot> bots, World world) {
        for (Bot bot : bots) {
            double variation = 0.5 + random.nextDouble();
            bot.getPosition().move(variation);
        }
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}