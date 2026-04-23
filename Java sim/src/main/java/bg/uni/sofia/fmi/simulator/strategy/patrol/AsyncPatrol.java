package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.planning.Navigation;

public class AsyncPatrol implements PatrolModel {

    private Integer robotsPerSection;

    public AsyncPatrol(PatrolConfig config) {
        this.robotsPerSection = config.getRobotsPerSection();
    }
  
    @Override
    public void initialize(List<Bot> bots, World world) {
        // optional random offsets
    }

    @Override
    public void execute(List<Bot> bots, World world, Navigation navigation) {
        for (Bot bot : bots) {
            if (bot.getPosition().getX() > world.getPerimeter().getSize() * 0.9) {
                bot.setDirection(-1);
            }

            if (bot.getPosition().getX() < world.getPerimeter().getSize() * 0.1) {
                bot.setDirection(1);
            }
            bot.getBehavior().getNavigation().moveTowards(bot, world);
            navigation.moveForward(bot, world);
            bot.move();
        }
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}