package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.planning.Navigation;

public class RingPatrol implements PatrolModel {
    private Navigation navigation;
    private Integer robotsPerSection;

    public RingPatrol(PatrolConfig config) {
        this.robotsPerSection = config.getRobotsPerSection();
        this.navigation = new Navigation(new bg.uni.sofia.fmi.simulator.planning.ObstacleAvoidance());
    }

    @Override
    public void initialize(List<Bot> bots, World world) {
        // Optionally distribute bots evenly
    }

    @Override
    public void execute(List<Bot> bots, World world) {
        for (Bot bot : bots) {
            bot.getBehavior().getNavigation().moveTowards(bot, world); // simple continuous movement
        }
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
    
}
