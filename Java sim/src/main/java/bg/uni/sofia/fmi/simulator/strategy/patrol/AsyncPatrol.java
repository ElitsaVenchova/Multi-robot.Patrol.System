package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;

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
    public void execute(List<Bot> bots, World world) {
        for (Bot bot : bots) {
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