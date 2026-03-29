package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;

public class RingPatrol implements PatrolModel  {
    @Override
    public void initialize(List<Bot> bots, World world) {
        // Optionally distribute bots evenly
    }

    @Override
    public void execute(List<Bot> bots, World world) {
        for (Bot bot : bots) {
            bot.move(); // simple continuous movement
        }
    }
}
