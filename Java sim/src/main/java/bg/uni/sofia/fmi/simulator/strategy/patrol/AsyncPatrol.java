package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;
import java.util.Random;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;

public class AsyncPatrol implements PatrolModel {

    private Random random = new Random();

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
}