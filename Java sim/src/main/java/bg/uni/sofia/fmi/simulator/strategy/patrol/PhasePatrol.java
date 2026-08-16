package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;

//Патрулиране по секции като всички роботи се движат едновременно в една или друга посока като
// така поддържат еднакво разстояние помежду си
public class PhasePatrol implements PatrolModel {


    public PhasePatrol(PatrolConfig config) {

    }

    @Override
    public void initialize(Bot bot, World world) {
        // assign directions (even/odd)
    }

    @Override
    public void execute(Bot bot, World world, int currentTime) {
        double step = bot.getMaxSpeed();
        double perimeter = world.getPerimeter().getSize();
        double targetX;

        if (bot.getId() % 2 == 0) {
            targetX = bot.getPosition().getX() - step;
        } else {
            targetX = bot.getPosition().getX() + step;
        }
        targetX = Math.max(0, Math.min(targetX, perimeter));
        Position target = new Position(targetX);
        
        bot.getBehavior().getNavigation().moveTowards(bot, target);

        PatrolModel.scan(bot, world, currentTime);
    }
}