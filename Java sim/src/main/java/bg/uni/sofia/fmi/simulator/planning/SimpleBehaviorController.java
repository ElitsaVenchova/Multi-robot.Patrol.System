package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;

public class SimpleBehaviorController implements BehaviorModule {
    private EnergyManager energyManager;
    private Navigation navigation;

    public SimpleBehaviorController(EnergyManager energyManager,
            Navigation navigation) {
        this.energyManager = energyManager;
        this.navigation = navigation;
    }

    @Override
    public void update(Bot bot, World world, int currentTime) {

        if (bot.getBattery().isEmpty()) {
            bot.setState(BotState.ERROR);
            return;
        }

        // =========================
        // CHARGING
        // =========================
        if (bot.getState() == BotState.CHARGING) {
            bot.getCurrentStation().chargeBot(bot);

            if (bot.getBattery().isFull()) {

                bot.getCurrentStation().releaseSlot(bot);
                bot.setCurrentStation(null);

                bot.setState(BotState.PATROLLING);
            }

            return;
        }

        // =========================
        // LOW ENERGY
        // =========================
        if (energyManager.isLow(bot)) {

            StationSelector selector = new StationSelector();

            ChargingStation best = selector.selectBestStation(bot, world);

            if (best == null) {
                bot.setState(BotState.ERROR);
                return;
            }

            bot.setTargetStation(best);

            double dist = distance(bot.getPosition(), best.getLocation());

            if (dist < 1.0) {

                boolean canCharge = best.tryOccupySlot(bot);

                if (canCharge) {
                    bot.setCurrentStation(best);
                    bot.setState(BotState.CHARGING);
                } else {
                    // wait in queue
                    bot.setState(BotState.GOING_TO_CHARGE);
                }

                return;
            }

            bot.setState(BotState.GOING_TO_CHARGE);
            navigation.moveTowards(bot, best.getLocation());

            return;
        }

        // =========================
        // PATROL
        // =========================
        bot.setState(BotState.PATROLLING);
        navigation.patrol(bot, world);
    }

    private double distance(Position a, Position b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
