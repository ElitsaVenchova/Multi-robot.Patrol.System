package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;

public class SimpleBehaviorController implements BehaviorModule {
    private EnergyManager energyManager;
    private PatrolModel patrolModel;
    private Navigation navigation;

    public SimpleBehaviorController(EnergyManager energyManager,PatrolModel patrolModel, Navigation navigation) {
        this.energyManager = energyManager;
        this.patrolModel = patrolModel;
        this.navigation = navigation;
    }

    @Override
    public void update(Bot bot, int currentTime) {
        // Ако батерията е празна, ботът влиза в грешка и не може да прави нищо друго
        if (bot.getBattery().isEmpty()) {
            bot.setState(BotState.ERROR);
            return;
        }

        // Ако ботът е в процес на зареждане
        if (bot.getState() == BotState.CHARGING) {
            // Зареждаме бота
            bot.getCurrentStation().chargeBot(bot);

            // Ако батерията е пълна, освобождаваме слота и се връщаме към патрулиране
            if (bot.getBattery().isFull()) {
                bot.getCurrentStation().releaseSlot(bot);
                bot.setCurrentStation(null);

                bot.setState(BotState.PATROLLING);
            }
            return;
        }

        // Ако батерията е ниска
        if (energyManager.isLow(bot)) {
            // Избираме най-добрата станция за зареждане
            StationSelector selector = new StationSelector();
            ChargingStation best = selector.selectBestStation(bot);
            // Ако няма налична станция
            if (best == null) {
                // Ще потърси отново на следващия ход
                // bot.setState(BotState.ERROR);
                return;
            }
            bot.setState(BotState.GOING_TO_CHARGE);
            bot.setGoalPosition(best.getLocation());

            double distX = distanceX(bot.getPosition(), bot.getGoalPosition());
            if (distX < 1.0) {
                boolean canCharge = best.tryOccupySlot(bot);
                if (canCharge) {
                    bot.setGoalPosition(null);
                    bot.setCurrentStation(best);
                    bot.setState(BotState.CHARGING);
                } else {
                    // чака, [TODO] но ако е заето повече от 5 цикъла, търси друга станция
                    bot.setState(BotState.GOING_TO_CHARGE);
                }
                return;
            }
            // Насочваме се към станцията
            navigation.goToChargingStation(bot);

            return;
        }

        // Ако батерията е достатъчна, продължаваме с патрулирането
        bot.setState(BotState.PATROLLING);
        patrolModel.execute(bot);
    }

    private double distanceX(Position a, Position b) {
        return Math.abs(a.getX() - b.getX());
    }

    @Override
    public Navigation getNavigation() {
        return navigation;
    }
}
