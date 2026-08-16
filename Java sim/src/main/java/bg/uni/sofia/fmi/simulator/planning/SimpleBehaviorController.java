package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;

public class SimpleBehaviorController implements BehaviorModule {
    private final EnergyManager energyManager;
    private final PatrolModel patrolModel;
    private final Navigation navigation;

    private ChargingStation currentStation; // за да знаем на коя станция се зареждаме
    private final World world; // референция към света, в който се намира бота. В бъдеще ще е света, в който си мисли, че се намира спрямо събраната информация от сензорит/комуникация

    public SimpleBehaviorController(EnergyManager energyManager,PatrolModel patrolModel, Navigation navigation, World world) {
        this.energyManager = energyManager;
        this.patrolModel = patrolModel;
        this.navigation = navigation;
        this.world = world;
    }

    @Override
    public void update(Bot bot, int currentTime) {
        // Ако батерията е празна, ботът влиза в грешка и не може да прави нищо друго
        if (bot.getBattery().isEmpty()) {
//             System.out.println("Bot " + bot.getId() + " at " + bot.getPosition() + " state: " + bot.getState() + " battery level: " + bot.getBattery().getCurrentLevel() +
//                              " goal position: " + bot.getGoalPosition() + " current station: " + (bot.getCurrentStation() != null ? bot.getCurrentStation().getName() : "None"));
            bot.setState(BotState.ERROR);
        } else if (bot.getState() == BotState.CHARGING) {
            // Ако ботът е в процес на зареждане
            change(bot);
        } else if (energyManager.isLow(bot)) {
            // Ако батерията е ниска
            lowBattery(bot);
        } else {
            // Ако батерията е достатъчна, продължаваме с патрулирането
            bot.setState(BotState.PATROLLING);
            patrolModel.execute(bot);
        }
    }

    private void change(Bot bot){
        // Зареждаме бота
        currentStation.chargeBot(bot);
        // Ако батерията е пълна, освобождаваме слота и се връщаме към патрулиране
        if (bot.getBattery().isFull()) {
            currentStation.releaseSlot(bot);
            currentStation = null;
            bot.setState(BotState.PATROLLING);
        }
    }

    private void lowBattery(Bot bot) {
        // Избираме най-добрата станция за зареждане
        StationSelector selector = new StationSelector();
        ChargingStation best = selector.selectBestStation(bot);
        // Ако няма налична станция
        if (best == null) {
            // Ще потърси отново на следващия ход
            return;
        }
        bot.setState(BotState.GOING_TO_CHARGE);
        Position goalPosition = best.getLocation();

        double distX = distanceX(bot.getPosition(), goalPosition);
        if (distX < 1.0) {
            boolean canCharge = best.tryOccupySlot(bot);
            if (canCharge) {
                goalPosition = null;
                currentStation = best;
                bot.setState(BotState.CHARGING);
            } else {
                // чака, [TODO] но ако е заето повече от 5 цикъла, търси друга станция
                bot.setState(BotState.GOING_TO_CHARGE);
            }
        } else {
            // Насочваме се към станцията
            navigation.moveTowards(bot, world, goalPosition);
        }
    }

    private double distanceX(Position a, Position b) {
        return Math.abs(a.getX() - b.getX());
    }

    @Override
    public Navigation getNavigation() {
        return navigation;
    }
}
