package bg.uni.sofia.fmi.simulator.behavior.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;
import bg.uni.sofia.fmi.simulator.behavior.energyManagment.EnergyManager;
import bg.uni.sofia.fmi.simulator.behavior.navigation.Navigation;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import bg.uni.sofia.fmi.simulator.strategy.patrol.Direction;

import java.util.logging.Logger;

public class SimplePlanningController implements PlanningModule {
    Logger logger = Logger.getLogger(getClass().getName());
    private final EnergyManager energyManager;
    private final PatrolModel patrolModel;
    private final Navigation navigation;

    private ChargingStation currentStation; // за да знаем на коя станция се зареждаме

    public SimplePlanningController(EnergyManager energyManager, PatrolModel patrolModel, Navigation navigation) {
        this.energyManager = energyManager;
        this.patrolModel = patrolModel;
        this.navigation = navigation;
    }

    @Override
    public void update(Bot bot, int currentTime) {
        if(currentTime < 2500) { logger.fine(" Bot " + bot.getId() + " at " + bot.getPosition() +
                " state: " + bot.getState() + bot.getBattery()); }
        Position goalPosition = null;
        Direction movementDirection = null;
        // Ако батерията е празна, ботът влиза в грешка и не може да прави нищо друго
        if (bot.getBattery().isEmpty()) {
            bot.setState(BotState.ERROR);
            patrolModel.onPatrolSuspended(bot);
        } else if (bot.getState() == BotState.CHARGING) {
            // Ако ботът е в процес на зареждане
            patrolModel.onPatrolSuspended(bot);
            change(bot);
        } else if (energyManager.isLow(bot)) {
            // Ако батерията е ниска
            patrolModel.onPatrolSuspended(bot);
            goalPosition = lowBattery(bot);
        } else {
            // Ако батерията е достатъчна, продължаваме с патрулирането
            patrolModel.onPatrolResumed(bot);
            bot.setState(BotState.PATROLLING);
            goalPosition = patrolModel.execute(bot, currentTime);
            movementDirection = patrolModel.getMovementDirection();
        }
        if(goalPosition != null){
            navigation.moveTowards(bot, goalPosition, movementDirection);
        }
        if(currentTime < 2500) { logger.fine(goalPosition != null ? " " + goalPosition : ""); }
    }

    private void change(Bot bot){
        // Зареждаме бота
        currentStation.chargeBot(bot);
        // Ако батерията е пълна, освобождаваме слота и се връщаме към патрулиране
        if (bot.getBattery().isFull()) {
            currentStation.releaseSlot(bot);
            currentStation = null;
            bot.setState(BotState.PATROLLING);
            patrolModel.onPatrolResumed(bot);
        }
    }

    private Position lowBattery(Bot bot) {
        // Избираме най-добрата станция за зареждане
        StationSelector selector = new StationSelector();
        ChargingStation best = selector.selectBestStation(bot);
        // Ако няма налична станция
        if (best == null) {
            bot.setState(BotState.WAITING_FOR_CHARGE);
            // Ще потърси отново на следващия ход
            return null;
        }
        bot.setState(BotState.GOING_TO_CHARGE);
        Position goalPosition = best.getLocation();

        double distX = GeometryUtils.perimeterDistance(bot.getPosition(), goalPosition);
        if (distX < 1.0) {
            boolean canCharge = best.tryOccupySlot(bot);
            if (canCharge) {
                currentStation = best;
                bot.setState(BotState.CHARGING);
            } else {
                // чака, [TODO] но ако е заето повече от 5 цикъла, търси друга станция
                bot.setState(BotState.WAITING_FOR_CHARGE);
            }
            return null;
        } else {
            // Насочваме се към станцията
            return goalPosition;
        }
    }

    @Override
    public Navigation getNavigation() {
        return navigation;
    }

    @Override
    public PatrolModel getPatrolModel() { return patrolModel; }
}
