package bg.uni.sofia.fmi.simulator.behavior.planning;

import bg.uni.sofia.fmi.simulator.behavior.energyManagment.StationSelector;
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

    public SimplePlanningController(EnergyManager energyManager, PatrolModel patrolModel, Navigation navigation) {
        this.energyManager = energyManager;
        this.patrolModel = patrolModel;
        this.navigation = navigation;
    }

    @Override
    public void update(Bot bot, int currentTime) {
        if (currentTime < 2500) {
            logger.fine(" Bot " + bot.getId() + " at " + bot.getPosition() + " state: " + bot.getState() + bot.getBattery());
        }

        Position goalPosition = null;
        Direction movementDirection = null;
        if (bot.getBattery().isEmpty() || bot.getState() == BotState.ERROR) {//ERROR
            bot.setState(BotState.ERROR);
            patrolModel.onPatrolSuspended(bot);
        } else if (bot.getState() == BotState.CHARGING) {
            patrolModel.onPatrolSuspended(bot);
            if(energyManager.change(bot)){ //зарежда бота и връща дали батерията е пълна
                bot.setState(BotState.GOING_TO_PATROL);
            }
        } else if (energyManager.isLow(bot)) {
            patrolModel.onPatrolSuspended(bot);
            ChargingStation best = energyManager.selectStation(bot);
            if (best == null) {
                bot.setState(BotState.WAITING_FOR_CHARGE);
            } else {
                bot.setState(BotState.GOING_TO_CHARGE);
                double distX = GeometryUtils.perimeterDistance(bot.getPosition(), best.getLocation());
                if (distX < 1.0) {
                    if (energyManager.tryToCharge(best, bot)) {
                        bot.setState(BotState.CHARGING);
                    } else { // чака
                        bot.setState(BotState.WAITING_FOR_CHARGE);
                    }
                } else {
                    // Насочваме се към станцията
                    goalPosition = best.getLocation();
                }
            }
        } else {
            patrolModel.onPatrolResumed(bot);
            bot.setState(BotState.PATROLLING);
            goalPosition = patrolModel.execute(bot, currentTime);
            movementDirection = patrolModel.getMovementDirection();
        }

        if (goalPosition != null) {
            navigation.moveTowards(bot, goalPosition, movementDirection);
        }
        if (currentTime < 2500) { logger.fine(goalPosition != null ? " " + goalPosition : ""); }
    }

    @Override
    public Navigation getNavigation() { return navigation; }

    @Override
    public PatrolModel getPatrolModel() { return patrolModel; }
}
