package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;

public class Navigation {
    private ObstacleAvoidance obstacleAvoidance;

    public Navigation(ObstacleAvoidance obstacleAvoidance) {
        this.obstacleAvoidance = obstacleAvoidance;
    }

    public void patrol(Bot bot, World world) {

        if (bot.getState() != BotState.PATROLLING) {
            return;
        }

        // avoid obstacles (currently dummy)
        obstacleAvoidance.avoid(bot, world);

        // simple movement for now
        bot.move();
    }

    public void goToChargingStation(Bot bot, World world) {

        ChargingStation station = findNearestStation(bot, world);

        if (station == null)
            return;

        moveTowards(bot, station.getLocation());
    }

    private ChargingStation findNearestStation(Bot bot, World world) {

        ChargingStation best = null;
        double bestDist = Double.MAX_VALUE;

        for (ChargingStation s : world.getChargingStations()) {

            double dx = bot.getPosition().getX() - s.getLocation().getX();
            double dy = bot.getPosition().getY() - s.getLocation().getY();

            double dist = Math.sqrt(dx * dx + dy * dy);

            if (dist < bestDist) {
                bestDist = dist;
                best = s;
            }
        }

        return best;
    }

    public void moveForward(Bot bot) {
        bot.move();
    }

    public void moveTowards(Bot bot, Position target) {

        Position p = bot.getPosition();

        double dx = target.getX() - p.getX();
        double dy = target.getY() - p.getY();

        double step = bot.getMaxSpeed();

        double length = Math.sqrt(dx * dx + dy * dy);

        if (length == 0)
            return;

        p.setX(p.getX() + step * dx / length);
        p.setY(p.getY() + step * dy / length);
    }
}
