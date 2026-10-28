package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.behavior.navigation.Navigation;
import bg.uni.sofia.fmi.simulator.behavior.collisionAvoidance.ObstacleAvoidance;
import bg.uni.sofia.fmi.simulator.domain.enums.*;
import org.junit.Test;
import static org.junit.Assert.*;

public class NavigationRoundoffTest {
    private Bot robot() {
        World.reset(); World.setPerimeter(new Perimeter(200, PerimeterType.LINEAR));
        return new Bot(1, new Position(63.9), new Battery(100, 5000), new Lidar(1, 0),
                new CommunicationModule(1, 1, 0, 1), 2.5, RobotType.GROUND, "test", 0, 1, 0, null);
    }
    @Test public void acceptsRoundedCoordinateDifferenceAtSpeedLimit() {
        Bot bot = robot(); new Navigation(new ObstacleAvoidance()).moveTowards(bot, new Position(100));
        assertEquals(66.4, bot.getPosition().getX(), 1e-12);
    }
    @Test(expected = RuntimeException.class) public void stillRejectsActualSpeedViolation() {
        robot().move(new Position(67));
    }
}
