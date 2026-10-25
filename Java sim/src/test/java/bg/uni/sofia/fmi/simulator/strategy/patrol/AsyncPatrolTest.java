package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.behavior.collisionAvoidance.ObstacleAvoidance;
import bg.uni.sofia.fmi.simulator.behavior.navigation.Navigation;
import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.*;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class AsyncPatrolTest {

    @Test
    public void lastCircularPatrolSectionCrossesPerimeterSeam() {
        PatrolConfig config = new PatrolConfig();
        config.setDeviationProbability(0);
        config.setMaxDeviationDuration(1);
        AsyncPatrol patrol = new AsyncPatrol(config);

        World world = new World();
        World.setPerimeter(new Perimeter(1000, PerimeterType.CIRCULAR));
        List<Bot> bots = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            bots.add(new Bot(100 + i, new Position(i), new Battery(1000, 1), new Lidar(0, 0),
                    new CommunicationModule(10, 1, 0, 1), 10, RobotType.GROUND, "bot-" + i,
                    0, 0, 0, null));
        }
        world.addBots(bots);

        Bot lastBot = bots.get(4);
        lastBot.getPosition().setX(999);
        patrol.initialize(lastBot);

        assertEquals(800, patrol.getPatrolSection().startPosition().getX(), 0);
        assertEquals(1000, patrol.getPatrolSection().endPosition().getX(), 0);
        assertTrue(lastBot.isInPatrolSection(patrol.getPatrolSection()));

        Position target = patrol.getPatrolTarget(lastBot);
        assertEquals(1000, target.getX(), 0);

        new Navigation(new ObstacleAvoidance()).moveTowards(lastBot, target);
        assertEquals(0, lastBot.getPosition().getX(), 0);
        assertTrue(lastBot.isInPatrolSection(patrol.getPatrolSection()));
        assertEquals(800, patrol.getPatrolTarget(lastBot).getX(), 0);
    }
}
