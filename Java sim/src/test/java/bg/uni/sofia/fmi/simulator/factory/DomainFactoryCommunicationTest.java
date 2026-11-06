package bg.uni.sofia.fmi.simulator.factory;

import bg.uni.sofia.fmi.simulator.config.ConfigLoader;
import bg.uni.sofia.fmi.simulator.config.RobotConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.strategy.patrol.Direction;
import bg.uni.sofia.fmi.simulator.strategy.patrol.EncounterPatrol;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.*;

public class DomainFactoryCommunicationTest {
    @Before
    public void resetWorld() {
        World.reset();
        IdGenerator.reset();
        RandomProvider.setSeed(42);
    }

    @After
    public void clearWorld() {
        World.reset();
        IdGenerator.reset();
    }

    @Test
    public void factoryBotsCompleteAnEncounterThroughWorldCommunicationRounds() {
        var config = new ConfigLoader().load("configs/scenarios/base.yaml");
        config.getPatrolModel().setModel("EncounterPatrol");
        config.getSimulation().setChargeThreshold(0);
        config.getAttackModel().setLambda(0.0);
        config.getRobots().get(0).setCount(1);
        RobotConfig drone = new RobotConfig();
        drone.setModel("drone");
        drone.setCount(1);
        config.setRobots(List.of(config.getRobots().get(0), drone));
        World world = DomainFactory.createWorld(config);
        Bot slow = World.getBots().get(0);
        Bot fast = World.getBots().get(1);
        // Both start at their deployment boundary, within mutual LiDAR and radio range.
        double boundary = config.getPerimeter().getSize() / 2.0;
        slow.getPosition().setX(boundary);
        fast.getPosition().setX(boundary);
        EncounterPatrol slowPatrol = (EncounterPatrol) slow.getPlanningModule().getPatrolModel();
        EncounterPatrol fastPatrol = (EncounterPatrol) fast.getPlanningModule().getPatrolModel();

        world.tick(0);

        assertEquals(1, slowPatrol.getCompletedEncounterCount());
        assertEquals(1, fastPatrol.getCompletedEncounterCount());
        assertEquals(fast.getId(), slowPatrol.getPairing().orElseThrow().peerState().senderId());
        assertEquals(slow.getId(), fastPatrol.getPairing().orElseThrow().peerState().senderId());
        assertTrue(slowPatrol.getReceivedStates().containsKey(fast.getId()));
        assertTrue(fastPatrol.getReceivedStates().containsKey(slow.getId()));
        assertEquals(Direction.RIGHT, slowPatrol.getMovementDirection());
        assertEquals(Direction.LEFT, fastPatrol.getMovementDirection());
        assertEquals(boundary + slow.getMaxSpeed(), slow.getPosition().getX(), 1e-9);
        assertEquals(boundary - fast.getMaxSpeed(), fast.getPosition().getX(), 1e-9);

        world.tick(1);

        assertEquals(1, slowPatrol.getCompletedEncounterCount());
        assertEquals(1, fastPatrol.getCompletedEncounterCount());
        assertEquals(boundary + 2 * slow.getMaxSpeed(), slow.getPosition().getX(), 1e-9);
        assertEquals(boundary - 2 * fast.getMaxSpeed(), fast.getPosition().getX(), 1e-9);
        World.finish();
    }
}
