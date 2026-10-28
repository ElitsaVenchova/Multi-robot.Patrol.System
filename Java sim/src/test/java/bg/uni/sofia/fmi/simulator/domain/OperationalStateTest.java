package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.config.*;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.factory.DomainFactory;
import bg.uni.sofia.fmi.simulator.util.*;
import org.junit.Test;
import java.util.List;
import static org.junit.Assert.*;

public class OperationalStateTest {
    private World setup(double threshold, int count) {
        var config = new ConfigLoader().load("configs/scenarios/base.yaml");
        config.getRobots().get(0).setCount(count); config.getSimulation().setChargeThreshold(threshold);
        ChargingStationConfig station = new ChargingStationConfig(); station.setModel("cheap_station"); station.setX(1);
        config.setChargingStations(List.of(station)); IdGenerator.reset(); RandomProvider.setSeed(42);
        return DomainFactory.createWorld(config);
    }
    @Test public void queuedRobotIsWaitingAndTravelingRobotIsTraveling() {
        World world = setup(20, 2);
        World.getBots().forEach(b -> b.getBattery().consume(90));
        world.tick(0); world.tick(1); world.tick(2); world.finish();
        Bot waiting = World.getBots().get(1);
        assertEquals(1L, (long) waiting.getStateTicks().get(BotState.GOING_TO_CHARGE));
        assertEquals(2L, (long) waiting.getStateTicks().get(BotState.WAITING_FOR_CHARGE));
        assertEquals(3, waiting.getStateTicks().values().stream().mapToLong(Long::longValue).sum());
    }
    @Test public void tickThatCompletesChargingStillCountsAsCharging() {
        World world = setup(100, 1);
        World.getBots().get(0).getBattery().consume(0.5);
        world.tick(0); world.tick(1); world.finish();
        Bot bot = World.getBots().get(0);
        assertEquals(BotState.PATROLLING, bot.getState());
        assertEquals(2L, (long) bot.getStateTicks().get(BotState.CHARGING));
    }
}
