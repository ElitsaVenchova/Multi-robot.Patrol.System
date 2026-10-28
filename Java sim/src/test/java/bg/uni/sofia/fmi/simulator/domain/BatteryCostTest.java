package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.*;
import org.junit.Test;
import static org.junit.Assert.*;

public class BatteryCostTest {
    @Test public void actualThroughputAndFractionalCycleDepreciation() {
        Battery battery = new Battery(100, 5000); battery.setWearModel(1000.0, 200.0);
        battery.consume(25); assertEquals(0.05, battery.getWearCostEur(), 1e-12);
        battery.charge(1000); assertEquals(25, battery.getChargedWh(), 0);
        battery.consume(1000); battery.consume(10);
        assertEquals(125, battery.getDischargedWh(), 0);
        assertEquals(1.25, battery.getEquivalentDischargeCycles(), 0);
        assertEquals(0.25, battery.getWearCostEur(), 1e-12);
    }
    @Test public void chargingPowerConvertsWattsToWhPerMinuteAndClipsAtCapacity() {
        Battery battery = new Battery(100, 5000); battery.consume(2);
        Bot robot = new Bot(1, new Position(0), battery, new Lidar(1, 0),
                new CommunicationModule(1, 1, 0, 1), 1, RobotType.GROUND, "test", 0, 1, 0, null);
        ChargingStation station = new ChargingStation("charger", 1, 1, 300, 0, new Position(0));
        station.chargeBot(robot);
        assertEquals(100, battery.getCurrentLevel(), 0); assertEquals(2, battery.getChargedWh(), 0);
    }
    @Test public void capacityVariantsCanHaveDifferentReplacementCosts() {
        Battery small = new Battery(50, 5000), large = new Battery(100, 5000);
        small.setWearModel(1000.0, 100.0); large.setWearModel(1000.0, 300.0);
        small.consume(25); large.consume(25);
        assertEquals(0.05, small.getWearCostEur(), 1e-12); assertEquals(0.075, large.getWearCostEur(), 1e-12);
    }
}
