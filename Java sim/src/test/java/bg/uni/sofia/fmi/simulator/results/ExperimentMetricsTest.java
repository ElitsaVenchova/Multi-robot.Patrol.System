package bg.uni.sofia.fmi.simulator.results;

import bg.uni.sofia.fmi.simulator.config.*;
import bg.uni.sofia.fmi.simulator.domain.*;
import bg.uni.sofia.fmi.simulator.domain.enums.*;
import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.behavior.planning.PlanningModule;
import bg.uni.sofia.fmi.simulator.behavior.navigation.Navigation;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import java.util.List;
import org.junit.Test;
import static org.junit.Assert.*;

public class ExperimentMetricsTest {
    @Test public void knownWorldHasIndependentOutcomeActivityAndCostExpectations() {
        World.reset();
        World world = new World();
        World.setPerimeter(new Perimeter(100, PerimeterType.LINEAR));
        world.setAttackModel(time -> List.of());
        Bot first = accountingRobot(1, 100, 5);
        Bot second = accountingRobot(2, 200, 1);
        first.getBattery().consume(25); first.getBattery().charge(10);
        second.getBattery().consume(50); second.getBattery().charge(20);
        world.addBots(List.of(first, second));
        world.addChargingStations(List.of(new ChargingStation("fixture", 40, 1, 60, 0, new Position(0))));
        Attack a = new Attack(new Position(0), 0, null); a.intercept(2);
        Attack b = new Attack(new Position(0), 1, null); b.intercept(5);
        for (Attack attack : List.of(a, b, new Attack(new Position(0), 0, 6),
                new Attack(new Position(0), 5, 10), new Attack(new Position(0), 0, null))) {
            World.getPerimeter().addAttack(attack);
        }
        for (int t = 0; t < 6; t++) world.tick(t);
        var config = new ConfigLoader().load("configs/scenarios/base.yaml");
        config.setEnergyPriceEurPerKwh(0.2);
        var metrics = new MetricsCalculator().calculate(config);

        assertEquals(5, metrics.getTotalAttacks());
        assertEquals(2, metrics.getInterceptedAttacks());
        assertEquals(2, metrics.getMissedAttacks());
        assertEquals(1, metrics.getActiveAttacks());
        assertEquals(0.5, metrics.getCaptureCoefficient(), 1e-12);
        assertEquals(0.4, metrics.getSuccessRate(), 1e-12);
        assertEquals(3, metrics.getAverageDetectionTime(), 1e-12);
        // Six patrol minutes / six other minutes; the mean robot ratio would be 2.6.
        assertEquals(1, metrics.getActivityCoefficient(), 1e-12);
        assertEquals(5L, (long) metrics.getRobots().get(0).stateMinutes().get(BotState.PATROLLING));
        assertEquals(1L, (long) metrics.getRobots().get(1).stateMinutes().get(BotState.PATROLLING));
        assertEquals(6, metrics.getRobots().get(0).totalMinutes());
        assertEquals(6, metrics.getRobots().get(1).totalMinutes());
        assertEquals(300, metrics.getRobotAcquisitionCost(), 0);
        assertEquals(40, metrics.getStationAcquisitionCost(), 0);
        assertEquals(0.006, metrics.getEnergyCost(), 1e-12);
        assertEquals(0.15, metrics.getBatteryReplacementCost(), 1e-12);
        assertEquals(340.156, metrics.getTco(), 1e-12);
        assertEquals(0.25, metrics.getRobots().get(0).equivalentDischargeCycles(), 0);
        assertEquals(0.5, metrics.getRobots().get(1).equivalentDischargeCycles(), 0);

        // Finalization is idempotent, and later worlds cannot mutate captured metrics.
        assertEquals(MetricsAggregator.experimentValues(metrics),
                MetricsAggregator.experimentValues(new MetricsCalculator().calculate(config)));
        World.reset();
        assertEquals(2, metrics.getRobots().size());
        assertEquals(6, metrics.getRobots().get(0).totalMinutes());
    }

    private Bot accountingRobot(long id, double price, int patrolTicks) {
        PlanningModule planner = new PlanningModule() {
            public void update(Bot bot, int time) {
                bot.setState(time < patrolTicks ? BotState.PATROLLING : BotState.WAITING_FOR_CHARGE);
            }
            public Navigation getNavigation() { return null; }
            public PatrolModel getPatrolModel() { return null; }
        };
        Battery battery = new Battery(100, 5000);
        battery.setWearModel(1000.0, 200.0);
        Bot bot = new Bot(id, new Position(0), battery, new Lidar(0, 0),
                new CommunicationModule(1, 1, 0, 1), 1, RobotType.GROUND, "fixture", 0, price, 0, planner);
        bot.setBaselineConsumptionWh(0);
        return bot;
    }

    @Test public void populationStatisticsKeepUndefinedAndSingletonCasesDistinct() {
        var defined = MetricsAggregator.statistic(List.of(1.0, 3.0, Double.NaN));
        assertEquals(2, defined.count());
        assertEquals(2, defined.mean(), 0);
        assertEquals(1, defined.std(), 0); // Population SD, not sample SD sqrt(2).
        var singleton = MetricsAggregator.statistic(List.of(Double.NaN, 4.0));
        assertEquals(1, singleton.count());
        assertEquals(4, singleton.mean(), 0);
        assertEquals(0, singleton.std(), 0);
        var undefined = MetricsAggregator.statistic(List.of(Double.NaN));
        assertEquals(0, undefined.count());
        assertTrue(Double.isNaN(undefined.mean()));
        assertTrue(Double.isNaN(undefined.std()));
    }

    @Test public void unexpiredOnlyRunHasUndefinedCapture() {
        World.reset(); World world = new World(); World.setPerimeter(new Perimeter(100, PerimeterType.LINEAR));
        World.getPerimeter().addAttack(new Attack(new Position(0), 0, 10));
        var m = new MetricsCalculator().calculate(new ConfigLoader().load("configs/scenarios/base.yaml"));
        assertEquals(1, m.getActiveAttacks()); assertTrue(Double.isNaN(m.getCaptureCoefficient()));
    }
    @Test public void resolvesExpiredAndDurationlessButExcludesUnexpired() {
        World.reset(); World world = new World(); World.setPerimeter(new Perimeter(100, PerimeterType.LINEAR));
        Attack captured = new Attack(new Position(0), 0, null); captured.intercept(0);
        Attack expired = new Attack(new Position(0), 0, 0);
        Attack unresolved = new Attack(new Position(0), 0, 10);
        Attack indefinite = new Attack(new Position(0), 0, null);
        for (Attack a : new Attack[]{captured, expired, unresolved, indefinite}) World.getPerimeter().addAttack(a);
        var config = new ConfigLoader().load("configs/scenarios/base.yaml");
        var m = new MetricsCalculator().calculate(config);
        assertEquals(4, m.getTotalAttacks()); assertEquals(1, m.getInterceptedAttacks());
        assertEquals(2, m.getMissedAttacks()); assertEquals(1, m.getActiveAttacks());
        assertEquals(1.0 / 3, m.getCaptureCoefficient(), 1e-12);
    }
    @Test public void undefinedCaptureAndExactlyOneStatePerTick() {
        var config = new ConfigLoader().load("configs/scenarios/base.yaml");
        config.getSimulation().setDuration(9); config.getAttackModel().setLambda(0.0);
        var m = new SimulationRunner().runWithResult(config, false);
        assertTrue(Double.isNaN(m.getCaptureCoefficient()));
        assertEquals(45, m.getRobots().stream().mapToLong(RobotMetrics::totalMinutes).sum());
        for (var r : m.getRobots()) assertEquals(9, r.totalMinutes());
        long patrol = m.getRobots().stream().mapToLong(RobotMetrics::patrolMinutes).sum();
        assertEquals((double) patrol / (45 - patrol), m.getActivityCoefficient(), 0);
    }
    @Test public void expiryIncludesBoundaryAndPreservesInterceptedStatus() {
        Attack attack = new Attack(new Position(0), 9, 1); attack.resolveAtBoundary(10);
        assertEquals(AttackStatus.MISSED, attack.getStatus());
        Attack intercepted = new Attack(new Position(0), 0, null); intercepted.intercept(2); intercepted.resolveAtBoundary(10);
        assertEquals(AttackStatus.INTERCEPTED, intercepted.getStatus());
    }
    @Test public void emptyBatteryCountsErrorForEveryTick() {
        var config = new ConfigLoader().load("configs/scenarios/base.yaml");
        config.getSimulation().setDuration(3); config.getSimulation().setChargeThreshold(0);
        RobotModelConfig model = new RobotModelLoader().load("turtlebot_pi");
        model.getBattery().setBatteryCapacity(0.001); config.getRobotModels().put("turtlebot_pi", model);
        var m = new SimulationRunner().runWithResult(config, false);
        for (var robot : m.getRobots()) {
            assertEquals(3, robot.totalMinutes()); assertEquals(2L, (long) robot.stateMinutes().get(BotState.ERROR));
        }
    }
}
