package bg.uni.sofia.fmi.simulator.experiments;

import bg.uni.sofia.fmi.simulator.config.*;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.engine.SimulationRunner;
import bg.uni.sofia.fmi.simulator.results.SimulationMetrics;
import org.junit.Test;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;
import java.nio.file.Files;
import java.util.*;
import static org.junit.Assert.*;

public class ExperimentRunnerTest {
    @Rule public TemporaryFolder folder = new TemporaryFolder();

    @Test public void rejectsIncompletePatrolsBeforeSimulationOrOutput() throws Exception {
        for (String patrol : List.of("PhasePatrol", "CounterPhasePatrol", "RingPatrol", "EncounterPatrol", "UnknownPatrol")) {
            var p = plan();
            var base = new ConfigLoader().load(p.getBaseConfig());
            base.getPatrolModel().setModel(patrol);
            var path = folder.newFile(patrol + ".yaml").toPath();
            Files.writeString(path, ConfigLoader.toYaml(base)); p.setBaseConfig(path.toString());
            List<SimulationConfig> simulated = new ArrayList<>();
            var runner = new ExperimentRunner(config -> { simulated.add(config); return new SimulationMetrics(); });
            try {
                runner.runBatch(p, folder.getRoot().toPath());
                fail("Must reject unsupported experiment patrol " + patrol);
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(patrol));
            }
            assertTrue("Unsupported patrol must not execute", simulated.isEmpty());
            assertFalse("Validation must precede output creation", Files.exists(folder.getRoot().toPath().resolve(p.getExperimentId())));
        }
    }

    static ExperimentConfig plan() {
        ExperimentConfig p = new ExperimentConfig();
        p.setExperimentId("test"); p.setBaseConfig("configs/scenarios/base.yaml"); p.setRepetitions(2);
        ExperimentConfig.Design d = new ExperimentConfig.Design();
        d.setId("ground"); d.setType(ExperimentConfig.Type.ATTACK_DENSITY); d.setSystem(ExperimentConfig.SystemType.TURTLEBOT);
        List<ExperimentConfig.Point> points = new ArrayList<>();
        for (int i = 0; i < 2; i++) {
            ExperimentConfig.Point point = new ExperimentConfig.Point();
            point.setId("p" + i); point.setResourceScenario("base"); point.setAttackProbability((double) i);
            RobotConfig robot = new RobotConfig(); robot.setModel("turtlebot_pi"); robot.setCount(2);
            point.setRobots(List.of(robot)); points.add(point);
        }
        d.setPoints(points); p.setDesigns(List.of(d)); return p;
    }
    @Test public void passesEveryPreparedConfigurationAndSeedToExecution() {
        List<SimulationConfig> seen = new ArrayList<>();
        ExperimentRunner runner = new ExperimentRunner(c -> { seen.add(c); return new SimulationMetrics(); });
        var p = plan(); var runs = runner.execute(p);
        assertEquals(4, runs.size());
        for (int i = 0; i < 4; i++) {
            assertEquals(42L + i % 2, (long) seen.get(i).getSimulation().getSeed());
            assertEquals(2, seen.get(i).getRobots().get(0).getCount());
            assertEquals((double) (i / 2), seen.get(i).getAttackModel().getLambda(), 0);
            assertEquals(3600, seen.get(i).getSimulation().getDuration());
            assertNotNull(seen.get(i).getRobotModels().get("turtlebot_pi"));
        }
        assertEquals(5, new ConfigLoader().load(p.getBaseConfig()).getRobots().get(0).getCount());
        assertEquals(2, runs.get(0).context().configuration().getRobots().get(0).getCount());
    }
    @Test public void realRunsExecuteDurationAndResetAllStaticWorldCollections() {
        var config = new ConfigLoader().load("configs/scenarios/base.yaml");
        config.getSimulation().setDuration(7); config.getAttackModel().setLambda(1.0);
        SimulationRunner runner = new SimulationRunner();
        var a = runner.runWithResult(config, false);
        var b = runner.runWithResult(config, false);
        assertEquals(7, a.getTotalAttacks()); assertEquals(a.getTotalAttacks(), b.getTotalAttacks());
        assertEquals(a.getInterceptedAttacks(), b.getInterceptedAttacks());
        assertEquals(5, World.getBots().size());
        assertEquals(2, World.getChargingStations().size());
    }
    @Test(expected = IllegalArgumentException.class) public void rejectsWrongSystemComposition() {
        var p = plan(); p.getDesigns().get(0).setSystem(ExperimentConfig.SystemType.HYBRID);
        new ExperimentRunner().prepare(p);
    }
    @Test(expected = IllegalArgumentException.class) public void rejectsAttackSweepThatChangesTeamSize() {
        var p = plan(); p.getDesigns().get(0).getPoints().get(1).getRobots().get(0).setCount(3);
        new ExperimentRunner().prepare(p);
    }
}
