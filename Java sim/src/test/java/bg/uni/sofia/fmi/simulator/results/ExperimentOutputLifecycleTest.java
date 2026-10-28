package bg.uni.sofia.fmi.simulator.results;

import bg.uni.sofia.fmi.simulator.config.*;
import bg.uni.sofia.fmi.simulator.experiments.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import static org.junit.Assert.*;

public class ExperimentOutputLifecycleTest {
    @Rule public TemporaryFolder folder = new TemporaryFolder();

    @Test public void failedSecondOpenClosesFirstWriterAndPreservesTheOpeningFailure() throws Exception {
        Path directory = folder.getRoot().toPath().resolve("batch");
        IOException opening = new IOException("open robots"), closing = new IOException("close runs");
        TrackingWriter runs = new TrackingWriter(closing);
        AtomicInteger opens = new AtomicInteger();
        try {
            new ResultExporter.ExperimentOutput(directory, Map.of(), path -> {
                if (opens.getAndIncrement() == 0) return runs;
                throw opening;
            });
            fail("Second open must fail");
        } catch (IOException actual) {
            assertSame(opening, actual);
            assertArrayEquals(new Throwable[]{closing}, actual.getSuppressed());
        }
        assertTrue(runs.closed);
        assertEquals("FAILED", Files.readString(directory.resolve("status.txt")).strip());
    }

    @Test public void bothWritersCloseAndACompletedSummaryCannotMaskCloseFailures() throws Exception {
        Path directory = folder.getRoot().toPath().resolve("batch");
        IOException runFailure = new IOException("close runs"), robotFailure = new IOException("close robots");
        TrackingWriter runs = new TrackingWriter(runFailure), robots = new TrackingWriter(robotFailure);
        var output = new ResultExporter.ExperimentOutput(directory, Map.of(),
                path -> path.getFileName().toString().equals("experiment_runs.csv") ? runs : robots);
        output.finish(List.of());
        try { output.close(); fail("Close must fail"); }
        catch (IOException actual) {
            assertSame(robotFailure, actual);
            assertArrayEquals(new Throwable[]{runFailure}, actual.getSuppressed());
        }
        assertTrue(runs.closed); assertTrue(robots.closed);
        assertEquals("FAILED", Files.readString(directory.resolve("status.txt")).strip());
        output.close();
        assertEquals("FAILED", Files.readString(directory.resolve("status.txt")).strip());
    }

    @Test public void unfinishedOutputClosesWritersAndRecordsFailure() throws Exception {
        Path directory = folder.getRoot().toPath().resolve("batch");
        TrackingWriter runs = new TrackingWriter(null), robots = new TrackingWriter(null);
        try (var ignored = new ResultExporter.ExperimentOutput(directory, Map.of(),
                path -> path.getFileName().toString().equals("experiment_runs.csv") ? runs : robots)) { }
        assertTrue(runs.closed); assertTrue(robots.closed);
        assertEquals("FAILED", Files.readString(directory.resolve("status.txt")).strip());
    }

    @Test public void statusWriteFailureStillClosesBothWriters() throws Exception {
        Path directory = folder.getRoot().toPath().resolve("batch");
        TrackingWriter runs = new TrackingWriter(null), robots = new TrackingWriter(null);
        var output = new ResultExporter.ExperimentOutput(directory, Map.of(),
                path -> path.getFileName().toString().equals("experiment_runs.csv") ? runs : robots);
        output.finish(List.of());
        Path status = directory.resolve("status.txt");
        Files.delete(status); Files.createDirectory(status); // Deterministic failure without OS permission assumptions.
        try { output.close(); fail("Writing status to a directory must fail"); }
        catch (IOException expected) { assertTrue(Files.isDirectory(status)); }
        assertTrue(runs.closed); assertTrue(robots.closed);
    }

    @Test public void statusCleanupFailureDoesNotReplaceTheOpeningFailure() throws Exception {
        Path directory = folder.getRoot().toPath().resolve("batch");
        IOException opening = new IOException("open robots");
        TrackingWriter runs = new TrackingWriter(null);
        AtomicInteger opens = new AtomicInteger();
        try {
            new ResultExporter.ExperimentOutput(directory, Map.of(), path -> {
                if (opens.getAndIncrement() == 0) return runs;
                Path status = directory.resolve("status.txt");
                Files.delete(status); Files.createDirectory(status);
                throw opening;
            });
            fail("Second open must fail");
        } catch (IOException actual) {
            assertSame(opening, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertTrue(actual.getSuppressed()[0] instanceof IOException);
        }
        assertTrue(runs.closed);
    }

    @Test public void existingBatchIsRefusedWithoutTouchingStatusOrOpeningWriters() throws Exception {
        Path directory = folder.newFolder("existing").toPath();
        Files.writeString(directory.resolve("status.txt"), "COMPLETE\n");
        Files.writeString(directory.resolve("experiment_runs.csv"), "sentinel\n");
        AtomicInteger opens = new AtomicInteger();
        try {
            new ResultExporter.ExperimentOutput(directory, Map.of(), path -> {
                opens.incrementAndGet(); return new TrackingWriter(null);
            });
            fail("Must refuse overwrite");
        } catch (FileAlreadyExistsException expected) { }
        assertEquals(0, opens.get());
        assertEquals("COMPLETE\n", Files.readString(directory.resolve("status.txt")));
        assertEquals("sentinel\n", Files.readString(directory.resolve("experiment_runs.csv")));
    }

    @Test public void runBatchRetainsSimulationFailureAndRecordsFailedStatus() throws Exception {
        IllegalStateException simulationFailure = new IllegalStateException("fixture simulation failed");
        var runner = new ExperimentRunner(config -> { throw simulationFailure; });
        var plan = plan();
        try { runner.runBatch(plan, folder.getRoot().toPath()); fail("Simulation must fail"); }
        catch (IllegalStateException actual) { assertSame(simulationFailure, actual); }
        assertEquals("FAILED", Files.readString(folder.getRoot().toPath().resolve("failure_fixture/status.txt")).strip());
        assertFalse(Files.exists(folder.getRoot().toPath().resolve("failure_fixture/experiment_summary.csv")));
    }

    private ExperimentConfig plan() {
        var plan = new ExperimentConfig(); plan.setExperimentId("failure_fixture");
        plan.setBaseConfig("configs/scenarios/base.yaml"); plan.setRepetitions(1);
        var design = new ExperimentConfig.Design(); design.setId("ground");
        design.setType(ExperimentConfig.Type.ATTACK_DENSITY); design.setSystem(ExperimentConfig.SystemType.TURTLEBOT);
        var point = new ExperimentConfig.Point(); point.setId("p0"); point.setResourceScenario("fixture");
        design.setPoints(List.of(point)); plan.setDesigns(List.of(design));
        return plan;
    }

    private static class TrackingWriter extends BufferedWriter {
        private IOException closeFailure;
        private boolean closed;
        TrackingWriter(IOException closeFailure) { super(new StringWriter()); this.closeFailure = closeFailure; }
        @Override public void close() throws IOException {
            closed = true;
            super.close();
            IOException failure = closeFailure; closeFailure = null;
            if (failure != null) throw failure;
        }
    }
}
