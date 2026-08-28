package bg.uni.sofia.fmi.simulator.util;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class TimeUtilsTest {

    @Test
    public void testDuration() throws InterruptedException {
        long startMs = TimeUtils.now();
        long startNs = TimeUtils.nowNanos();

        Thread.sleep(20);

        long durMs = TimeUtils.duration(startMs);
        long durNs = TimeUtils.durationNanos(startNs);

        assertTrue(durMs >= 15);
        assertTrue(durNs >= 15_000_000L);
    }

    @Test
    public void testFormatDurationMs() {
        assertEquals("450ms", TimeUtils.formatDurationMs(450));
        assertEquals("1.50s", TimeUtils.formatDurationMs(1500));
        assertEquals("2m 05s", TimeUtils.formatDurationMs(125000));
        assertEquals("1h 05m 10s", TimeUtils.formatDurationMs(3910000));
    }

    @Test
    public void testFormatDurationNanos() {
        assertEquals("500ns", TimeUtils.formatDurationNanos(500));
        assertEquals("45.20µs", TimeUtils.formatDurationNanos(45200));
        assertEquals("3.50ms", TimeUtils.formatDurationNanos(3500000));
        assertEquals("2.00s", TimeUtils.formatDurationNanos(2000000000L));
    }
}
