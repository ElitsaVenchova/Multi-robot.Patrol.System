package bg.uni.sofia.fmi.simulator.util;

/**
 * Utility for measuring time and durations in the simulation.
 * Useful for:
 * - benchmarking
 * - logging durations
 */
public class TimeUtils {

    public static long now() {
        return System.currentTimeMillis();
    }

    public static long duration(long start) {
        return System.currentTimeMillis() - start;
    }
}
