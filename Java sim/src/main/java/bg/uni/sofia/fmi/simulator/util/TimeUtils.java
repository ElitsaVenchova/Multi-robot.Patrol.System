package bg.uni.sofia.fmi.simulator.util;

import java.util.Locale;

/**
 * Клас за измерване на времето, изминало между две събития. Полезен за:
 * - бенчмаркинг (в милисекунди и наносекунди)
 * - логване и форматиране на продължителности
 */
public class TimeUtils {

    // Метод за получаване на текущото време в милисекунди
    public static long now() {
        return System.currentTimeMillis();
    }

    // Метод за получаване на текущото време с висока резолюция в наносекунди
    public static long nowNanos() {
        return System.nanoTime();
    }

    // Метод за изчисляване на продължителността от даден начален момент (в ms) до сега
    public static long duration(long startMillis) {
        return System.currentTimeMillis() - startMillis;
    }

    // Метод за изчисляване на продължителността в наносекунди от даден начален момент (в ns) до сега
    public static long durationNanos(long startNanos) {
        return System.nanoTime() - startNanos;
    }

    // Превръщане на милисекунди в четим формат (напр. "1m 23s", "450ms", "2.35s")
    public static String formatDurationMs(long millis) {
        if (millis < 0) {
            return "-" + formatDurationMs(-millis);
        }
        if (millis < 1000) {
            return millis + "ms";
        }
        long seconds = millis / 1000;
        long msRemainder = millis % 1000;
        if (seconds < 60) {
            return String.format(Locale.US, "%.2fs", millis / 1000.0);
        }
        long minutes = seconds / 60;
        long secRemainder = seconds % 60;
        if (minutes < 60) {
            return String.format(Locale.US, "%dm %02ds", minutes, secRemainder);
        }
        long hours = minutes / 60;
        long minRemainder = minutes % 60;
        return String.format(Locale.US, "%dh %02dm %02ds", hours, minRemainder, secRemainder);
    }

    // Превръщане на наносекунди в четим формат (напр. "120ns", "45.20µs", "3.15ms", "1.20s")
    public static String formatDurationNanos(long nanos) {
        if (nanos < 0) {
            return "-" + formatDurationNanos(-nanos);
        }
        if (nanos < 1000) {
            return nanos + "ns";
        }
        if (nanos < 1_000_000) {
            return String.format(Locale.US, "%.2fµs", nanos / 1_000.0);
        }
        if (nanos < 1_000_000_000L) {
            return String.format(Locale.US, "%.2fms", nanos / 1_000_000.0);
        }
        return formatDurationMs(nanos / 1_000_000L);
    }
}
