package bg.uni.sofia.fmi.simulator.util;

/**
 * Клас за измерване на времето, изминало между две събития. Полезен за:
 * - бенчмаркинг
 * - логване на продължителности
 */
public class TimeUtils {
    // Метод за получаване на текущото време в милисекунди
    public static long now() {
        return System.currentTimeMillis();
    }
    // Метод за изчисляване на продължителността от даден начален момент до сега
    public static long duration(long start) {
        return System.currentTimeMillis() - start;
    }
}
