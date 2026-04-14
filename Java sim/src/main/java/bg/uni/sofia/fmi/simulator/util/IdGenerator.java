package bg.uni.sofia.fmi.simulator.util;

import java.util.concurrent.atomic.AtomicLong;

// Генератор на уникални идентификатори за ботовете
public class IdGenerator {
    private static final AtomicLong counter = new AtomicLong(0);
    // Метод за получаване на следващия уникален идентификатор
    public static long nextId() {
        return counter.incrementAndGet();
    }
}
