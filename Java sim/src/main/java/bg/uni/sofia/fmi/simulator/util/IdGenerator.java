package bg.uni.sofia.fmi.simulator.util;

import java.util.concurrent.atomic.AtomicLong;

// Генератор на уникални идентификатори за ботовете и обектите
public class IdGenerator {
    private static final AtomicLong counter = new AtomicLong(0);

    // Метод за получаване на следващия уникален идентификатор
    public static long nextId() {
        return counter.incrementAndGet();
    }

    // Рестартиране на брояча (полезно при нови симулационни експерименти или тестове)
    public static void reset() {
        counter.set(0);
    }

    // Взимане на текущата стойност на брояча без инкрементиране
    public static long get() {
        return counter.get();
    }
}
