package bg.uni.sofia.fmi.simulator.util;

import java.util.concurrent.atomic.AtomicLong;

/**
 * Useful if you later want:
 * -tracking robots
 * -tracking attacks
 * -debugging
 */
public class IdGenerator {

    private static final AtomicLong counter = new AtomicLong(0);

    public static long nextId() {
        return counter.incrementAndGet();
    }
}
