package bg.uni.sofia.fmi.simulator.util;

import java.util.Random;

// За генериране на произволни числа с възможност за задаване на сийд (seed) за възпроизводимост
public class RandomProvider {
    private static Random random = RandomProvider.getRandom();

    public static void setSeed(long seed) {
        random = new Random(seed);
    }

    public static Random getRandom() {
        return random;
    }
    // Метод за генериране на произволно число от 0.0 до 1.0
    public static double nextDouble() {
        return random.nextDouble();
    }
    // Метод за генериране на произволно цяло число от 0 до bound-1
    public static int nextInt(int bound) {
        return random.nextInt(bound);
    }
}
