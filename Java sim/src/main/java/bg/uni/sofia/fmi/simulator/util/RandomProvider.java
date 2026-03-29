package bg.uni.sofia.fmi.simulator.util;

import java.util.Random;

public class RandomProvider {

    private static Random random = RandomProvider.getRandom();

    public static void setSeed(long seed) {
        random = new Random(seed);
    }

    public static Random getRandom() {
        return random;
    }

    public static double nextDouble() {
        return random.nextDouble();
    }

    public static int nextInt(int bound) {
        return random.nextInt(bound);
    }
}
