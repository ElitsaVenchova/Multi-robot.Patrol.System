package bg.uni.sofia.fmi.simulator.util;

import java.util.Random;

// За генериране на произволни числа с възможност за задаване на сийд (seed) за възпроизводимост
public class RandomProvider {
    private static Random random = new Random();

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

    // Метод за генериране на произволно реално число в интервала [min, max)
    public static double nextDouble(double min, double max) {
        if (min >= max) {
            throw new IllegalArgumentException("min must be less than max");
        }
        return min + (max - min) * random.nextDouble();
    }

    // Метод за генериране на произволно цяло число от 0 до bound-1
    public static int nextInt(int bound) {
        if (bound <= 0) {
            throw new IllegalArgumentException("bound must be positive");
        }
        return random.nextInt(bound);
    }

    // Метод за генериране на произволно цяло число в интервала [min, max] (включително)
    public static int nextInt(int min, int max) {
        if (min > max) {
            throw new IllegalArgumentException("min must be less than or equal to max");
        }
        return min + random.nextInt((max - min) + 1);
    }

    // Генериране на експоненциално разпределена случайна величина (за интервали между събития в Poisson процес)
    public static double nextExponential(double lambda) {
        if (lambda <= 0) {
            throw new IllegalArgumentException("lambda must be positive");
        }
        // u е в (0, 1] за избягване на ln(0)
        double u = random.nextDouble();
        while (u == 0.0) {
            u = random.nextDouble();
        }
        return -Math.log(u) / lambda;
    }

    // Бернулиев опит с вероятност за успех probability
    public static boolean nextBoolean(double probability) {
        if (probability < 0.0 || probability > 1.0) {
            throw new IllegalArgumentException("probability must be between 0.0 and 1.0");
        }
        return random.nextDouble() < probability;
    }

    // Генериране на Гаусово (нормално) разпределена случайна величина
    public static double nextGaussian(double mean, double stdDev) {
        if (stdDev < 0) {
            throw new IllegalArgumentException("stdDev cannot be negative");
        }
        return mean + random.nextGaussian() * stdDev;
    }
}
