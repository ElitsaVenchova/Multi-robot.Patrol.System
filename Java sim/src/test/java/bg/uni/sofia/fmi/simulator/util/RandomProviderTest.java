package bg.uni.sofia.fmi.simulator.util;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class RandomProviderTest {

    @Before
    public void setUp() {
        RandomProvider.setSeed(12345L);
    }

    @Test
    public void testNextDoubleRange() {
        for (int i = 0; i < 1000; i++) {
            double val = RandomProvider.nextDouble(5.0, 15.0);
            assertTrue(val >= 5.0 && val < 15.0);
        }

        try {
            RandomProvider.nextDouble(10.0, 5.0);
            fail("Expected IllegalArgumentException when min >= max");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testNextIntRange() {
        for (int i = 0; i < 1000; i++) {
            int val = RandomProvider.nextInt(10, 20);
            assertTrue(val >= 10 && val <= 20);
        }

        try {
            RandomProvider.nextInt(20, 10);
            fail("Expected IllegalArgumentException when min > max");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testNextExponential() {
        double lambda = 0.5;
        double sum = 0.0;
        int n = 10000;
        for (int i = 0; i < n; i++) {
            double val = RandomProvider.nextExponential(lambda);
            assertTrue(val >= 0.0);
            sum += val;
        }
        double sampleMean = sum / n;
        double expectedMean = 1.0 / lambda; // 2.0
        assertEquals(expectedMean, sampleMean, 0.1);

        try {
            RandomProvider.nextExponential(0.0);
            fail("Expected IllegalArgumentException when lambda <= 0");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testNextBoolean() {
        double p = 0.3;
        int countTrue = 0;
        int n = 10000;
        for (int i = 0; i < n; i++) {
            if (RandomProvider.nextBoolean(p)) {
                countTrue++;
            }
        }
        double observedP = (double) countTrue / n;
        assertEquals(p, observedP, 0.02);

        try {
            RandomProvider.nextBoolean(-0.1);
            fail("Expected IllegalArgumentException when p < 0");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testNextGaussian() {
        double mean = 10.0;
        double std = 2.0;
        double sum = 0.0;
        int n = 10000;
        for (int i = 0; i < n; i++) {
            sum += RandomProvider.nextGaussian(mean, std);
        }
        double observedMean = sum / n;
        assertEquals(mean, observedMean, 0.1);

        try {
            RandomProvider.nextGaussian(mean, -1.0);
            fail("Expected IllegalArgumentException when std < 0");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }
}
