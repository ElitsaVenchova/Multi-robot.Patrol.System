package bg.uni.sofia.fmi.simulator.util;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class MathUtilsTest {

    private static final double EPSILON = 1e-6;

    @Test
    public void testClamp() {
        assertEquals(5.0, MathUtils.clamp(5.0, 0.0, 10.0), EPSILON);
        assertEquals(0.0, MathUtils.clamp(-2.0, 0.0, 10.0), EPSILON);
        assertEquals(10.0, MathUtils.clamp(15.0, 0.0, 10.0), EPSILON);
        assertEquals(0.0, MathUtils.clamp(0.0, 0.0, 10.0), EPSILON);
        assertEquals(10.0, MathUtils.clamp(10.0, 0.0, 10.0), EPSILON);

        try {
            MathUtils.clamp(5.0, 10.0, 0.0);
            fail("Expected IllegalArgumentException when min > max");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testMean() {
        assertEquals(0.0, MathUtils.mean(null), EPSILON);
        assertEquals(0.0, MathUtils.mean(Collections.emptyList()), EPSILON);

        List<Double> values = Arrays.asList(2.0, 4.0, 6.0, 8.0);
        assertEquals(5.0, MathUtils.mean(values), EPSILON);

        List<Double> single = Collections.singletonList(42.0);
        assertEquals(42.0, MathUtils.mean(single), EPSILON);
    }

    @Test
    public void testPopulationStd() {
        assertEquals(0.0, MathUtils.std(null, 0.0), EPSILON);
        assertEquals(0.0, MathUtils.std(Collections.emptyList(), 0.0), EPSILON);

        List<Double> values = Arrays.asList(2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0);
        double mean = MathUtils.mean(values); // 5.0
        // Variance = (9 + 1 + 1 + 1 + 0 + 0 + 4 + 16) / 8 = 32 / 8 = 4.0 -> std = 2.0
        assertEquals(2.0, MathUtils.std(values, mean), EPSILON);
    }

    @Test
    public void testSampleStd() {
        assertEquals(0.0, MathUtils.sampleStd(null, 0.0), EPSILON);
        assertEquals(0.0, MathUtils.sampleStd(Collections.emptyList(), 0.0), EPSILON);
        assertEquals(0.0, MathUtils.sampleStd(Collections.singletonList(5.0), 5.0), EPSILON);

        List<Double> values = Arrays.asList(2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0);
        double mean = MathUtils.mean(values); // 5.0
        // Sample Variance = 32 / 7 = 4.57142857 -> sampleStd = 2.138089935
        assertEquals(Math.sqrt(32.0 / 7.0), MathUtils.sampleStd(values, mean), EPSILON);
    }

    @Test
    public void testStandardError() {
        assertEquals(0.0, MathUtils.standardError(null, 2.0), EPSILON);
        assertEquals(0.0, MathUtils.standardError(Collections.emptyList(), 2.0), EPSILON);

        List<Double> values = Arrays.asList(1.0, 2.0, 3.0, 4.0);
        // SEM = 2.0 / sqrt(4) = 1.0
        assertEquals(1.0, MathUtils.standardError(values, 2.0), EPSILON);
    }

    @Test
    public void testMedian() {
        assertEquals(0.0, MathUtils.median(null), EPSILON);
        assertEquals(0.0, MathUtils.median(Collections.emptyList()), EPSILON);

        List<Double> odd = Arrays.asList(9.0, 1.0, 5.0);
        assertEquals(5.0, MathUtils.median(odd), EPSILON);

        List<Double> even = Arrays.asList(9.0, 1.0, 5.0, 7.0);
        assertEquals(6.0, MathUtils.median(even), EPSILON);
    }

    @Test
    public void testMinAndMax() {
        assertEquals(0.0, MathUtils.min(null), EPSILON);
        assertEquals(0.0, MathUtils.min(Collections.emptyList()), EPSILON);
        assertEquals(0.0, MathUtils.max(null), EPSILON);
        assertEquals(0.0, MathUtils.max(Collections.emptyList()), EPSILON);

        List<Double> values = Arrays.asList(7.5, -3.2, 12.8, 0.0, -8.1);
        assertEquals(-8.1, MathUtils.min(values), EPSILON);
        assertEquals(12.8, MathUtils.max(values), EPSILON);
    }
}
