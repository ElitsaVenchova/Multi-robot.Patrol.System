package bg.uni.sofia.fmi.simulator.util;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class ValidationUtilsTest {

    private static final double EPSILON = 1e-6;

    @Test
    public void testCheckProbability() {
        assertEquals(0.0, ValidationUtils.checkProbability(0.0, "p"), EPSILON);
        assertEquals(0.5, ValidationUtils.checkProbability(0.5, "p"), EPSILON);
        assertEquals(1.0, ValidationUtils.checkProbability(1.0, "p"), EPSILON);

        try {
            ValidationUtils.checkProbability(-0.01, "p");
            fail("Expected IllegalArgumentException for negative probability");
        } catch (IllegalArgumentException e) {
            // expected
        }

        try {
            ValidationUtils.checkProbability(1.01, "p");
            fail("Expected IllegalArgumentException for probability > 1.0");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testCheckPositive() {
        assertEquals(5.5, ValidationUtils.checkPositive(5.5, "speed"), EPSILON);
        assertEquals(10, ValidationUtils.checkPositive(10, "bots"));

        try {
            ValidationUtils.checkPositive(0.0, "speed");
            fail("Expected IllegalArgumentException for 0.0");
        } catch (IllegalArgumentException e) {
            // expected
        }

        try {
            ValidationUtils.checkPositive(0, "bots");
            fail("Expected IllegalArgumentException for 0");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testCheckNonNegative() {
        assertEquals(0.0, ValidationUtils.checkNonNegative(0.0, "time"), EPSILON);
        assertEquals(5.5, ValidationUtils.checkNonNegative(5.5, "time"), EPSILON);
        assertEquals(0, ValidationUtils.checkNonNegative(0, "index"));

        try {
            ValidationUtils.checkNonNegative(-0.01, "time");
            fail("Expected IllegalArgumentException for negative value");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testCheckRange() {
        assertEquals(5.0, ValidationUtils.checkRange(5.0, 0.0, 10.0, "val"), EPSILON);
        assertEquals(5, ValidationUtils.checkRange(5, 0, 10, "val"));

        try {
            ValidationUtils.checkRange(-1.0, 0.0, 10.0, "val");
            fail("Expected IllegalArgumentException for out of range value");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testCheckNotNull() {
        String test = "ok";
        assertEquals(test, ValidationUtils.checkNotNull(test, "str"));

        try {
            ValidationUtils.checkNotNull(null, "str");
            fail("Expected IllegalArgumentException for null");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }
}
