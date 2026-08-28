package bg.uni.sofia.fmi.simulator.util;

import bg.uni.sofia.fmi.simulator.domain.Position;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class GeometryUtilsTest {

    private static final double EPSILON = 1e-6;

    @Test
    public void test2DDistance() {
        assertEquals(5.0, GeometryUtils.distance(0, 0, 3, 4), EPSILON);
        assertEquals(0.0, GeometryUtils.distance(2.5, 2.5, 2.5, 2.5), EPSILON);
    }

    @Test
    public void test3DDistance() {
        assertEquals(Math.sqrt(3.0), GeometryUtils.distance(0, 0, 0, 1, 1, 1), EPSILON);
    }

    @Test
    public void testEuclideanDistancePosition() {
        Position p1 = new Position(1.0, 2.0, 3.0);
        Position p2 = new Position(4.0, 6.0, 3.0);
        assertEquals(5.0, GeometryUtils.euclideanDistance(p1, p2), EPSILON);

        try {
            GeometryUtils.euclideanDistance(null, p2);
            fail("Expected IllegalArgumentException when position is null");
        } catch (IllegalArgumentException e) {
            // expected
        }
    }

    @Test
    public void testNormalizePosition() {
        double perimeter = 100.0;
        assertEquals(25.0, GeometryUtils.normalizePosition(25.0, perimeter), EPSILON);
        assertEquals(25.0, GeometryUtils.normalizePosition(125.0, perimeter), EPSILON);
        assertEquals(75.0, GeometryUtils.normalizePosition(-25.0, perimeter), EPSILON);
        assertEquals(0.0, GeometryUtils.normalizePosition(100.0, perimeter), EPSILON);
        assertEquals(0.0, GeometryUtils.normalizePosition(0.0, perimeter), EPSILON);
    }

    @Test
    public void testRingDistance() {
        double perimeter = 100.0;
        // Direct arc is 20
        assertEquals(20.0, GeometryUtils.ringDistance(10.0, 30.0, perimeter), EPSILON);
        // Shortest arc wrapping around boundary (95 to 5 is 10)
        assertEquals(10.0, GeometryUtils.ringDistance(95.0, 5.0, perimeter), EPSILON);
        assertEquals(10.0, GeometryUtils.ringDistance(5.0, 95.0, perimeter), EPSILON);
        // Exact half ring
        assertEquals(50.0, GeometryUtils.ringDistance(0.0, 50.0, perimeter), EPSILON);
        // Same position
        assertEquals(0.0, GeometryUtils.ringDistance(30.0, 30.0, perimeter), EPSILON);
    }

    @Test
    public void testShortestRingDirection() {
        double perimeter = 100.0;
        // 10 to 30 -> shortest is forward (+1)
        assertEquals(1, GeometryUtils.shortestRingDirection(10.0, 30.0, perimeter));
        // 30 to 10 -> shortest is backward (-1)
        assertEquals(-1, GeometryUtils.shortestRingDirection(30.0, 10.0, perimeter));
        // 95 to 5 -> shortest is wrapping forward (+1)
        assertEquals(1, GeometryUtils.shortestRingDirection(95.0, 5.0, perimeter));
        // 5 to 95 -> shortest is wrapping backward (-1)
        assertEquals(-1, GeometryUtils.shortestRingDirection(5.0, 95.0, perimeter));
        // 20 to 20 -> same position (0)
        assertEquals(0, GeometryUtils.shortestRingDirection(20.0, 20.0, perimeter));
    }
}
