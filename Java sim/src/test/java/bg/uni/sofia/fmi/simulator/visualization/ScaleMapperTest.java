package bg.uni.sofia.fmi.simulator.visualization;

import javafx.geometry.Point2D;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ScaleMapperTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void convertsPerimeterDistanceToCanvasDistance() {
        ScaleMapper linear = new ScaleMapper(100, 50, 950);
        assertEquals(90, linear.toCanvasDistance(10), EPSILON);

        ScaleMapper circular = ScaleMapper.circular(100, 500, 300, 20);
        assertEquals(4 * Math.PI, circular.toCanvasDistance(10), EPSILON);
    }

    @Test
    public void mapsCircularPositionsAcrossTheSeam() {
        ScaleMapper mapper = ScaleMapper.circular(100, 500, 300, 20);
        Point2D negativePosition = mapper.toCanvasPoint(-25, 0);
        Point2D equivalentPosition = mapper.toCanvasPoint(75, 0);
        Point2D positionBeyondEnd = mapper.toCanvasPoint(125, 0);
        Point2D equivalentWrappedPosition = mapper.toCanvasPoint(25, 0);

        assertEquals(equivalentPosition.getX(), negativePosition.getX(), EPSILON);
        assertEquals(equivalentPosition.getY(), negativePosition.getY(), EPSILON);
        assertEquals(equivalentWrappedPosition.getX(), positionBeyondEnd.getX(), EPSILON);
        assertEquals(equivalentWrappedPosition.getY(), positionBeyondEnd.getY(), EPSILON);
    }
}
