package bg.uni.sofia.fmi.simulator.results;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;

public class MetricsAggregatorTest {

    private static final double EPSILON = 1e-6;

    @Test
    public void testAggregateEmpty() {
        MetricsAggregator aggregator = new MetricsAggregator();
        AggregatedMetrics result = aggregator.aggregate(Collections.emptyList());
        assertEquals(0, result.getRuns());
        assertEquals(0.0, result.getMeanSuccessRate(), EPSILON);
        assertEquals(0.0, result.getMeanDetectionTime(), EPSILON);
        assertEquals(0.0, result.getStdSuccessRate(), EPSILON);
        assertEquals(0.0, result.getStdDetectionTime(), EPSILON);
    }

    @Test
    public void testAggregateMetrics() {
        MetricsAggregator aggregator = new MetricsAggregator();
        List<SimulationMetrics> metricsList = new ArrayList<>();

        SimulationMetrics m1 = new SimulationMetrics();
        m1.setSuccessRate(1.0);
        m1.setAverageDetectionTime(10.0);
        metricsList.add(m1);

        SimulationMetrics m2 = new SimulationMetrics();
        m2.setSuccessRate(0.5);
        m2.setAverageDetectionTime(20.0);
        metricsList.add(m2);

        AggregatedMetrics result = aggregator.aggregate(metricsList);
        assertEquals(2, result.getRuns());
        assertEquals(0.75, result.getMeanSuccessRate(), EPSILON);
        assertEquals(15.0, result.getMeanDetectionTime(), EPSILON);

        // population std: sqrt(((1-0.75)^2 + (0.5-0.75)^2)/2) = sqrt((0.0625 + 0.0625)/2) = 0.25
        assertEquals(0.25, result.getStdSuccessRate(), EPSILON);
        // detection std: sqrt(((10-15)^2 + (20-15)^2)/2) = sqrt((25+25)/2) = 5.0
        assertEquals(5.0, result.getStdDetectionTime(), EPSILON);
    }
}
