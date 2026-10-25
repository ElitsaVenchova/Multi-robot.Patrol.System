package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.Battery;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.CommunicationModule;
import bg.uni.sofia.fmi.simulator.domain.Lidar;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Polyline;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class BotRangeVisualNodeTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void rendersLidarAsConfiguredCircularRadius() {
        BotRangeVisualNode node = createNode();

        Group lidarGlyph = (Group) node.getChildren().get(0);
        Circle lidarCircle = (Circle) lidarGlyph.getChildren().get(0);

        assertEquals(500, lidarCircle.getCenterX(), EPSILON);
        assertEquals(300, lidarCircle.getCenterY(), EPSILON);
        assertEquals(50, lidarCircle.getRadius(), EPSILON);
    }

    @Test
    public void rendersCommunicationAsTwoBracketsAtConfiguredRange() {
        BotRangeVisualNode node = createNode();

        Group communicationGlyph = (Group) node.getChildren().get(1);
        assertEquals(2, communicationGlyph.getChildren().size());
        Polyline left = (Polyline) communicationGlyph.getChildren().get(0);
        Polyline right = (Polyline) communicationGlyph.getChildren().get(1);

        assertEquals(-50, left.getPoints().get(0), EPSILON);
        assertEquals(86.6025403784, left.getPoints().get(1), EPSILON);
        assertEquals(50, right.getPoints().get(0), EPSILON);
        assertEquals(-86.6025403784, right.getPoints().get(1), EPSILON);
        assertEquals(34, left.getPoints().size());
        assertEquals(34, right.getPoints().size());
    }

    @Test
    public void togglesEachRangeIndependentlyAndKeepsStateAcrossFrames() {
        BotRangeVisualNode node = createNode();
        Group lidarGlyph = (Group) node.getChildren().get(0);
        Group communicationGlyph = (Group) node.getChildren().get(1);

        node.setCommunicationVisible(false);
        node.updateFrame();
        assertFalse(communicationGlyph.isVisible());
        assertTrue(lidarGlyph.isVisible());

        node.setLidarVisible(false);
        node.updateFrame();
        assertFalse(communicationGlyph.isVisible());
        assertFalse(lidarGlyph.isVisible());

        node.setCommunicationVisible(true);
        node.updateFrame();
        assertTrue(communicationGlyph.isVisible());
        assertFalse(lidarGlyph.isVisible());
    }

    private BotRangeVisualNode createNode() {
        Bot bot = new Bot(1, new Position(50), new Battery(100, 1), new Lidar(5, 0.1),
                new CommunicationModule(10, 1, 0, 1), 1, RobotType.GROUND, "test", 0, 0, 0, null);
        ScaleMapper mapper = new ScaleMapper(100, 0, 1000);
        return new BotRangeVisualNode(bot, mapper, 300);
    }
}
