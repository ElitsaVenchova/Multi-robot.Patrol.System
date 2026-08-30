package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;
import javafx.geometry.Point2D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Polyline;
import javafx.scene.shape.StrokeLineCap;

/**
 * Visual representation of a robot's assigned patrol section.
 * Renders the section as a translucent line with boundary markers.
 */
public class PatrolSectionVisualNode extends Group {
    private static final int CIRCULAR_SEGMENTS = 48;//брой сегменти на рисуване при кръгов периметър
    private static final double SECTION_WIDTH = 24;//ширина на сегна

    public PatrolSectionVisualNode(PatrolSection patrolSection, ScaleMapper scaleMapper,
                                   double perimeterY, Color sectionColor) {
        Polyline sectionLine = new Polyline();
        sectionLine.setStroke(sectionColor.deriveColor(0, 1, 1, 0.55));
        sectionLine.setStrokeWidth(SECTION_WIDTH);
        sectionLine.setStrokeLineCap(StrokeLineCap.ROUND);
        sectionLine.setMouseTransparent(true);

        double start = patrolSection.getStartPosition().getX();
        double end = patrolSection.getEndPosition().getX();
        int segments = scaleMapper.isCircular() ? CIRCULAR_SEGMENTS : 1;
        Point2D startPoint = null;
        Point2D endPoint = null;
        for (int index = 0; index <= segments; index++) {
            double position = start + (end - start) * index / segments;
            Point2D point = scaleMapper.toCanvasPoint(position, perimeterY);
            sectionLine.getPoints().addAll(point.getX(), point.getY());//добавя точката от сегмента към линията
            if (index == 0) {
                startPoint = point;
            }
            if (index == segments) {
                endPoint = point;
            }
        }

        // Add boundary markers and finally add line and markers to the group
        Circle startMarker = createBoundaryMarker(startPoint, sectionColor);
        Circle endMarker = createBoundaryMarker(endPoint, sectionColor);
        getChildren().addAll(sectionLine, startMarker, endMarker);
        setMouseTransparent(true);
    }

    //Create boundary markers for the start and end of the section
    private Circle createBoundaryMarker(Point2D point, Color sectionColor) {
        Circle marker = new Circle(point.getX(), point.getY(), 4);
        marker.setFill(Color.WHITE);
        marker.setStroke(sectionColor);
        marker.setStrokeWidth(2);
        return marker;
    }
}
