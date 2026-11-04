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

    private final ScaleMapper scaleMapper;
    private final double perimeterY;
    private final Polyline sectionLine;
    private final Circle startMarker;
    private final Circle endMarker;
    private double renderedStart = Double.NaN;
    private double renderedEnd = Double.NaN;

    public PatrolSectionVisualNode(PatrolSection patrolSection, ScaleMapper scaleMapper,
                                   double perimeterY, Color sectionColor) {
        this.scaleMapper = scaleMapper;
        this.perimeterY = perimeterY;
        sectionLine = new Polyline();
        sectionLine.setStroke(sectionColor.deriveColor(0, 1, 1, 0.55));
        sectionLine.setStrokeWidth(SECTION_WIDTH);
        sectionLine.setStrokeLineCap(StrokeLineCap.ROUND);
        sectionLine.setMouseTransparent(true);

        startMarker = createBoundaryMarker(sectionColor);
        endMarker = createBoundaryMarker(sectionColor);
        getChildren().addAll(sectionLine, startMarker, endMarker);
        setMouseTransparent(true);
        updateSection(patrolSection);
    }

    /** Refresh the existing shapes only when the section's perimeter endpoints change. */
    public void updateSection(PatrolSection patrolSection) {
        double start = patrolSection.startPosition().getX();
        double end = patrolSection.endPosition().getX();
        if (Double.compare(start, renderedStart) == 0 && Double.compare(end, renderedEnd) == 0) {
            return;
        }

        sectionLine.getPoints().clear();
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

        startMarker.setCenterX(startPoint.getX());
        startMarker.setCenterY(startPoint.getY());
        endMarker.setCenterX(endPoint.getX());
        endMarker.setCenterY(endPoint.getY());
        renderedStart = start;
        renderedEnd = end;
    }

    //Create boundary markers for the start and end of the section
    private Circle createBoundaryMarker(Color sectionColor) {
        Circle marker = new Circle(4);
        marker.setFill(Color.WHITE);
        marker.setStroke(sectionColor);
        marker.setStrokeWidth(2);
        return marker;
    }
}
