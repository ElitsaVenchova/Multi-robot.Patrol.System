package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;
import javafx.geometry.Point2D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.transform.Rotate;

/** Visual tick mark for an attack at its position on the perimeter. */
public class AttackVisualNode extends Group {
    private static final double POINT_RADIUS = 6;
    private static final double MARKER_LENGTH = 14;

    private final Attack attack;
    private final Line marker;
    private final Circle point;

    public AttackVisualNode(Attack attack, ScaleMapper scaleMapper, double perimeterY) {
        this.attack = attack;
        this.marker = new Line(
                0, -(POINT_RADIUS + MARKER_LENGTH),
                0, -(POINT_RADIUS + 1)
        );
        this.point = new Circle(0, 0, POINT_RADIUS);

        getChildren().addAll(marker, point);
        Point2D canvasPosition = scaleMapper.toCanvasPoint(attack.getPosition().getX(), perimeterY);
        setLayoutX(canvasPosition.getX());
        setLayoutY(canvasPosition.getY());
        getTransforms().add(new Rotate(
                scaleMapper.toMarkerRotationDegrees(attack.getPosition().getX()), 0, 0
        ));
        updateFrame();
    }

    /** Refresh the mark's colour after the attack status changes. */
    public void updateFrame() {
        Color color = switch (attack.getStatus()) {
            case ACTIVE -> Color.CRIMSON;
            case INTERCEPTED -> Color.FORESTGREEN;
            case MISSED -> Color.DARKGRAY;
        };

        marker.setStroke(color);
        marker.setStrokeWidth(2);
        point.setFill(color);
        point.setStroke(Color.WHITE);
        point.setStrokeWidth(2);

        if (attack.getStatus() == AttackStatus.MISSED) {
            marker.getStrokeDashArray().setAll(3.0, 3.0);
            point.setOpacity(0.65);
        } else {
            marker.getStrokeDashArray().clear();
            point.setOpacity(1.0);
        }
    }
}
