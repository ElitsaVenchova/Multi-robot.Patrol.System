package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;

/** Visual tick mark for an attack at its position on the perimeter. */
public class AttackVisualNode extends Group {
    private static final double MARKER_HEIGHT = 20;

    private final Attack attack;
    private final Line marker;
    private final Circle point;

    public AttackVisualNode(Attack attack, ScaleMapper scaleMapper, double perimeterY) {
        this.attack = attack;
        this.marker = new Line(0, -MARKER_HEIGHT, 0, -3);
        this.point = new Circle(0, -MARKER_HEIGHT, 3);

        getChildren().addAll(marker, point);
        setLayoutX(scaleMapper.toCanvasX(attack.getPosition().getX()));
        setLayoutY(perimeterY);
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

        if (attack.getStatus() == AttackStatus.MISSED) {
            marker.getStrokeDashArray().setAll(3.0, 3.0);
            point.setOpacity(0.65);
        } else {
            marker.getStrokeDashArray().clear();
            point.setOpacity(1.0);
        }
    }
}
