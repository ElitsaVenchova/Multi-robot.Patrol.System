package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import javafx.geometry.Point2D;
import javafx.scene.Group;
import javafx.scene.control.Tooltip;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.util.Duration;

/**
 * Visual representation of a charging station on the perimeter.
 * - Drawn as a rectangle on the perimeter
 * - Color indicates station status:
 *   - Green (AVAILABLE)
 *   - Yellow (OCCUPIED)
 *   - Red (FAIL)
 * - Queue size displayed as a number
 * - Tooltip shows detailed metadata on hover
 */
public class ChargingStationVisualNode extends Group {
    private static final double STATION_WIDTH = 20;//размери на станцията
    private static final double STATION_HEIGHT = 10;//размери на станцията
    private static final double LABEL_FONT_SIZE = 9;//размер на label-а (брой свободни слотове)

    private final ChargingStation station;
    private final ScaleMapper scaleMapper;
    private final double perimeterY;
    private Rectangle stationRect;
    private Text queueLabel;

    public ChargingStationVisualNode(ChargingStation station, ScaleMapper scaleMapper, double perimeterY) {
        this.station = station;
        this.scaleMapper = scaleMapper;
        this.perimeterY = perimeterY;

        // Create station rectangle
        createStationShape();

        // Add queue size label
        addQueueLabel();

        // Add tooltip with metadata
        addMetadataTooltip();

        // Position on canvas
        updatePosition();
    }

    //Създаване на правоъгълник за представяне на станцията
    private void createStationShape() {
        stationRect = new Rectangle(STATION_WIDTH, STATION_HEIGHT);
        stationRect.setStroke(Color.BLACK);
        stationRect.setStrokeWidth(1);
        updateStationColor();

        this.getChildren().add(stationRect);
    }

    //Добавяне на надпис с размера на опашката за станцията
    private void addQueueLabel() {
        queueLabel = new Text(String.valueOf(station.getQueueSize()));
        queueLabel.setFont(new Font(LABEL_FONT_SIZE));
        queueLabel.setFill(Color.BLACK);
        
        // Position label centered on station
        queueLabel.setY(STATION_HEIGHT / 2.0 + 3);
        queueLabel.setX(-queueLabel.prefWidth(0) / 2.0);

        this.getChildren().add(queueLabel);
    }

    /// Add a tooltip showing detailed station metadata.
    private void addMetadataTooltip() {
        String tooltipText = String.format(
                "Charging Station #%d\nStatus: %s\nQueue: %d/%d\nLocation: %.1f\nPower: %.1f",
                station.getId(), station.getStatus().toString(), station.getQueueSize(),
                station.getTotalSlots(), station.getLocation().getX(), station.getPower());

        Tooltip tooltip = new Tooltip(tooltipText);
        tooltip.setShowDelay(Duration.millis(300));
        tooltip.setFont(new Font(10));
        Tooltip.install(this, tooltip);
    }

    /// Update the station rectangle position based on its X coordinate.
    public void updatePosition() {
        if (scaleMapper.isCircular()) {
            Point2D canvasPosition = scaleMapper.toCanvasPoint(station.getLocation().getX(), perimeterY, 15);
            this.setLayoutX(canvasPosition.getX() - STATION_WIDTH / 2.0);
            this.setLayoutY(canvasPosition.getY() - STATION_HEIGHT / 2.0);
        } else {
            this.setLayoutX(scaleMapper.toCanvasX(station.getLocation().getX()));
            this.setLayoutY(perimeterY - STATION_HEIGHT);
        }
    }

    /**
     * Update visual representation based on the current station state.
     * Called each frame to reflect queue size and status changes.
     */
    public void updateFrame() {
        // Update color based on status
        updateStationColor();

        // Update queue label
        queueLabel.setText(String.valueOf(station.getQueueSize()));
    }

    private void updateStationColor() {
        Color stationColor = switch (station.getStatus()) {
            case AVAILABLE -> Color.GREEN;
            case OCCUPIED -> Color.YELLOW;
            case FAIL -> Color.RED;
        };
        stationRect.setFill(stationColor);
    }
}
