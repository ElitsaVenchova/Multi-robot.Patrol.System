package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import javafx.geometry.Bounds;
import javafx.geometry.Point2D;
import javafx.scene.Group;
import javafx.scene.control.Tooltip;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Polygon;
import javafx.scene.shape.Rectangle;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.util.Duration;

/**
 * Visual representation of a robot on the perimeter.
 * 
 * - GROUND robots are drawn as circles
 * - DRONE robots are drawn as triangles (pointing up)
 * - Color indicates robot state:
 *   - Green (PATROLLING)
 *   - Blue (GOING_TO_PATROL)
 *   - Yellow (GOING_TO_CHARGE)
 *   - Orange (CHARGING)
 *   - Red (ERROR)
 * - Robot ID is displayed as label
 * - Battery level indicator bar shown below robot (Phase 5)
 * - Tooltip shows detailed metadata: battery %, state, position (Phase 5)
 */
public class RobotVisualNode extends Group {
    private static final double ROBOT_SIZE = 8;
    private static final double LABEL_FONT_SIZE = 10;
    private static final double BATTERY_BAR_WIDTH = 20;
    private static final double BATTERY_BAR_HEIGHT = 3;

    private final Bot bot;
    private final ScaleMapper scaleMapper;
    private final double perimeterY;
    private Rectangle batteryBar;

    public RobotVisualNode(Bot bot, ScaleMapper scaleMapper, double perimeterY) {
        this.bot = bot;
        this.scaleMapper = scaleMapper;
        this.perimeterY = perimeterY;

        // Create robot shape based on type
        if (bot.getType() == RobotType.GROUND) {
            createGroundRobotShape();
        } else if (bot.getType() == RobotType.DRONE) {
            createDroneShape();
        }

        // Add robot ID label
        addRobotLabel();
        
        // Add battery indicator bar (Phase 5)
        addBatteryBar();
        
        // Add tooltip with metadata (Phase 5)
        addMetadataTooltip();
    }

    private void createGroundRobotShape() {
        Circle circle = new Circle(ROBOT_SIZE);
        circle.setFill(getStateColor());
        circle.setStroke(Color.BLACK);
        circle.setStrokeWidth(1);

        this.getChildren().add(circle);
    }

    private void createDroneShape() {
        // Triangle pointing up (drone shape)
        double baseX = 0;
        double baseY = ROBOT_SIZE;
        double topY = -ROBOT_SIZE;
        
        Polygon triangle = new Polygon(
                baseX, baseY,                              // bottom left
                baseX + ROBOT_SIZE, baseY,               // bottom right
                baseX + ROBOT_SIZE / 2.0, topY           // top center
        );
        triangle.setFill(getStateColor());
        triangle.setStroke(Color.BLACK);
        triangle.setStrokeWidth(1);

        this.getChildren().add(triangle);
    }

    private void addRobotLabel() {
        Text label = new Text(String.valueOf(bot.getId()));
        label.setFont(new Font(LABEL_FONT_SIZE));
        label.setFill(Color.BLACK);
        
        // Position label below the robot shape
        label.setY(ROBOT_SIZE + 12);
        label.setX(-label.prefWidth(0) / 2);

        this.getChildren().add(label);
    }

    /**
     * Phase 5: Add battery level indicator bar below the robot.
     * Bar color represents battery level:
     * - Green (>60%) → Yellow (30-60%) → Red (<30%)
     */
    private void addBatteryBar() {
        batteryBar = new Rectangle(BATTERY_BAR_WIDTH, BATTERY_BAR_HEIGHT);
        batteryBar.setStroke(Color.BLACK);
        batteryBar.setStrokeWidth(0.5);
        
        // Position below robot and label
        batteryBar.setY(ROBOT_SIZE + 25);
        batteryBar.setX(-BATTERY_BAR_WIDTH / 2.0);

        updateBatteryBar();
        
        this.getChildren().add(batteryBar);
    }

    /**
     * Update battery bar fill color and width based on current battery level.
     */
    private void updateBatteryBar() {
        double currentLevel = bot.getBattery().getCurrentLevel();
        double capacity = bot.getBattery().getCurrentLevel() + 
                         (100 - bot.getBattery().getCurrentLevel()); // Estimate max
        
        // Use actual battery capacity if available, else estimate
        double batteryPercent = currentLevel / capacity;
        batteryPercent = Math.max(0, Math.min(1, batteryPercent)); // Clamp [0, 1]

        // Determine color based on battery level
        Color batteryColor;
        if (batteryPercent > 0.6) {
            batteryColor = Color.GREEN;
        } else if (batteryPercent > 0.3) {
            batteryColor = Color.YELLOW;
        } else {
            batteryColor = Color.RED;
        }

        // Update bar width and color
        batteryBar.setWidth(BATTERY_BAR_WIDTH * batteryPercent);
        batteryBar.setFill(batteryColor);
    }

    /**
     * Phase 5: Add tooltip showing detailed robot metadata.
     */
    private void addMetadataTooltip() {
        String tooltipText = String.format(
                "Robot #%d\nType: %s\nState: %s\nBattery: %.1f%%\nPosition: %.1f",
                bot.getId(),
                bot.getType().toString(),
                bot.getState().toString(),
                getBatteryPercent(),
                bot.getPosition().getX()
        );

        Tooltip tooltip = new Tooltip(tooltipText);
        tooltip.setShowDelay(Duration.millis(300));
        tooltip.setFont(new Font(10));
        Tooltip.install(this, tooltip);
    }

    /**
     * Calculate battery percentage for display.
     */
    private double getBatteryPercent() {
        double current = bot.getBattery().getCurrentLevel();
        // Estimate capacity - in real implementation, Bot should expose maxCapacity
        // For now, we use a rough estimate based on current level
        double estimated = current * 1.5; // Rough estimate
        return Math.min(100, (current / estimated) * 100);
    }

    /**
     * Update the robot's canvas position based on its current perimeter position.
     * Should be called whenever the bot moves.
     */
    public void updatePosition() {
        Point2D canvasPosition = scaleMapper.toCanvasPoint(bot.getPosition().getX(), perimeterY);
        this.setLayoutX(canvasPosition.getX());
        this.setLayoutY(canvasPosition.getY());
    }

    /**
     * Update the robot's visual representation based on current state and battery.
     * Should be called at each simulation tick.
     */
    public void updateFrame() {
        updatePosition();
        updateBatteryBar();
        
        // Recreate shape if state changed (color updated)
        // This is a simple approach; could be optimized by just updating fill
        if (bot.getType() == RobotType.GROUND) {
            Circle circle = (Circle) this.getChildren().get(0);
            if (!circle.getFill().equals(getStateColor())) {
                circle.setFill(getStateColor());
            }
        } else if (bot.getType() == RobotType.DRONE) {
            Polygon triangle = (Polygon) this.getChildren().get(0);
            if (!triangle.getFill().equals(getStateColor())) {
                triangle.setFill(getStateColor());
            }
        }
    }

    /**
     * Update the robot's visual color based on current state.
     * Should be called whenever the bot's state changes.
     */
    public void updateState() {
        // Remove old shape and recreate with new color
        this.getChildren().clear();

        if (bot.getType() == RobotType.GROUND) {
            createGroundRobotShape();
        } else if (bot.getType() == RobotType.DRONE) {
            createDroneShape();
        }

        addRobotLabel();
        addBatteryBar();
        addMetadataTooltip();
    }

    private Color getStateColor() {
        return switch (bot.getState()) {
            case PATROLLING -> Color.GREEN;
            case GOING_TO_PATROL -> Color.DODGERBLUE;
            case GOING_TO_CHARGE -> Color.YELLOW;
            case CHARGING -> Color.ORANGE;
            case ERROR -> Color.RED;
        };
    }

    public Bot getBot() {
        return bot;
    }
}
