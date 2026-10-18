package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import javafx.geometry.Point2D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Polyline;
import javafx.scene.shape.StrokeLineCap;

/** Visualizes a bot's communication boundary and circular LiDAR coverage. */
public class BotRangeVisualNode extends Group {
    private static final double LIDAR_STROKE_WIDTH = 2.5;
    private static final int BRACKET_SEGMENTS = 16;
    private static final Color LIDAR_COLOR = Color.ORANGE;
    private static final Color COMMUNICATION_COLOR = Color.BLUE;

    private final Bot bot;
    private final ScaleMapper scaleMapper;
    private final double perimeterY;
    private final Group communicationGlyph = new Group();
    private final Group lidarGlyph = new Group();
    private final Polyline communicationLeftBoundary = new Polyline();
    private final Polyline communicationRightBoundary = new Polyline();
    private final Circle lidarRange = new Circle();
    private boolean communicationVisible = true;
    private boolean lidarVisible = true;

    public BotRangeVisualNode(Bot bot, ScaleMapper scaleMapper, double perimeterY) {
        this.bot = bot;
        this.scaleMapper = scaleMapper;
        this.perimeterY = perimeterY;

        configureCommunicationBoundary(communicationLeftBoundary);
        configureCommunicationBoundary(communicationRightBoundary);
        communicationGlyph.getChildren().addAll(communicationLeftBoundary, communicationRightBoundary);

        lidarRange.setFill(LIDAR_COLOR.deriveColor(0, 1, 1, 0.12));
        lidarRange.setStroke(LIDAR_COLOR.deriveColor(0, 1, 0.75, 0.8));
        lidarRange.setStrokeWidth(LIDAR_STROKE_WIDTH);
        lidarRange.setStrokeLineCap(StrokeLineCap.ROUND);
        lidarGlyph.getChildren().add(lidarRange);
        getChildren().addAll(lidarGlyph, communicationGlyph);
        setMouseTransparent(true);
        updateFrame();
    }

    private void configureCommunicationBoundary(Polyline boundary) {
        boundary.setStroke(COMMUNICATION_COLOR);
        boundary.setStrokeWidth(2.5);
        boundary.setFill(Color.TRANSPARENT);
        boundary.setStrokeLineJoin(javafx.scene.shape.StrokeLineJoin.ROUND);
    }

    public void setCommunicationVisible(boolean visible) {
        communicationVisible = visible;
        communicationGlyph.setVisible(visible);
    }

    public void setLidarVisible(boolean visible) {
        lidarVisible = visible;
        lidarGlyph.setVisible(visible);
    }

    /** Refresh the overlays after the bot moves. */
    public void updateFrame() {
        double position = bot.getPosition().getX();
        Point2D botPoint = scaleMapper.toCanvasPoint(position, perimeterY);
        communicationGlyph.setLayoutX(botPoint.getX());
        communicationGlyph.setLayoutY(botPoint.getY());
        communicationGlyph.setRotate(scaleMapper.isCircular()
                ? scaleMapper.toMarkerRotationDegrees(position)
                : 0);

        double communicationRadius = Math.max(0,
                scaleMapper.toCanvasDistance(bot.getCommunicationModule().communicationRange()));
        updateCommunicationBracket(communicationLeftBoundary, communicationRadius, 120, 240);
        updateCommunicationBracket(communicationRightBoundary, communicationRadius, -60, 60);
        communicationGlyph.setVisible(communicationVisible && communicationRadius > 0);

        updateLidarRange(botPoint, Math.max(0, bot.getLidar().range()));
    }

    private void updateLidarRange(Point2D botPoint, double range) {
        lidarRange.setCenterX(botPoint.getX());
        lidarRange.setCenterY(botPoint.getY());
        lidarRange.setRadius(Math.max(0, scaleMapper.toCanvasDistance(range)));
        lidarGlyph.setVisible(lidarVisible && range > 0);
    }

    private void updateCommunicationBracket(Polyline bracket, double radius, double startDegrees, double endDegrees) {
        bracket.getPoints().clear();
        for (int index = 0; index <= BRACKET_SEGMENTS; index++) {
            double angle = Math.toRadians(startDegrees + (endDegrees - startDegrees) * index / BRACKET_SEGMENTS);
            bracket.getPoints().addAll(radius * Math.cos(angle), radius * Math.sin(angle));
        }
    }
}
