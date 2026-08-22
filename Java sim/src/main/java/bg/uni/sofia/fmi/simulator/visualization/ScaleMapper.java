package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import javafx.geometry.Point2D;

/**
 * Helper class for mapping between simulation coordinates (perimeter position)
 * and canvas coordinates (pixels).
 * 
 * The perimeter is represented as a 1D line segment on the canvas.
 * Perimeter position [0, perimeterSize) maps to canvas X [canvasMinX, canvasMaxX]
 */
public class ScaleMapper {
    private final int perimeterSize;
    private final double canvasMinX;
    private final double canvasMaxX;
    private final PerimeterType perimeterType;
    private final double centerX;
    private final double centerY;
    private final double radius;

    public ScaleMapper(int perimeterSize, double canvasMinX, double canvasMaxX) {
        this(perimeterSize, canvasMinX, canvasMaxX, PerimeterType.LINEAR, 0, 0, 0);
    }

    private ScaleMapper(int perimeterSize, double canvasMinX, double canvasMaxX,
                        PerimeterType perimeterType, double centerX, double centerY, double radius) {
        this.perimeterSize = perimeterSize;
        this.canvasMinX = canvasMinX;
        this.canvasMaxX = canvasMaxX;
        this.perimeterType = perimeterType;
        this.centerX = centerX;
        this.centerY = centerY;
        this.radius = radius;
    }

    public static ScaleMapper circular(int perimeterSize, double centerX, double centerY, double radius) {
        return new ScaleMapper(perimeterSize, 0, 0, PerimeterType.CIRCULAR, centerX, centerY, radius);
    }

    /**
     * Convert perimeter position (0 to perimeterSize) to canvas X coordinate.
     */
    public double toCanvasX(double perimeterX) {
        if (isCircular()) {
            return toCanvasPoint(perimeterX, 0).getX();
        }
        if (perimeterSize == 0) {
            return canvasMinX;
        }
        double ratio = perimeterX / perimeterSize;
        ratio = Math.max(0, Math.min(1, ratio)); // Clamp to [0, 1]
        return canvasMinX + ratio * (canvasMaxX - canvasMinX);
    }

    /**
     * Convert canvas X coordinate back to perimeter position.
     */
    public double toPerimeterX(double canvasX) {
        if (isCircular()) {
            throw new UnsupportedOperationException("Circular coordinates require both X and Y values");
        }
        double canvasRange = canvasMaxX - canvasMinX;
        if (canvasRange == 0) {
            return 0;
        }
        double ratio = (canvasX - canvasMinX) / canvasRange;
        ratio = Math.max(0, Math.min(1, ratio)); // Clamp to [0, 1]
        return ratio * perimeterSize;
    }

    public int getPerimeterSize() {
        return perimeterSize;
    }

    public double getCanvasMinX() {
        return canvasMinX;
    }

    public double getCanvasMaxX() {
        return canvasMaxX;
    }

    public double getCanvasRangeX() {
        return canvasMaxX - canvasMinX;
    }

    public boolean isCircular() {
        return perimeterType == PerimeterType.CIRCULAR;
    }

    /** Map a perimeter position to a canvas point on the configured perimeter shape. */
    public Point2D toCanvasPoint(double perimeterPosition, double linearY) {
        return toCanvasPoint(perimeterPosition, linearY, 0);
    }

    /** Map a position with an optional outward offset from a circular perimeter. */
    public Point2D toCanvasPoint(double perimeterPosition, double linearY, double radialOffset) {
        if (!isCircular()) {
            return new Point2D(toCanvasX(perimeterPosition), linearY);
        }

        double angleRadians = Math.toRadians(toAngleDegrees(perimeterPosition));
        double effectiveRadius = radius + radialOffset;
        return new Point2D(
                centerX + effectiveRadius * Math.cos(angleRadians),
                centerY + effectiveRadius * Math.sin(angleRadians)
        );
    }

    /** The clockwise canvas angle for a position; position zero is at the top of the circle. */
    public double toAngleDegrees(double perimeterPosition) {
        if (perimeterSize == 0) {
            return -90;
        }
        double ratio = perimeterPosition / perimeterSize;
        ratio = Math.max(0, Math.min(1, ratio));
        return -90 + ratio * 360;
    }

    /** Rotation for a mark initially drawn upward from its origin. */
    public double toMarkerRotationDegrees(double perimeterPosition) {
        return isCircular() ? toAngleDegrees(perimeterPosition) + 90 : 0;
    }
}
