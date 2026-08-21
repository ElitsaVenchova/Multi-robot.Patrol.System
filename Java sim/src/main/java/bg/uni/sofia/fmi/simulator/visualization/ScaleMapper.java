package bg.uni.sofia.fmi.simulator.visualization;

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

    public ScaleMapper(int perimeterSize, double canvasMinX, double canvasMaxX) {
        this.perimeterSize = perimeterSize;
        this.canvasMinX = canvasMinX;
        this.canvasMaxX = canvasMaxX;
    }

    /**
     * Convert perimeter position (0 to perimeterSize) to canvas X coordinate.
     */
    public double toCanvasX(double perimeterX) {
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
}
