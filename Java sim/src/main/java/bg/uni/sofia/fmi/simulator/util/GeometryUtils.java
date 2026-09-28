package bg.uni.sofia.fmi.simulator.util;

import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;

/**
 * Помощен клас за геометрични и пространствени изчисления в симулацията.
 */
public class GeometryUtils {

    // Изчисляване на 2D Евклидово разстояние между координати (x1, y1) и (x2, y2)
    public static double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // Изчисляване на 3D Евклидово разстояние между координати
    public static double distance(double x1, double y1, double z1, double x2, double y2, double z2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Изчисляване на Евклидово разстояние между два Position обекта
    public static double euclideanDistance(Position a, Position b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("Positions cannot be null");
        }
        return distance(a.getX(), a.getY(), a.getZ(), b.getX(), b.getY(), b.getZ());
    }

    // Разстояние между позиции според типа на периметъра
    public static double perimeterDistance(Position a, Position b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("Positions cannot be null");
        }
        if (World.getPerimeter().getType() == PerimeterType.CIRCULAR) {
            return ringDistance(a.getX(), b.getX());
        }
        return Math.abs(a.getX() - b.getX());
    }

    // Нормализиране на позиция в затворен цикличен периметър [0, perimeterLength)
    public static double normalizePosition(double pos) {
        if (World.getPerimeter().getSize() <= 0) {
            throw new IllegalArgumentException("perimeterLength must be positive");
        }
        double normalized = pos % World.getPerimeter().getSize();
        if (normalized < 0) {
            normalized += World.getPerimeter().getSize();
        }
        // За избягване на -0.0
        return normalized == 0.0 ? 0.0 : normalized;
    }

    // Най-кратко разстояние по окръжност / затворен контур (ring distance)
    public static double ringDistance(double x1, double x2) {
        if (World.getPerimeter().getSize() <= 0) {
            throw new IllegalArgumentException("perimeterLength must be positive");
        }
        double norm1 = normalizePosition(x1);
        double norm2 = normalizePosition(x2);

        double direct = Math.abs(norm1 - norm2);
        return Math.min(direct, World.getPerimeter().getSize() - direct);
    }

    // Определяне на най-кратката посока на движение по окръжност (+1 за посока на нарастване / по часовниковата стрелка, -1 за обратна, 0 ако са на еднаква позиция)
    public static int shortestRingDirection(double from, double to) {
        if (World.getPerimeter().getSize() <= 0) {
            throw new IllegalArgumentException("perimeterLength must be positive");
        }
        double normFrom = normalizePosition(from);
        double normTo = normalizePosition(to);

        double diff = normTo - normFrom;
        if (Math.abs(diff) < 1e-9 || Math.abs(Math.abs(diff) - World.getPerimeter().getSize()) < 1e-9) {
            return 0;
        }

        // Forward distance (по посока на нарастване)
        double forwardDist = diff > 0 ? diff : diff + World.getPerimeter().getSize();
        // Backward distance
        double backwardDist = World.getPerimeter().getSize() - forwardDist;

        if (Math.abs(forwardDist - backwardDist) < 1e-9) {
            // При точно срещуположна позиция избираме посока напред (+1)
            return 1;
        }

        return forwardDist < backwardDist ? 1 : -1;
    }
}
