package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Position;

/**
 * A section with defensively copied endpoints. Circular encounter sections use
 * increasing unwrapped coordinates: [80, 120] crosses the origin on a ring of
 * length 100, [80, 180] covers the ring, and [80, 80] is a point.
 */
public record PatrolSection(Position startPosition, Position endPosition) {
    public PatrolSection {
        if (startPosition == null || endPosition == null) {
            throw new IllegalArgumentException("Start and end positions cannot be null");
        }
        startPosition = copy(startPosition);
        endPosition = copy(endPosition);
    }

    public Position startPosition() {
        return copy(startPosition);
    }

    public Position endPosition() {
        return copy(endPosition);
    }

    private static Position copy(Position position) {
        return new Position(position.getX(), position.getY(), position.getZ());
    }

    /** Endpoint distance; for increasing 1D unwrapped sections this is the arc length. */
    public double getLength() {
        return Math.sqrt(
                Math.pow(endPosition.getX() - startPosition.getX(), 2) +
                        Math.pow(endPosition.getY() - startPosition.getY(), 2) +
                        Math.pow(endPosition.getZ() - startPosition.getZ(), 2)
        );
    }

    @Override
    public String toString() {
        return String.format(" PatrolSection[start: %s, end: %s]", startPosition, endPosition);
    }
}