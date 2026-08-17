package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Position;

//Секцията, в която ще платрулира робота
public class PatrolSection {
    private final Position startPosition;
    private final Position endPosition;

    public PatrolSection(Position startPosition, Position endPosition) {
        if (startPosition == null || endPosition == null) {
            throw new IllegalArgumentException("Start and end positions cannot be null");
        }
        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }

    public Position getStartPosition() {
        return startPosition;
    }

    public Position getEndPosition() {
        return endPosition;
    }

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
