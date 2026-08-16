package bg.uni.sofia.fmi.simulator.strategy.patrol;

public enum Direction {
    LEFT,
    RIGHT;

    public Direction reverse() {
        return this == LEFT ? RIGHT : LEFT;
    }
}