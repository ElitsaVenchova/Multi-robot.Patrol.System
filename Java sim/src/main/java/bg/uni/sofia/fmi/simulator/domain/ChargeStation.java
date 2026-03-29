package bg.uni.sofia.fmi.simulator.domain;

public class ChargeStation {
    private Position position;

    public ChargeStation(Position position) {
        this.position = position;
    }

    public boolean isAtStation(Position botPosition) {
        return botPosition.getValue() == position.getValue();
    }
}
