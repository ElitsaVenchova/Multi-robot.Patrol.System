package bg.uni.sofia.fmi.simulator.domain;

//[TODO] Да се добави различни видове зарядни станции, които имат различна скорост на зареждане и 
// и могат да зареждат няколко бота едновременно
public class ChargeStation {
    private Position position;

    public ChargeStation(Position position) {
        this.position = position;
    }

    public boolean isAtStation(Position botPosition) {
        return botPosition.getValue() == position.getValue();
    }
}
