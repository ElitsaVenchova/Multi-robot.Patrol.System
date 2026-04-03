package bg.uni.sofia.fmi.simulator.domain;

public class Lidar {
    private double range;

    public Lidar(double range) {
        this.range = range;
    }

    //[TODO] Може би ще е по-добре да проверява "петиметъра" по обхвата,
    // а не само разстоянието между бота и всяка атака
    public boolean detect(Position botPosition, Position attackPosition) {
        return Math.abs(botPosition.getValue() - attackPosition.getValue()) <= range;
    }
}
