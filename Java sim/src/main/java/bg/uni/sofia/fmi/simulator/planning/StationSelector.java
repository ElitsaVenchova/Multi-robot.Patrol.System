package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.ChargingStationStatus;

// Клас, отговорен за избора на най-добрата станция за зареждане, когато батерията е ниска
public class StationSelector {
    public ChargingStation selectBestStation(Bot bot) {
        ChargingStation best = null;
        double bestScore = Double.MAX_VALUE;

        for (ChargingStation s : bot.getWorld().getChargingStations()) {
            // Ако станцията е в грешка, я пропускаме
            if (s.getStatus() == ChargingStationStatus.FAIL) {
                continue;
            }

            // Изчисляваме "оценка" за станцията, която включва разстояние и време за чакане
            double distance = distance(bot.getPosition(), s.getLocation());
            int waitTime = s.getEstimatedWaitTime();
            double score = distance + waitTime;

            if (score < bestScore) {
                bestScore = score;
                best = s;
            }
        }

        return best;
    }

    private double distance(Position a, Position b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
