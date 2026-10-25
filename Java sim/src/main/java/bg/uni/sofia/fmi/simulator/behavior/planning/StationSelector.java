package bg.uni.sofia.fmi.simulator.behavior.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;
import bg.uni.sofia.fmi.simulator.domain.enums.ChargingStationStatus;

// Клас, отговорен за избора на най-добрата станция за зареждане, когато батерията е ниска
public class StationSelector {
    public ChargingStation selectBestStation(Bot bot) {
        ChargingStation best = null;
        double bestScore = Double.MAX_VALUE;

        for (ChargingStation s : World.getChargingStations()) {
            // Ако станцията е в грешка, я пропускаме
            if (s.getStatus() == ChargingStationStatus.FAIL) {
                continue;
            }

            // Изчисляваме "оценка" за станцията, която включва разстояние и време за чакане
            double distance = World.getPerimeter().getType() == PerimeterType.CIRCULAR
                    ? GeometryUtils.ringDistance(bot.getPosition().getX(), s.getLocation().getX())
                    : GeometryUtils.euclideanDistance(bot.getPosition(), s.getLocation());
            int waitTime = s.getEstimatedWaitTime();
            double score = distance + waitTime;

            if (score < bestScore) {
                bestScore = score;
                best = s;
            }
        }

        return best;
    }
}
