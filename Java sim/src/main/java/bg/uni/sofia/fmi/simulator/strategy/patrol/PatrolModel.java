package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

public interface PatrolModel {
    //setup (positions, phases)
    void initialize(Bot bot);
    /** Lifecycle notifications; existing strategies need not react. */
    default void onPatrolSuspended(Bot bot) { }

    default void onPatrolResumed(Bot bot) { }

    /** Null retains shortest-route navigation, including initial deployment. */
    default Direction getMovementDirection() {
        return null;
    }
    //called every tick
    Position execute(Bot bot,int currentTime);
    //Сканиране за нарушители с лидара
    static void scan(Bot bot, int currentTime) {
        // Сканиране с лидара за нарушители
        bot.getLidar().detect(bot.getPosition(), World.getPerimeter(), currentTime);
        bot.getBattery().consume(bot.getLidar().batteryConsumptionRate());
    }

    //Проверява дали робота е в секцията за патрулиране
    public default boolean isInPatrolSection(Bot bot, PatrolSection patrolSection) {
        if (patrolSection == null) {
            return false;
        }
        if (World.getPerimeter().getType() == PerimeterType.LINEAR) {
            double minX = Math.min(patrolSection.startPosition().getX(), patrolSection.endPosition().getX());
            double maxX = Math.max(patrolSection.startPosition().getX(), patrolSection.endPosition().getX());
            return bot.getPosition().getX() >= minX && bot.getPosition().getX() <= maxX;
        } else {
            double rawStart = patrolSection.startPosition().getX();
            double span = Math.abs(patrolSection.endPosition().getX() - rawStart);
            if (span >= World.getPerimeter().getSize()) {
                return true; // An unwrapped full-ring section contains every position.
            }
            double x = GeometryUtils.normalizePosition(bot.getPosition().getX());
            double start = GeometryUtils.normalizePosition(patrolSection.startPosition().getX());
            double rawEnd = patrolSection.endPosition().getX();
            double end = GeometryUtils.normalizePosition(rawEnd);
            // Секцията завършва точно в края на периметъра,  който е физически същата точка като 0.
            boolean endsAtSeam = rawEnd > 0 && end == 0;
            if (endsAtSeam) {
                return x >= start || x < 0.001;
            }
            if (start <= end) { // Обикновена секция: например 20 → 60
                return x >= start && x <= end;
            }
            return x >= start || x <= end; // Секция, която преминава през 0: например 80 → 20
        }
    }

    // Секцията за патрулиране, ако е приложимо, иначе връща null
    default PatrolSection getPatrolSection() {
        return null;
    }
}
