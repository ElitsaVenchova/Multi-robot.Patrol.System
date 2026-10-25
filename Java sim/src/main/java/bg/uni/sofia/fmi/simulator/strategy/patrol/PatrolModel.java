package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;

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
    // Секцията за патрулиране, ако е приложимо, иначе връща null
    default PatrolSection getPatrolSection() {
        return null;
    }
}
