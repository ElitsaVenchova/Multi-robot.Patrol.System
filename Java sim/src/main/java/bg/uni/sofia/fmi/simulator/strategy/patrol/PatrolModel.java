package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;

public interface PatrolModel {
    //setup (positions, phases)
    void initialize(Bot bot, World world);
    //called every tick
    public Position execute(Bot bot, World world, int currentTime);
    //Сканиране за нарушители с лидара
    static void scan(Bot bot, World world, int currentTime) {
        // Сканиране с лидара за нарушители
        bot.getLidar().detect(bot.getPosition(), world.getPerimeter(), currentTime);
        bot.getBattery().consume(bot.getLidar().batteryConsumptionRate());
    }
}
