package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;

// Интерфейс за поведенчески модул, който определя как ботът взема 
// решения въз основа на текущото състояние на света и времето
public interface BehaviorModule {
    // Метод за актуализиране на поведението на бота, който се извиква всеки такт на симулацията
    void update(Bot bot, World world, int currentTime);

}