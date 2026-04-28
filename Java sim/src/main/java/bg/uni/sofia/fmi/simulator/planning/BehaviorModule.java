package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;

// Интерфейс за поведенчески модул, който определя как ботът взема 
// решения въз основа на текущото състояние на света и времето
public interface BehaviorModule {
    // Метод за актуализиране на поведението на бота, който се извиква всеки такт на симулацията
    void update(Bot bot, int currentTime);

    // За да може различните стратегии да използват навигацията
    public Navigation getNavigation();

}