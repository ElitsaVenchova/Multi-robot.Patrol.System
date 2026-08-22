package bg.uni.sofia.fmi.simulator.behavior.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.behavior.navigation.Navigation;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;

// Интерфейс за поведенчески модул, който определя как ботът взема 
// решения въз основа на текущото състояние на света и времето
public interface PlanningModule {
    // Метод за актуализиране на поведението на бота, който се извиква всеки такт на симулацията
    void update(Bot bot, int currentTime);

    // За да може различните стратегии да използват навигацията
    public Navigation getNavigation();
    // Стратегията за патрулиране
    public PatrolModel getPatrolModel();

}