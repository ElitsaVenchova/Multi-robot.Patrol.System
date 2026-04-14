package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за патрулирането
public class PatrolConfig {
    private String model; //името на вида патрулиране, което може да се използва за избор на конкретна реализация на патрулирането в симулацията
    private Integer robotsPerSection; // опционално за някои видове патрулиране, като [TODO] асинхронно

    public String getModel() {
        return model;
    }

    public void setModel(String type) {
        this.model = type;
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}
