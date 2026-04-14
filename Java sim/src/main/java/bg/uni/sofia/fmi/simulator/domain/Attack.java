package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;

// Клас, представляващ атака в симулацията
public class Attack {
    private Integer duration; // null = infinite
    
    private long id; // за да може да се идентифицира бота при нужда, напр. за логване
    private Position position; // позиция на атаката
    private AttackStatus status; // статус на атаката
    private int creationTime; // време на създаване на атаката
    private int detectionTime = -1; // време на засичане (ако е засечена)

    public Attack(Position position, int creationTime, Integer duration) {
        this.duration = duration;
        
        this.id = IdGenerator.nextId();
        this.status = AttackStatus.ACTIVE;
        this.position = position;
        this.creationTime = creationTime;
    }

    public boolean isExpired(int currentTime) {
        // Ако няма зададена продължителност, се засича "време за засичане"
        if (duration == null)
            return false;

        return (currentTime - creationTime) > duration;
    }

    public void intercept(int time) {
        this.status = AttackStatus.INTERCEPTED;
        this.detectionTime = time;
    }

    public Position getPosition() {
        return position;
    }

    public AttackStatus getStatus() {
        return status;
    }

    public void intercept() {
        this.status = AttackStatus.INTERCEPTED;
    }

    public void miss() {
        this.status = AttackStatus.MISSED;
    }

    public boolean isActive() {
        return status == AttackStatus.ACTIVE;
    }

    public int getCreationTime() {
        return creationTime;
    }

    public int getDetectionTime() {
        return detectionTime;
    }

    public long getId() {
        return id;
    }
}
