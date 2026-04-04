package bg.uni.sofia.fmi.simulator.concurrency;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

import java.util.ArrayList;

//Parallel detection (bots vs attacks).
public class ParallelDetectionEngine {

    private ExecutorService executor;

    public ParallelDetectionEngine(ExecutorService executor) {
        this.executor = executor;
    }

    public void detect(List<Bot> bots, List<Attack> attacks, int currentTime) {

        List<Future<?>> futures = new ArrayList<>();

        for (Bot bot : bots) {
            futures.add(executor.submit(() -> {
                for (Attack attack : attacks) {

                    if (attack.getStatus() == AttackStatus.ACTIVE &&
                            bot.getLidar().detect(bot.getPosition(), attack)) {

                        synchronized (attack) {
                            if (attack.getStatus() == AttackStatus.ACTIVE) {
                                attack.intercept(currentTime);
                            }
                        }
                    }
                }
            }));
        }

        waitAll(futures);
    }

    private void waitAll(List<Future<?>> futures) {
        for (Future<?> f : futures) {
            try {
                f.get();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }
}
