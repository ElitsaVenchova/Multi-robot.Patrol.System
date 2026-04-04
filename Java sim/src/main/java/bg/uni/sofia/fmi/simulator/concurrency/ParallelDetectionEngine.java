package bg.uni.sofia.fmi.simulator.concurrency;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Perimeter;

import java.util.ArrayList;

//Parallel detection (bots vs attacks).
public class ParallelDetectionEngine {

    private ExecutorService executor;

    public ParallelDetectionEngine(ExecutorService executor) {
        this.executor = executor;
    }

    public void detect(List<Bot> bots, Perimeter perimeter, int currentTime) {

        List<Future<?>> futures = new ArrayList<>();

        for (Bot bot : bots) {

            futures.add(executor.submit(() -> {
                bot.getLidar().detect(
                        bot.getPosition(),
                        perimeter,
                        currentTime);
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
