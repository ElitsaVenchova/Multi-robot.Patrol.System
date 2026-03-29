package bg.uni.sofia.fmi.simulator.concurrency;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

import bg.uni.sofia.fmi.simulator.domain.Bot;

import java.util.ArrayList;

//Parallelizes robot movement.
public class ParallelPatrolExecutor {

    private ExecutorService executor;

    public ParallelPatrolExecutor(ExecutorService executor) {
        this.executor = executor;
    }

    public void execute(List<Bot> bots) {

        List<Future<?>> futures = new ArrayList<>();

        for (Bot bot : bots) {
            futures.add(executor.submit(bot::move));
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