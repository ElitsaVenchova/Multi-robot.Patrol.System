package bg.uni.sofia.fmi.simulator.concurrency;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

//Core thread pool manager.
public class ParallelExecutor {

    private ExecutorService executor;

    public ParallelExecutor(int threads) {
        this.executor = Executors.newFixedThreadPool(threads);
    }

    public ExecutorService getExecutor() {
        return executor;
    }

    public void shutdown() {
        executor.shutdown();
    }
}