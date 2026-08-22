package bg.uni.sofia.fmi.simulator.engine;

import bg.uni.sofia.fmi.simulator.domain.World;

/**
 * Core simulation engine that executes the main simulation loop.
 * 
 * Extracted from SimulationRunner to support both:
 * - Fast non-visualization execution (callback=null)
 * - Real-time visualization (callback notified each tick)
 * 
 * Design:
 * - Encapsulates simulation loop logic in one place
 * - Callback pattern allows decoupled monitoring
 * - Thread-safe: runs on calling thread, visualizer reads on JavaFX thread
 */
public class Simulator {
    
    /**
     * Execute the simulation for the given duration.
     * 
     * @param world The world to simulate
     * @param duration Number of ticks to simulate
     * @param callback Optional callback called after each tick (null for no-op)
     */
    public void run(World world, int duration, SimulationCallback callback) {
        for (int t = 0; t < duration; t++) {
            // Advance simulation
            world.tick(t);
            
            // Notify callback if provided
            // Used by visualizer to synchronize animation with simulation
            if (callback != null) {
                callback.onTick(t);
            }
        }
    }
}
