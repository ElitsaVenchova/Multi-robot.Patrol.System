package bg.uni.sofia.fmi.simulator.engine;

/**
 * Callback interface for simulation tick events.
 * 
 * Allows the visualizer (or other components) to be notified after each simulation tick
 * without creating tight coupling between simulation and visualization layers.
 * 
 * Usage:
 * - Pass null for fast execution (non-visualization case)
 * - Pass implementation for real-time monitoring (visualization case)
 */
public interface SimulationCallback {
    /**
     * Called after each simulation tick completes.
     * 
     * @param currentTime The current simulation time (tick number)
     */
    void onTick(int currentTime);
}
