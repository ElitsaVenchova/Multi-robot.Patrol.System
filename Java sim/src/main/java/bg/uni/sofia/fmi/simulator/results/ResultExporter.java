package bg.uni.sofia.fmi.simulator.results;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.util.FileUtils;

// This class will handle exporting the results to a CSV file
// It will create a file "simulation_results.csv" in the "outputs/results" directory
public class ResultExporter {

    private static final String OUTPUT_DIR = "outputs/results/";
    private static final String FILE_NAME = "simulation_results.csv";

    public void export(SimulationMetrics metrics) {
        FileUtils.ensureDirectoryExists(OUTPUT_DIR);

        File file = new File(OUTPUT_DIR + FILE_NAME);
        boolean fileExists = file.exists();

        try (FileWriter writer = new FileWriter(file, true)) {

            // Write header only once
            if (!fileExists) {
                writer.append("total_attacks,intercepted,missed,success_rate,avg_detection_time\n");
            }

            // Write data row
            writer.append(String.valueOf(metrics.getTotalAttacks())).append(",");
            writer.append(String.valueOf(metrics.getInterceptedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getMissedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getAverageDetectionTime())).append("\n");

        } catch (IOException e) {
            throw new RuntimeException("Error writing results file", e);
        }
    }

    public void exportWithContext(SimulationMetrics metrics, SimulationConfig config) {

        FileUtils.ensureDirectoryExists(OUTPUT_DIR);

        File file = new File(OUTPUT_DIR + FILE_NAME);
        boolean fileExists = file.exists();

        try (FileWriter writer = new FileWriter(file, true)) {

            if (!fileExists) {
                writer.append("total_attacks,intercepted,missed,success_rate,avg_detection_time,avg_response_time\n");
            }

            writer.append(String.valueOf(config.getPatrolModel().getRobotsPerSection())).append(",");
            writer.append(String.valueOf(config.getAttackModel().getLambda())).append(",");
            writer.append(String.valueOf(metrics.getTotalAttacks())).append(",");
            writer.append(String.valueOf(metrics.getInterceptedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getMissedAttacks())).append(",");
            writer.append(String.valueOf(metrics.getSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getAverageDetectionTime())).append(",");
            writer.append(String.valueOf(metrics.getAverageResponseTime())).append("\n");

        } catch (IOException e) {
            throw new RuntimeException("Error writing results file", e);
        }
    }

    public void exportAggregated(AggregatedMetrics metrics,
            String strategy,
            double lambda) {

        FileUtils.ensureDirectoryExists(OUTPUT_DIR);

        File file = new File(OUTPUT_DIR + "aggregated_results.csv");
        boolean fileExists = file.exists();

        try (FileWriter writer = new FileWriter(file, true)) {

            if (!fileExists) {
                writer.append("strategy,lambda,runs,mean_success,std_success,mean_detection,std_detection\n");
            }

            writer.append(strategy).append(",");
            writer.append(String.valueOf(lambda)).append(",");
            writer.append(String.valueOf(metrics.getRuns())).append(",");
            writer.append(String.valueOf(metrics.getMeanSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getStdSuccessRate())).append(",");
            writer.append(String.valueOf(metrics.getMeanDetectionTime())).append(",");
            writer.append(String.valueOf(metrics.getStdDetectionTime())).append("\n");

        } catch (IOException e) {
            throw new RuntimeException("Error writing aggregated results", e);
        }
    }
}
