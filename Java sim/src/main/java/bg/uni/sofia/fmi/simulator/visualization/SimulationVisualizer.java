package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.World;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;
import javafx.scene.shape.Line;
import javafx.stage.Stage;

public class SimulationVisualizer extends Application {
    private static World world;
    private static final double CANVAS_PADDING_X = 50;
    private static final double CANVAS_WIDTH = 900;
    private static final double PERIMETER_Y = 300;
    private static final double LABEL_OFFSET_Y = 5;

    public SimulationVisualizer() {
    }

    @Override
    public void start(Stage stage) {
        Pane root = new Pane();

        // Get perimeter size from world (Phase 3: Real perimeter)
        int perimeterSize = world.getPerimeter().getSize();
        
        // Create scale mapper for coordinate transformations
        ScaleMapper scaleMapper = new ScaleMapper(
                perimeterSize,
                CANVAS_PADDING_X,
                CANVAS_PADDING_X + CANVAS_WIDTH
        );

        // Draw perimeter line based on real world size
        Line perimeter = new Line(
                CANVAS_PADDING_X, PERIMETER_Y,
                CANVAS_PADDING_X + CANVAS_WIDTH, PERIMETER_Y
        );

        // Start label (position 0)
        Label startLabel = new Label("0");
        startLabel.setLayoutX(CANVAS_PADDING_X - 5);
        startLabel.setLayoutY(PERIMETER_Y + LABEL_OFFSET_Y);

        // End label (perimeter size)
        Label endLabel = new Label(String.valueOf(perimeterSize));
        endLabel.setLayoutX(CANVAS_PADDING_X + CANVAS_WIDTH - 30);
        endLabel.setLayoutY(PERIMETER_Y + LABEL_OFFSET_Y);

        root.getChildren().addAll(
                perimeter,
                startLabel,
                endLabel
        );

        Scene scene = new Scene(root, 1000, 600);

        stage.setTitle("Perimeter Surveillance Simulator");
        stage.setScene(scene);
        stage.show();
    }

    /**
     * Launch the visualizer in a background thread.
     * This allows the simulation to continue running while the visualizer window is displayed.
     */
    public static void launch(World simulationWorld) {
        world = simulationWorld;
        // Launch JavaFX application in a separate thread to avoid blocking the simulation
        new Thread(() -> Application.launch(SimulationVisualizer.class)).start();
    }

    public static void main(String[] args) {
        // Test launcher - creates a dummy world for testing
        World testWorld = new World(1000);
        launch(testWorld);
    }
}