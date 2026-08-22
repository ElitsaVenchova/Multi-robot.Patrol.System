package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.*;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.Group;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.shape.Shape;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SimulationVisualizer extends Application {
    private static World world;
    private static int duration;
    private static final double CANVAS_PADDING_X = 50;
    private static final double CANVAS_WIDTH = 900;
    private static final double PERIMETER_Y = 300;
    private static final double LABEL_OFFSET_Y = 5;
    private static final double SCENE_WIDTH = 1000;
    private static final double SCENE_HEIGHT = 600;
    private static final double CIRCULAR_CENTER_X = 500;
    private static final double CIRCULAR_CENTER_Y = SCENE_HEIGHT / 2;
    private static final double CIRCULAR_RADIUS = 220;
    private static final double MIN_ZOOM = 0.5;
    private static final double MAX_ZOOM = 3.0;
    private static final double ZOOM_FACTOR = 1.25;
    private static final double PAN_STEP = 50;
    private static final Color[] PATROL_SECTION_COLORS = {
            Color.DODGERBLUE, Color.MEDIUMSEAGREEN, Color.DARKORANGE,
            Color.MEDIUMPURPLE, Color.DEEPSKYBLUE, Color.HOTPINK
    };
    
    // Phase 6: Animation loop members
    private final Map<Bot, RobotVisualNode> robotNodes = new HashMap<>();
    // Phase 7: Charging stations
    private final Map<ChargingStation, ChargingStationVisualNode> stationNodes = new HashMap<>();
    private final Map<Attack, AttackVisualNode> attackNodes = new HashMap<>();
    private final Map<Bot, PatrolSectionVisualNode> patrolSectionNodes = new HashMap<>();
    private final Group worldLayer = new Group();
    private final Scale worldScale = new Scale(1, 1, 0, 0);
    private final Translate worldTranslation = new Translate();
    private final Group patrolSectionLayer = new Group();
    private final Group attackLayer = new Group();
    private ScaleMapper scaleMapper;
    private Label tickLabel;
    private Label attackSummaryLabel;
    private HBox playbackControls;
    private VBox viewportControls;
    private Button pauseButton;
    private Button resumeButton;
    private Button stepButton;
    private AnimationTimer animationTimer;
    private int currentTick = 0;
    private boolean paused;
    private double zoom = 1.0;
    private double panX;
    private double panY;
    private Scene scene;

    public SimulationVisualizer() {
        worldLayer.getTransforms().addAll(worldTranslation, worldScale);
    }

    @Override
    public void start(Stage stage) {
        Pane root = new Pane();

        // Get perimeter size from world
        int perimeterSize = world.getPerimeter().getSize();
        
        PerimeterType perimeterType = world.getPerimeter().getType();
        Shape perimeter;
        List<Label> perimeterLabels = new java.util.ArrayList<>();
        if (perimeterType == PerimeterType.CIRCULAR) {
            scaleMapper = ScaleMapper.circular(
                    perimeterSize, CIRCULAR_CENTER_X, CIRCULAR_CENTER_Y, CIRCULAR_RADIUS
            );
            perimeter = new Circle(CIRCULAR_CENTER_X, CIRCULAR_CENTER_Y, CIRCULAR_RADIUS);
            perimeter.setFill(Color.TRANSPARENT);
            perimeter.setStroke(Color.LIGHTGRAY);
            perimeter.setStrokeWidth(1.5);

            Label circularLabel = new Label("0 / " + perimeterSize);
            circularLabel.setLayoutX(CIRCULAR_CENTER_X + 10);
            circularLabel.setLayoutY(CIRCULAR_CENTER_Y - CIRCULAR_RADIUS - 10);
            perimeterLabels.add(circularLabel);
        } else {
            scaleMapper = new ScaleMapper(
                    perimeterSize,
                    CANVAS_PADDING_X,
                    CANVAS_PADDING_X + CANVAS_WIDTH
            );
            perimeter = new Line(
                    CANVAS_PADDING_X, PERIMETER_Y,
                    CANVAS_PADDING_X + CANVAS_WIDTH, PERIMETER_Y
            );

            Label startLabel = new Label("0");
            startLabel.setLayoutX(CANVAS_PADDING_X - 5);
            startLabel.setLayoutY(PERIMETER_Y + LABEL_OFFSET_Y);
            perimeterLabels.add(startLabel);

            Label endLabel = new Label(String.valueOf(perimeterSize));
            endLabel.setLayoutX(CANVAS_PADDING_X + CANVAS_WIDTH - 30);
            endLabel.setLayoutY(PERIMETER_Y + LABEL_OFFSET_Y);
            perimeterLabels.add(endLabel);
        }

        tickLabel = new Label();
        tickLabel.setLayoutX(15);
        tickLabel.setLayoutY(15);
        updateTickLabel();

        attackSummaryLabel = new Label();
        attackSummaryLabel.setLayoutX(15);
        attackSummaryLabel.setLayoutY(40);
        updateAttackSummaryLabel();

        worldLayer.getChildren().addAll(perimeter, patrolSectionLayer, attackLayer);
        worldLayer.getChildren().addAll(perimeterLabels);
        root.getChildren().add(worldLayer);
        root.getChildren().addAll(tickLabel, attackSummaryLabel);
        root.getChildren().add(createPlaybackControls());
        root.getChildren().add(createViewportControls());

        // Phase 7: Render all charging stations
        for (ChargingStation station : world.getChargingStations()) {
            ChargingStationVisualNode stationNode = new ChargingStationVisualNode(station, scaleMapper, PERIMETER_Y);
            stationNodes.put(station, stationNode);
            worldLayer.getChildren().add(stationNode);
        }

        // Render all robots
        for (Bot bot : world.getBots()) {
            RobotVisualNode robotNode = new RobotVisualNode(bot, scaleMapper, PERIMETER_Y);
            robotNode.updatePosition();
            robotNodes.put(bot, robotNode);
            worldLayer.getChildren().add(robotNode);
        }

        scene = new Scene(root, SCENE_WIDTH, SCENE_HEIGHT);
        scene.widthProperty().addListener((observable, oldWidth, newWidth) -> updateViewportLayout());
        scene.heightProperty().addListener((observable, oldHeight, newHeight) -> updateViewportLayout());
        updateViewportLayout();

        stage.setTitle("Perimeter Surveillance Simulator");
        stage.setScene(scene);
        stage.show();
        Platform.runLater(this::updateViewportLayout);
        
        // Phase 6: Start animation loop that drives simulation ticks
        startAnimationLoop();
    }

    /**
     * Phase 6 (Refactored): Start animation timer that drives simulation ticks.
     * 
     * The timer controls the simulation tick rate:
     * - Calls world.tick(currentTick) to advance simulation
     * - Updates robot visuals after each tick
     * - Runs at ~60 FPS or controlled animation speed
     * 
     * This synchronizes visualization with simulation for real-time animation.
     */
    private void startAnimationLoop() {
        animationTimer = new AnimationTimer() {
            private long lastUpdate = 0;
            private static final long UPDATE_INTERVAL_NANOS = 16_666_667; // ~60 FPS

            @Override
            public void handle(long now) {
                // Rate limit to ~60 FPS to avoid excessive redraws
                if (now - lastUpdate < UPDATE_INTERVAL_NANOS) {
                    return;
                }
                lastUpdate = now;

                if (!paused) {
                    advanceSimulation();
                }
            }
        };
        
        animationTimer.start();
    }

    private HBox createPlaybackControls() {
        pauseButton = new Button("Pause");
        resumeButton = new Button("Resume");
        stepButton = new Button("Step");

        pauseButton.setOnAction(event -> {
            paused = true;
            updatePlaybackControls();
        });
        resumeButton.setOnAction(event -> {
            paused = false;
            updatePlaybackControls();
        });
        stepButton.setOnAction(event -> {
            paused = true;
            advanceSimulation();
        });

        playbackControls = new HBox(8, pauseButton, resumeButton, stepButton);
        playbackControls.setLayoutY(15);
        updatePlaybackControls();
        return playbackControls;
    }

    private VBox createViewportControls() {
        Button zoomInButton = new Button("Zoom +");
        Button zoomOutButton = new Button("Zoom -");
        Button resetViewButton = new Button("Reset view");
        Button panLeftButton = new Button("←");
        Button panUpButton = new Button("↑");
        Button panDownButton = new Button("↓");
        Button panRightButton = new Button("→");

        zoomInButton.setOnAction(event -> changeZoom(ZOOM_FACTOR));
        zoomOutButton.setOnAction(event -> changeZoom(1 / ZOOM_FACTOR));
        resetViewButton.setOnAction(event -> resetViewport());
        panLeftButton.setOnAction(event -> panViewport(-PAN_STEP, 0));
        panUpButton.setOnAction(event -> panViewport(0, -PAN_STEP));
        panDownButton.setOnAction(event -> panViewport(0, PAN_STEP));
        panRightButton.setOnAction(event -> panViewport(PAN_STEP, 0));

        viewportControls = new VBox(
                6,
                new HBox(6, zoomInButton, zoomOutButton, resetViewButton),
                new HBox(6, panLeftButton, panUpButton, panDownButton, panRightButton)
        );
        viewportControls.setLayoutY(50);
        return viewportControls;
    }

    private void changeZoom(double factor) {
        zoom = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, zoom * factor));
        updateViewportTransform();
    }

    private void resetViewport() {
        zoom = 1.0;
        panX = 0;
        panY = 0;
        updateViewportTransform();
    }

    private void panViewport(double deltaX, double deltaY) {
        panX += deltaX;
        panY += deltaY;
        updateViewportTransform();
    }

    /** Fit the world to the current window and keep its center fixed while zooming. */
    private void updateViewportLayout() {
        if (scene == null) {
            return;
        }

        double sceneWidth = scene.getWidth();
        double sceneHeight = scene.getHeight();
        double fitScale = Math.min(sceneWidth / SCENE_WIDTH, sceneHeight / SCENE_HEIGHT);
        double effectiveScale = fitScale * zoom;

        worldScale.setX(effectiveScale);
        worldScale.setY(effectiveScale);
        worldTranslation.setX(sceneWidth / 2 - effectiveScale * SCENE_WIDTH / 2 + panX);
        worldTranslation.setY(sceneHeight / 2 - effectiveScale * SCENE_HEIGHT / 2 + panY);

        if (playbackControls != null) {
            playbackControls.setLayoutX(sceneWidth - playbackControls.prefWidth(-1) - 15);
        }
        if (viewportControls != null) {
            viewportControls.setLayoutX(sceneWidth - viewportControls.prefWidth(-1) - 15);
        }
    }

    private void updateViewportTransform() {
        updateViewportLayout();
    }

    /** Advance exactly one simulation tick and then refresh every visual element. */
    private void advanceSimulation() {
        if (currentTick >= duration) {
            paused = true;
            updatePlaybackControls();
            return;
        }

        world.tick(currentTick);
        currentTick++;
        refreshVisuals();

        if (currentTick >= duration) {
            paused = true;
            animationTimer.stop();
        }
        updatePlaybackControls();
    }

    private void refreshVisuals() {
        updateTickLabel();
        updatePatrolSectionVisuals();
        updateAttackVisuals();
        updateAttackSummaryLabel();

        for (Bot bot : world.getBots()) {
            RobotVisualNode robotNode = robotNodes.get(bot);
            if (robotNode != null) {
                robotNode.updateFrame();
            }
        }

        for (ChargingStation station : world.getChargingStations()) {
            ChargingStationVisualNode stationNode = stationNodes.get(station);
            if (stationNode != null) {
                stationNode.updateFrame();
            }
        }
    }

    private void updatePlaybackControls() {
        boolean complete = currentTick >= duration;
        pauseButton.setDisable(paused || complete);
        resumeButton.setDisable(!paused || complete);
        stepButton.setDisable(!paused || complete);
    }

    private void updateTickLabel() {
        tickLabel.setText("Tick: " + currentTick + " / " + duration);
    }

    private void updateAttackSummaryLabel() {
        List<Attack> attacks = world.getPerimeter().streamAttacks().toList();
        long intercepted = attacks.stream()
                .filter(attack -> attack.getStatus() == AttackStatus.INTERCEPTED)
                .count();
        double successRate = attacks.isEmpty() ? 0.0 : intercepted * 100.0 / attacks.size();

        attackSummaryLabel.setText(String.format(
                "Intercepted: %d / %d (Success rate: %.1f%%)",
                intercepted,
                attacks.size(),
                successRate
        ));
    }

    /** Show a robot's patrol section only while it is actively patrolling it. */
    private void updatePatrolSectionVisuals() {
        for (Bot bot : world.getBots()) {
            PatrolSection patrolSection = bot.getPlanningModule().getPatrolModel().getPatrolSection();
            PatrolSectionVisualNode sectionNode = patrolSectionNodes.get(bot);

            if (bot.getState() == BotState.PATROLLING && patrolSection != null) {
                if (sectionNode == null) {
                    sectionNode = new PatrolSectionVisualNode(
                            patrolSection,
                            scaleMapper,
                            PERIMETER_Y,
                            getPatrolSectionColor(bot)
                    );
                    patrolSectionNodes.put(bot, sectionNode);
                    patrolSectionLayer.getChildren().add(sectionNode);
                }
                sectionNode.setVisible(true);
            } else if (sectionNode != null) {
                sectionNode.setVisible(false);
            }
        }
    }

    private Color getPatrolSectionColor(Bot bot) {
        return PATROL_SECTION_COLORS[Math.floorMod(bot.getId(), PATROL_SECTION_COLORS.length)];
    }

    /** Add marks for newly generated attacks and refresh their current status. */
    private void updateAttackVisuals() {
        world.getPerimeter().streamAttacks().forEach(attack -> {
            AttackVisualNode attackNode = attackNodes.get(attack);
            if (attackNode == null) {
                attackNode = new AttackVisualNode(attack, scaleMapper, PERIMETER_Y);
                attackNodes.put(attack, attackNode);
                attackLayer.getChildren().add(attackNode);
            }
            attackNode.updateFrame();
        });
    }

    /**
     * Stop the animation loop (called when the visualizer is closed).
     */
    public void stopAnimationLoop() {
        if (animationTimer != null) {
            animationTimer.stop();
        }
    }

    /**
     * Launch the visualizer in a background thread.
     * This allows the simulation to continue running while the visualizer window is displayed.
     * 
     * @param simulationWorld The world to visualize
     * @param simulationDuration Number of ticks to simulate
     */
    public static void launch(World simulationWorld, int simulationDuration) {
        world = simulationWorld;
        duration = simulationDuration;
        // Launch JavaFX application in a separate thread to avoid blocking
        new Thread(() -> Application.launch(SimulationVisualizer.class)).start();
    }

    public static void main(String[] args) {
        // Test launcher - creates a dummy world for testing
        World testWorld = new World(new Perimeter(1000, PerimeterType.CIRCULAR));
        launch(testWorld, 1000);
    }
}
