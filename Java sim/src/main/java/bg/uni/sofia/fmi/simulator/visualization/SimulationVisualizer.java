package bg.uni.sofia.fmi.simulator.visualization;

import bg.uni.sofia.fmi.simulator.domain.*;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.engine.SimulationCallback;
import bg.uni.sofia.fmi.simulator.engine.Simulator;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.Group;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.shape.Shape;
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
    private static final double CIRCULAR_CENTER_X = 500;
    private static final double CIRCULAR_CENTER_Y = 320;
    private static final double CIRCULAR_RADIUS = 220;
    private static final Color[] PATROL_SECTION_COLORS = {
            Color.DODGERBLUE, Color.MEDIUMSEAGREEN, Color.DARKORANGE,
            Color.MEDIUMPURPLE, Color.DEEPSKYBLUE, Color.HOTPINK
    };
    
    // Phase 6: Animation loop members
    private Map<Bot, RobotVisualNode> robotNodes = new HashMap<>();
    // Phase 7: Charging stations
    private Map<ChargingStation, ChargingStationVisualNode> stationNodes = new HashMap<>();
    private final Map<Attack, AttackVisualNode> attackNodes = new HashMap<>();
    private final Map<Bot, PatrolSectionVisualNode> patrolSectionNodes = new HashMap<>();
    private final Group patrolSectionLayer = new Group();
    private final Group attackLayer = new Group();
    private ScaleMapper scaleMapper;
    private Label tickLabel;
    private Label attackSummaryLabel;
    private AnimationTimer animationTimer;
    private int currentTick = 0;

    public SimulationVisualizer() {
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

        root.getChildren().addAll(perimeter, patrolSectionLayer, attackLayer);
        root.getChildren().addAll(perimeterLabels);
        root.getChildren().addAll(tickLabel, attackSummaryLabel);

        // Phase 7: Render all charging stations
        for (ChargingStation station : world.getChargingStations()) {
            ChargingStationVisualNode stationNode = new ChargingStationVisualNode(station, scaleMapper, PERIMETER_Y);
            stationNodes.put(station, stationNode);
            root.getChildren().add(stationNode);
        }

        // Render all robots
        for (Bot bot : world.getBots()) {
            RobotVisualNode robotNode = new RobotVisualNode(bot, scaleMapper, PERIMETER_Y);
            robotNode.updatePosition();
            robotNodes.put(bot, robotNode);
            root.getChildren().add(robotNode);
        }

        Scene scene = new Scene(root, 1000, 600);

        stage.setTitle("Perimeter Surveillance Simulator");
        stage.setScene(scene);
        stage.show();
        
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

                // Advance simulation one tick (if not finished)
                if (currentTick < duration) {
                    world.tick(currentTick);
                    currentTick++;
                }

                updateTickLabel();
                updatePatrolSectionVisuals();
                updateAttackVisuals();
                updateAttackSummaryLabel();

                // Update all robot visuals based on current world state
                for (Bot bot : world.getBots()) {
                    RobotVisualNode robotNode = robotNodes.get(bot);
                    if (robotNode != null) {
                        robotNode.updateFrame();
                    }
                }
                
                // Phase 7: Update all charging station visuals
                for (ChargingStation station : world.getChargingStations()) {
                    ChargingStationVisualNode stationNode = stationNodes.get(station);
                    if (stationNode != null) {
                        stationNode.updateFrame();
                    }
                }
                
                // Stop animation when simulation is complete
                if (currentTick >= duration) {
                    this.stop();
                }
            }
        };
        
        animationTimer.start();
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
