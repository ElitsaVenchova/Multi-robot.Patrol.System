package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.EncounterRearmPolicy;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

import java.util.List;
import java.util.Objects;

/**
 * Independent patrol with informational sections. Encounter negotiation is added
 * separately; section endpoints only constrain initial deployment.
 */
public class EncounterPatrol implements PatrolModel {
    private final EncounterRearmPolicy encounterRearmPolicy;
    private PatrolSection patrolSection;
    private Direction patrolDirection = Direction.RIGHT;
    private boolean initialized;
    private boolean deployed;
    private boolean suspended;

    public EncounterPatrol(PatrolConfig config) {
        encounterRearmPolicy = Objects.requireNonNull(
                Objects.requireNonNull(config, "Patrol config").getEncounterRearmPolicy(),
                "Encounter rearm policy");
    }

    @Override
    public void initialize(Bot bot) {
        if (initialized) {
            return;
        }
        if (World.getPerimeter().getSize() <= 0) {
            throw new IllegalArgumentException("Perimeter length must be positive");
        }
        List<Bot> participants = World.getBots().stream()
                .filter(candidate -> candidate.getPlanningModule() != null
                        && candidate.getPlanningModule().getPatrolModel() instanceof EncounterPatrol)
                .toList();
        for (Bot participant : participants) {
            if (!Double.isFinite(participant.getMaxSpeed()) || participant.getMaxSpeed() <= 0) {
                throw new IllegalArgumentException("Maximum speed must be finite and positive");
            }
        }
        List<Long> ids = participants.stream().map(Bot::getId).toList();
        if (ids.stream().distinct().count() != ids.size()) {
            throw new IllegalArgumentException("Robot IDs must be unique");
        }
        PatrolSection section =
                new EncounterSectionAllocator(World.getPerimeter()).allocateInitial(ids).get(bot.getId());
        if (section == null) {
            throw new IllegalArgumentException("Robot must be an EncounterPatrol participant");
        }
        patrolSection = section;
        initialized = true;
    }

    @Override
    public Position execute(Bot bot, int currentTime) {
        initialize(bot);
        if (bot.getState() == BotState.ERROR) {
            onPatrolSuspended(bot);
            return null;
        }
        onPatrolResumed(bot);
        if (!deployed) {
            if (!bot.isInPatrolSection(patrolSection)) {
                bot.setState(BotState.GOING_TO_PATROL);
                return bot.getTargetToPatrolSection(patrolSection);
            }
            deployed = true;
        }
        bot.setState(BotState.PATROLLING);
        if (bot.getLidar() != null) {
            PatrolModel.scan(bot, currentTime);
        }
        double currentX = bot.getPosition().getX();
        if (World.getPerimeter().getType() == PerimeterType.LINEAR) {
            if (currentX <= 0 && patrolDirection == Direction.LEFT
                    || currentX >= World.getPerimeter().getSize() && patrolDirection == Direction.RIGHT) {
                patrolDirection = patrolDirection.reverse();
            }
            return new Position(patrolDirection == Direction.RIGHT
                    ? Math.min(World.getPerimeter().getSize(), currentX + bot.getMaxSpeed())
                    : Math.max(0, currentX - bot.getMaxSpeed()));
        }
        // Keep the target unwrapped so a complete lap is distinguishable from no movement.
        return new Position(GeometryUtils.normalizePosition(currentX)
                + (patrolDirection == Direction.RIGHT ? 1 : -1) * bot.getMaxSpeed());
    }

    @Override
    public void onPatrolSuspended(Bot bot) {
        suspended = true;
        patrolSection = null;
    }

    @Override
    public void onPatrolResumed(Bot bot) {
        if (suspended && bot.getState() != BotState.ERROR) {
            double x = bot.getPosition().getX();
            if (World.getPerimeter().getType() == PerimeterType.CIRCULAR) {
                x = GeometryUtils.normalizePosition(x);
            }
            patrolSection = new PatrolSection(new Position(x), new Position(x));
            deployed = true;
            suspended = false;
        }
    }

    @Override
    public Direction getMovementDirection() {
        return deployed && !suspended ? patrolDirection : null;
    }

    @Override
    public PatrolSection getPatrolSection() {
        return patrolSection;
    }

    public EncounterRearmPolicy getEncounterRearmPolicy() {
        return encounterRearmPolicy;
    }
}