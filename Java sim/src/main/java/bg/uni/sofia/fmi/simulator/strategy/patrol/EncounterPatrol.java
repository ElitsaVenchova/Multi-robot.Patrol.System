package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.behavior.communication.CommunicationController;
import bg.uni.sofia.fmi.simulator.behavior.communication.CommunicationParticipant;
import bg.uni.sofia.fmi.simulator.behavior.communication.messages.EncounterStateMessage;
import bg.uni.sofia.fmi.simulator.behavior.communication.messages.EncounterMotionMessage;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.EncounterRearmPolicy;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalLong;

/**
 * Independent patrol with informational sections and robot-local encounter handling. Initial
 * deployment alone is constrained by section endpoints; later movement is independent.
 */
public class EncounterPatrol implements PatrolModel, CommunicationParticipant {
    private final EncounterRearmPolicy encounterRearmPolicy;
    private PatrolSection patrolSection;
    private Direction patrolDirection = Direction.RIGHT;
    private boolean initialized;
    private boolean deployed;
    private boolean suspended;
    private final Map<Long, EncounterStateMessage> receivedStates = new LinkedHashMap<>();
    private Map<Long, Double> lidarObservations = Map.of();
    private final EncounterNegotiator negotiator;
    private EncounterStateMessage localEncounterState;
    private EncounterSectionAllocator.Redistribution proposedRedistribution;

    public EncounterPatrol(PatrolConfig config) {
        encounterRearmPolicy = Objects.requireNonNull(
                Objects.requireNonNull(config, "Patrol config").getEncounterRearmPolicy(),
                "Encounter rearm policy");
        negotiator = new EncounterNegotiator(encounterRearmPolicy);
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
        return planPatrolTarget(bot);
    }

    @Override
    public void onPatrolSuspended(Bot bot) {
        clearEncounterState();
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

    @Override
    public void onCommunicationRound(Bot bot, int currentTime,
            CommunicationController.Round round, CommunicationController communication) {
        List<CommunicationController.Envelope> incoming = communication.receive(bot.getId());
        // Current state alone determines eligibility; no pre-tick planning or battery prediction.
        if (bot.getState() != BotState.PATROLLING) {
            clearEncounterState();
            return;
        }
        initialize(bot);
        onPatrolResumed(bot);
        switch (round) {
            case DISCOVER -> {
                receivedStates.clear();
                proposedRedistribution = null;
                lidarObservations = communication.observeRobots(bot.getId());
                // Preserve deployment movement while still publishing the current robot's intent.
                Position target;
                if (!deployed && !bot.isInPatrolSection(patrolSection)) {
                    target = bot.getTargetToPatrolSection(patrolSection);
                } else {
                    deployed = true;
                    target = planPatrolTarget(bot);
                }
                double displacement = intendedPatrolDisplacement(bot, target);
                if (!deployed && World.getPerimeter().getType() == PerimeterType.CIRCULAR) {
                    displacement = GeometryUtils.signedPerimeterOffset(bot.getPosition(), target);
                }
                displacement = Math.copySign(Math.min(bot.getMaxSpeed(), Math.abs(displacement)), displacement);
                if (Math.abs(displacement) < 0.001) {
                    displacement = 0;
                }
                localEncounterState = new EncounterStateMessage(bot.getId(), bot.getMaxSpeed(), patrolSection);
                negotiator.beginTick(currentTime, localEncounterState, displacement, lidarObservations,
                        peerId -> communication.isWithinCommunicationRange(bot.getId(), peerId));
                // Directed discovery: never send section data to every robot in radio range.
                for (long detectedId : lidarObservations.keySet()) {
                    EncounterMotionMessage motion = new EncounterMotionMessage(displacement,
                            negotiator.getCompletedEncounterCount(), negotiator.hasObservedRangeExit(detectedId));
                    communication.send(bot.getId(), detectedId, motion);
                }
            }
            case SELECT -> {
                OptionalLong chosen = negotiator.selectPartner(incoming);
                if (chosen.isPresent()) {
                    negotiator.recordStateSend(communication.send(bot.getId(), chosen.getAsLong(),
                            localEncounterState));
                }
            }
            case APPLY -> negotiator.finish(incoming).ifPresent(pairing -> {
                EncounterSectionAllocator.Redistribution result =
                        new EncounterSectionAllocator(World.getPerimeter())
                                .redistribute(pairing.localState(), pairing.peerState());
                if (negotiator.markCommitted(pairing)) {
                    // Both use frozen input messages; no robot reads or changes its peer's state.
                    patrolSection = result.firstSection();
                    patrolDirection = pairing.reverseLocal()
                            ? pairing.localDirection().reverse() : pairing.localDirection();
                    deployed = true;
                    proposedRedistribution = result;
                    receivedStates.put(pairing.peerState().senderId(), pairing.peerState());
                }
            });
        }
    }

    private void clearEncounterState() {
        receivedStates.clear();
        lidarObservations = Map.of();
        negotiator.clear();
        localEncounterState = null;
        proposedRedistribution = null;
    }

    public OptionalLong getSelectedPartnerId() {
        return negotiator.getSelectedPartnerId();
    }

    public Optional<EncounterNegotiator.Pairing> getPairing() {
        return negotiator.getPairing();
    }

    public Optional<EncounterSectionAllocator.Redistribution> getProposedRedistribution() {
        return Optional.ofNullable(proposedRedistribution);
    }

    /** One source for advertised intent and the movement target; no remote robot access. */
    private Position planPatrolTarget(Bot bot) {
        preparePhysicalDirection(bot);
        double x = bot.getPosition().getX();
        if (World.getPerimeter().getType() == PerimeterType.LINEAR) {
            return new Position(patrolDirection == Direction.RIGHT
                    ? Math.min(World.getPerimeter().getSize(), x + bot.getMaxSpeed())
                    : Math.max(0, x - bot.getMaxSpeed()));
        }
        // Unwrapped target preserves direction and full laps.
        return new Position(GeometryUtils.normalizePosition(x)
                + (patrolDirection == Direction.RIGHT ? 1 : -1) * bot.getMaxSpeed());
    }

    /** Signed intended displacement, not maximum speed or a previous observed velocity. */
    private double intendedPatrolDisplacement(Bot bot, Position target) {
        double x = bot.getPosition().getX();
        if (World.getPerimeter().getType() == PerimeterType.CIRCULAR) {
            x = GeometryUtils.normalizePosition(x);
        }
        double displacement = target.getX() - x;
        return Math.abs(displacement) < 0.001 ? 0 : displacement;
    }

    private void preparePhysicalDirection(Bot bot) {
        if (World.getPerimeter().getType() == PerimeterType.LINEAR) {
            double x = bot.getPosition().getX();
            if (x <= 0 && patrolDirection == Direction.LEFT
                    || x >= World.getPerimeter().getSize() && patrolDirection == Direction.RIGHT) {
                patrolDirection = patrolDirection.reverse();
            }
        }
    }

    public Map<Long, EncounterStateMessage> getReceivedStates() {
        return Map.copyOf(receivedStates);
    }

    /** Signed local offsets; absolute values are LiDAR distances. */
    public Map<Long, Double> getLidarObservations() {
        return lidarObservations;
    }

    public long getCompletedEncounterCount() {
        return negotiator.getCompletedEncounterCount();
    }

    public EncounterRearmPolicy getEncounterRearmPolicy() {
        return encounterRearmPolicy;
    }
}