package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.behavior.communication.CommunicationController.Envelope;
import bg.uni.sofia.fmi.simulator.behavior.communication.CommunicationController.Round;
import bg.uni.sofia.fmi.simulator.behavior.communication.messages.EncounterMotionMessage;
import bg.uni.sofia.fmi.simulator.behavior.communication.messages.EncounterStateMessage;
import bg.uni.sofia.fmi.simulator.domain.enums.EncounterRearmPolicy;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalLong;
import java.util.Set;
import java.util.function.LongPredicate;

/**
 * Robot-local, tick-bounded pairing and encounter history. No live robots, world queries,
 * shared assignments, locks, or blocking waits. Reciprocal section data establishes a pair
 * under the simulator's reliable delivery and frozen-observation assumptions.
 */
public final class EncounterNegotiator {
    public enum Kind { HEAD_ON, CATCHING_UP }

    public record Pairing(int tick, EncounterStateMessage localState, EncounterStateMessage peerState,
                          Kind kind, boolean reverseLocal, Direction localDirection) { }

    private record Candidate(long id, double distance, Kind kind, boolean reverseLocal,
                             long peerEncounterCount) { }

    private record EncounterHistory(long localCountAtCommit, long peerCountAtCommit,
                                    boolean rangeExitObserved) { }

    private final EncounterRearmPolicy rearmPolicy;
    private final Map<Long, EncounterHistory> history = new HashMap<>();
    private long completedEncounterCount;
    private Integer lastCommittedTick;

    private int tick;
    private EncounterStateMessage localState;
    private double localDisplacement;
    private Map<Long, Double> observations = Map.of();
    private Candidate selected;
    private boolean selectionProcessed;
    private boolean stateSent;
    private boolean completed;
    private Pairing pairing;
    private Long failedPartner;
    private int failedTick;

    public EncounterNegotiator(EncounterRearmPolicy rearmPolicy) {
        this.rearmPolicy = Objects.requireNonNull(rearmPolicy, "Encounter rearm policy");
    }

    public void beginTick(int tick, EncounterStateMessage localState, double localDisplacement,
                          Map<Long, Double> observations, LongPredicate canCommunicate) {
        this.tick = tick;
        this.localState = Objects.requireNonNull(localState, "Local encounter state");
        this.localDisplacement = localDisplacement;
        this.observations = Map.copyOf(observations);
        if (rearmPolicy == EncounterRearmPolicy.EXIT_EITHER_RANGE) {
            // A missing local detection or a failed physical radio check proves a range exit.
            // Peer silence alone does not: it may simply be charging or choosing another robot.
            history.replaceAll((peerId, previous) ->
                    !observations.containsKey(peerId) || !canCommunicate.test(peerId)
                            ? new EncounterHistory(previous.localCountAtCommit(),
                                    previous.peerCountAtCommit(), true)
                            : previous);
        }
        selected = null;
        selectionProcessed = false;
        stateSent = false;
        completed = false;
        pairing = null;
    }

    /** One choice, independent of message arrival order; identity is only the final tie-breaker. */
    public OptionalLong selectPartner(List<Envelope> incoming) {
        if (localState == null || selectionProcessed || committedThisTick()) {
            return OptionalLong.empty();
        }
        selectionProcessed = true;
        Map<Long, EncounterMotionMessage> motions = new HashMap<>();
        Set<Long> conflicting = new HashSet<>();
        for (Envelope envelope : incoming) {
            if (isCurrent(envelope, Round.DISCOVER)
                    && observations.containsKey(envelope.senderId())
                    && envelope.message() instanceof EncounterMotionMessage motion) {
                EncounterMotionMessage previous = motions.putIfAbsent(envelope.senderId(), motion);
                if (previous != null && !previous.equals(motion)) {
                    conflicting.add(envelope.senderId());
                }
            }
        }
        selected = motions.entrySet().stream()
                .filter(entry -> !conflicting.contains(entry.getKey()))
                .filter(entry -> isArmed(entry.getKey(), entry.getValue()))
                .filter(entry -> failedPartner == null || entry.getKey().longValue() != failedPartner
                        || (long) tick != (long) failedTick + 1)
                .map(entry -> classify(entry.getKey(), observations.get(entry.getKey()), entry.getValue()))
                .flatMap(Optional::stream)
                .min(Comparator.comparingInt((Candidate candidate) -> candidate.kind().ordinal())
                        .thenComparingDouble(Candidate::distance).thenComparingLong(Candidate::id))
                .orElse(null);
        return selected == null ? OptionalLong.empty() : OptionalLong.of(selected.id());
    }

    private boolean isArmed(long peerId, EncounterMotionMessage peer) {
        EncounterHistory previous = history.get(peerId);
        if (previous == null) {
            return true;
        }
        if (peer.completedEncounterCount() < previous.peerCountAtCommit()) {
            return false;
        }
        return switch (rearmPolicy) {
            case EXIT_EITHER_RANGE -> previous.rangeExitObserved() || peer.rangeExitObserved();
            case AFTER_OTHER_ENCOUNTER -> completedEncounterCount > previous.localCountAtCommit()
                    || peer.completedEncounterCount() > previous.peerCountAtCommit();
        };
    }

    /** A failed send cannot create a pair merely because an unsolicited section arrives. */
    public void recordStateSend(boolean sent) {
        if (selected != null && !completed) {
            stateSent = sent;
        }
    }

    public Optional<Pairing> finish(List<Envelope> incoming) {
        if (localState == null || completed || committedThisTick()) {
            return Optional.empty();
        }
        completed = true;
        if (selected != null && stateSent) {
            for (Envelope envelope : incoming) {
                if (isCurrent(envelope, Round.SELECT) && envelope.senderId() == selected.id()
                        && envelope.message() instanceof EncounterStateMessage peer
                        && peer.senderId() == selected.id()) {
                    Direction heading = localDisplacement > 0 ? Direction.RIGHT : Direction.LEFT;
                    pairing = new Pairing(tick, localState, peer, selected.kind(),
                            selected.reverseLocal(), heading);
                    failedPartner = null;
                    return Optional.of(pairing);
                }
            }
        }
        if (selected != null) {
            // No reciprocal data: retry other candidates on the next tick, without stopping motion.
            failedPartner = selected.id();
            failedTick = tick;
        }
        return Optional.empty();
    }

    /**
     * Records only an actually applied local encounter. Frozen rounds and reciprocal selection
     * mean the peer commits the same pair, so its post-commit counter is the advertised one + 1.
     * No message is needed after APPLY. Loss/failure during commit is outside this transport model.
     */
    public boolean markCommitted(Pairing result) {
        if (result == null || result != pairing || result.tick() != tick || committedThisTick()) {
            return false;
        }
        long ownAfter = Math.incrementExact(completedEncounterCount);
        long peerAfter = Math.incrementExact(selected.peerEncounterCount());
        history.put(result.peerState().senderId(), new EncounterHistory(ownAfter, peerAfter, false));
        completedEncounterCount = ownAfter;
        lastCommittedTick = tick;
        return true;
    }

    private boolean committedThisTick() {
        return lastCommittedTick != null && lastCommittedTick == tick;
    }

    private boolean isCurrent(Envelope envelope, Round expectedRound) {
        return envelope.tick() == tick && envelope.sentRound() == expectedRound
                && envelope.receiverId() == localState.senderId()
                && envelope.senderId() != localState.senderId();
    }

    private Optional<Candidate> classify(long id, double offset, EncounterMotionMessage peer) {
        double own = localDisplacement;
        double peerDisplacement = peer.intendedDisplacement();
        if (own == 0 || peerDisplacement == 0) {
            return Optional.empty();
        }
        boolean opposite = Math.signum(own) != Math.signum(peerDisplacement);
        if (opposite && (offset == 0
                || Math.signum(offset) == Math.signum(own)
                && Math.signum(offset) != Math.signum(peerDisplacement))) {
            return Optional.of(new Candidate(id, Math.abs(offset), Kind.HEAD_ON, true,
                    peer.completedEncounterCount()));
        }
        if (!opposite && Math.abs(own) != Math.abs(peerDisplacement)
                && (offset == 0 || Math.signum(own - peerDisplacement) == Math.signum(offset))) {
            return Optional.of(new Candidate(id, Math.abs(offset), Kind.CATCHING_UP,
                    Math.abs(own) > Math.abs(peerDisplacement), peer.completedEncounterCount()));
        }
        return Optional.empty();
    }

    public long getCompletedEncounterCount() {
        return completedEncounterCount;
    }

    public boolean hasObservedRangeExit(long peerId) {
        EncounterHistory previous = history.get(peerId);
        return previous != null && previous.rangeExitObserved();
    }

    public OptionalLong getSelectedPartnerId() {
        return selected == null ? OptionalLong.empty() : OptionalLong.of(selected.id());
    }

    public Optional<Pairing> getPairing() {
        return Optional.ofNullable(pairing);
    }

    /** Departure invalidates negotiations but preserves completed history and the tick guard. */
    public void clear() {
        localState = null;
        observations = Map.of();
        selected = null;
        pairing = null;
        selectionProcessed = false;
        stateSent = false;
        completed = false;
        failedPartner = null;
    }
}