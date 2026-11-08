package bg.uni.sofia.fmi.simulator.behavior.communication;

import bg.uni.sofia.fmi.simulator.behavior.communication.messages.CommunicationMessage;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.CommunicationModule;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;

/**
 * Simulation transport only: frozen range observations and next-round delivery.
 * No partner selection, availability decisions, reservations, or allocation.
 * Delivery is idealized; configured latency, bandwidth, and loss are not simulated.
 */
public class CommunicationController {
    // TODO Security: authentication, authorization, integrity, encryption and replay protection.
    public enum Round {
        DISCOVER, SELECT, APPLY
    }

    public record Envelope(long senderId, long receiverId, int tick, Round sentRound,
                           CommunicationMessage message) { }

    /** Physical capabilities only; deliberately contains no patrol or availability state. */
    private record Endpoint(double x, double communicationRange, double lidarRange) { }

    private final Map<Long, Endpoint> endpoints = new TreeMap<>();
    private final Map<Long, List<Envelope>> inboxes = new HashMap<>();
    private final List<Envelope> pending = new ArrayList<>();
    private int tick;
    private Round round;
    private boolean active;

    public void beginTick(List<Bot> bots, int currentTime) {
        if (active) {
            throw new IllegalStateException("Previous communication tick is still active");
        }
        endpoints.clear();
        inboxes.clear();
        pending.clear();
        tick = currentTime;
        round = null;
        for (Bot bot : bots) {
            CommunicationModule module = bot.getCommunicationModule();
            Endpoint endpoint = new Endpoint(bot.getPosition().getX(),
                    module == null ? -1 : module.communicationRange(),
                    bot.getLidar() == null ? -1 : bot.getLidar().range());
            if (endpoints.putIfAbsent(bot.getId(), endpoint) != null) {
                throw new IllegalArgumentException("Robot IDs must be unique");
            }
        }
        active = true;
    }

    /** Messages sent during a round are never visible to another robot in that round. */
    public void beginRound(Round next) {
        Objects.requireNonNull(next, "Communication round");
        if (!active || next.ordinal() != (round == null ? 0 : round.ordinal() + 1)) {
            throw new IllegalStateException("Communication rounds must advance in order");
        }
        inboxes.clear();
        pending.sort(Comparator.comparingLong(Envelope::senderId).thenComparingLong(Envelope::receiverId));
        for (Envelope message : pending) {
            if (message.tick() == tick && isWithinCommunicationRange(message.senderId(), message.receiverId())) {
                inboxes.computeIfAbsent(message.receiverId(), ignored -> new ArrayList<>()).add(message);
            }
        }
        pending.clear();
        round = next;
    }

    public void endTick() {
        endpoints.clear();
        inboxes.clear();
        pending.clear();
        round = null;
        active = false;
    }

    /** Broadcasting discovers peers by physical radio range, without choosing a partner. */
    public void broadcast(long senderId, CommunicationMessage message) {
        requireSendingRound();
        Objects.requireNonNull(message, "Message");
        for (long receiverId : endpoints.keySet()) {
            send(senderId, receiverId, message);
        }
    }

    public boolean send(long senderId, long receiverId, CommunicationMessage message) {
        requireSendingRound();
        Objects.requireNonNull(message, "Message");
        if (!isWithinCommunicationRange(senderId, receiverId)) {
            return false;
        }
        pending.add(new Envelope(senderId, receiverId, tick, round, message));
        return true;
    }

    public List<Envelope> receive(long receiverId) {
        requireRound();
        List<Envelope> messages = inboxes.remove(receiverId);
        return messages == null ? List.of() : List.copyOf(messages);
    }

    /**
     * Local signed LiDAR offsets, not remote positions: positive is RIGHT, negative LEFT.
     * The absolute value is measured perimeter distance. Detection-to-ID association is
     * an explicit simulator abstraction, without recognition, occlusion or field of view.
     * A local observation alone does not establish mutual detection.
     */
    public Map<Long, Double> observeRobots(long observerId) {
        requireRound();
        Endpoint observer = endpoints.get(observerId);
        if (observer == null || !Double.isFinite(observer.lidarRange()) || observer.lidarRange() < 0) {
            return Map.of();
        }
        Map<Long, Double> observations = new LinkedHashMap<>();
        for (Map.Entry<Long, Endpoint> entry : endpoints.entrySet()) {
            if (entry.getKey() != observerId) {
                double offset = GeometryUtils.signedPerimeterOffset(
                        new Position(observer.x()), new Position(entry.getValue().x()));
                if (Math.abs(offset) <= observer.lidarRange()) {
                    observations.put(entry.getKey(), offset);
                }
            }
        }
        return Map.copyOf(observations);
    }

    /** Physical radio reachability in the frozen snapshot; contains no patrol/availability rule. */
    public boolean isWithinCommunicationRange(long firstId, long secondId) {
        if (!active) {
            throw new IllegalStateException("No active communication tick");
        }
        Endpoint first = endpoints.get(firstId);
        Endpoint second = endpoints.get(secondId);
        return firstId != secondId && first != null && second != null
                && first.communicationRange() >= 0 && second.communicationRange() >= 0
                && distance(first, second) <= Math.min(first.communicationRange(), second.communicationRange());
    }

    private double distance(Endpoint first, Endpoint second) {
        return GeometryUtils.perimeterDistance(new Position(first.x()), new Position(second.x()));
    }

    private void requireRound() {
        if (!active || round == null) {
            throw new IllegalStateException("No active communication round");
        }
    }

    /** APPLY receives reciprocal section data; it cannot rely on another delivery round. */
    private void requireSendingRound() {
        requireRound();
        if (round == Round.APPLY) {
            throw new IllegalStateException("APPLY is receive-only; send in an earlier round");
        }
    }
}