package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.behavior.communication.messages.EncounterStateMessage;
import bg.uni.sofia.fmi.simulator.domain.Perimeter;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Pure allocation of informational sections from locally available state snapshots. */
public final class EncounterSectionAllocator {
    private final double perimeterLength;
    private final PerimeterType perimeterType;

    public EncounterSectionAllocator(Perimeter perimeter) {
        Objects.requireNonNull(perimeter, "Perimeter");
        if (perimeter.getSize() <= 0) {
            throw new IllegalArgumentException("Perimeter length must be positive");
        }
        perimeterLength = perimeter.getSize();
        perimeterType = Objects.requireNonNull(perimeter.getType(), "Perimeter type");
    }

    /** Explicit IDs retain association even when spatial order or argument order differs. */
    public record Redistribution(long firstId, PatrolSection firstSection,
                                 long secondId, PatrolSection secondSection) { }

    /** IDs determine ordering only; their numerical values are never coordinates. */
    public Map<Long, PatrolSection> allocateInitial(Collection<Long> robotIds) {
        Objects.requireNonNull(robotIds, "Robot IDs");
        List<Long> sorted = robotIds.stream().sorted().toList();
        if (sorted.stream().distinct().count() != sorted.size()) {
            throw new IllegalArgumentException("Robot IDs must be unique");
        }
        Map<Long, PatrolSection> result = new LinkedHashMap<>();
        for (int index = 0; index < sorted.size(); index++) {
            double start = perimeterLength * ((double) index / sorted.size());
            double end = index + 1 == sorted.size() ? perimeterLength
                    : perimeterLength * ((double) (index + 1) / sorted.size());
            result.put(sorted.get(index), section(start, end));
        }
        return result;
    }

    /** Does not inspect live robots, transport state, or the current simulation world. */
    public Redistribution redistribute(EncounterStateMessage first, EncounterStateMessage second) {
        Objects.requireNonNull(first, "First robot state");
        Objects.requireNonNull(second, "Second robot state");
        if (first.senderId() == second.senderId()) {
            throw new IllegalArgumentException("Encounter requires different robot IDs");
        }
        requireSpeed(first.maxSpeed());
        requireSpeed(second.maxSpeed());
        PatrolSection firstSection = canonicalSection(first.patrolSection());
        PatrolSection secondSection = canonicalSection(second.patrolSection());
        PatrolSection region = connectedRegion(first.senderId(), firstSection,
                second.senderId(), secondSection);
        double start = region.startPosition().getX();
        double end = region.endPosition().getX();
        double firstOffset = offset(start, firstSection.startPosition().getX());
        double secondOffset = offset(start, secondSection.startPosition().getX());
        boolean firstLeads = firstOffset < secondOffset
                || (firstOffset == secondOffset && first.senderId() < second.senderId());
        double scale = Math.max(first.maxSpeed(), second.maxSpeed());
        double firstWeight = first.maxSpeed() / scale;
        double secondWeight = second.maxSpeed() / scale;
        double leadingWeight = firstLeads ? firstWeight : secondWeight;
        double boundary = start + length(region) * (leadingWeight / (firstWeight + secondWeight));
        boundary = Math.max(start, Math.min(end, boundary));
        PatrolSection leading = section(start, boundary);
        PatrolSection trailing = section(boundary, end);
        return new Redistribution(first.senderId(), firstLeads ? leading : trailing,
                second.senderId(), firstLeads ? trailing : leading);
    }

    /** Normalizes geometry internally without introducing a second section representation. */
    private PatrolSection canonicalSection(PatrolSection input) {
        double start = input.startPosition().getX();
        double end = input.endPosition().getX();
        if (!Double.isFinite(start) || !Double.isFinite(end)) {
            throw new IllegalArgumentException("Section coordinates must be finite");
        }
        if (perimeterType == PerimeterType.LINEAR) {
            double min = Math.min(start, end);
            double max = Math.max(start, end);
            if (min < 0 || max > perimeterLength) {
                throw new IllegalArgumentException("Linear section is outside the perimeter");
            }
            return section(min, max);
        }
        double difference = end - start;
        double tolerance = 4 * Math.ulp(Math.max(perimeterLength, Math.max(Math.abs(start), Math.abs(end))));
        if (!Double.isFinite(difference) || Math.abs(difference) > perimeterLength + tolerance) {
            throw new IllegalArgumentException("Circular section cannot exceed one perimeter");
        }
        double arcLength = Math.abs(difference) >= perimeterLength ? perimeterLength
                : difference >= 0 ? difference : normalize(difference);
        double normalizedStart = normalize(start);
        return section(normalizedStart, normalizedStart + arcLength);
    }

    private PatrolSection connectedRegion(long firstId, PatrolSection first,
                                         long secondId, PatrolSection second) {
        if (perimeterType == PerimeterType.LINEAR) {
            return section(Math.min(first.startPosition().getX(), second.startPosition().getX()),
                    Math.max(first.endPosition().getX(), second.endPosition().getX()));
        }
        double firstStart = normalize(first.startPosition().getX());
        double secondStart = normalize(second.startPosition().getX());
        double fromFirst = Math.max(length(first), normalize(secondStart - firstStart) + length(second));
        double fromSecond = Math.max(length(second), normalize(firstStart - secondStart) + length(first));
        if (fromFirst >= perimeterLength && fromSecond >= perimeterLength) {
            double start = firstId < secondId ? firstStart : secondStart;
            return section(start, start + perimeterLength);
        }
        boolean useFirst = fromFirst < fromSecond
                || (fromFirst == fromSecond && (firstStart < secondStart
                || (firstStart == secondStart && firstId < secondId)));
        double start = useFirst ? firstStart : secondStart;
        return section(start, start + (useFirst ? fromFirst : fromSecond));
    }

    /** Called only for canonical increasing sections, with roundoff capped at one ring. */
    private double length(PatrolSection section) {
        return Math.min(perimeterLength, section.endPosition().getX() - section.startPosition().getX());
    }

    private static PatrolSection section(double start, double end) {
        return new PatrolSection(new Position(start), new Position(end));
    }

    private double offset(double start, double position) {
        return perimeterType == PerimeterType.CIRCULAR
                ? normalize(position - start) : position - start;
    }

    private double normalize(double position) {
        return GeometryUtils.normalizePosition(position, perimeterLength);
    }

    private static void requireSpeed(double speed) {
        if (!Double.isFinite(speed) || speed <= 0) {
            throw new IllegalArgumentException("Maximum speed must be finite and positive");
        }
    }
}