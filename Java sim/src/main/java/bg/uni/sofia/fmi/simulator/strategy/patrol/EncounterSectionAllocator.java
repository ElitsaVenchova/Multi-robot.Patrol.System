package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Pure calculations for informational encounter sections; no world or robot mutation.
 * Circular spans use increasing, unwrapped endpoints: [80, 120] crosses the origin
 * of a ring of length 100, [0, 100] covers it, and [0, 0] is a point.
 */
public final class EncounterSectionAllocator {
    private final double perimeterLength;
    private final PerimeterType perimeterType;

    public EncounterSectionAllocator(double perimeterLength, PerimeterType perimeterType) {
        if (!Double.isFinite(perimeterLength) || perimeterLength <= 0 || !Double.isFinite(2 * perimeterLength)) {
            throw new IllegalArgumentException("Perimeter length must be finite and positive");
        }
        this.perimeterLength = perimeterLength;
        this.perimeterType = Objects.requireNonNull(perimeterType, "Perimeter type");
    }

    /** Immutable coordinates for messages and proposals, unlike mutable Position. */
    public record SectionSpan(double start, double end) {
        public SectionSpan {
            if (!Double.isFinite(start) || !Double.isFinite(end) || end < start
                    || !Double.isFinite(end - start)) {
                throw new IllegalArgumentException("Section must have finite increasing endpoints");
            }
        }

        public double length() {
            return end - start;
        }

        public PatrolSection toPatrolSection() {
            return new PatrolSection(new Position(start), new Position(end));
        }
    }

    /** The caller's argument order is retained, even when the spatial order differs. */
    public record Redistribution(SectionSpan firstSection, SectionSpan secondSection) { }

    /** IDs determine ordering only; their numerical values are never coordinates. */
    public Map<Long, SectionSpan> allocateInitial(Collection<Long> robotIds) {
        Objects.requireNonNull(robotIds, "Robot IDs");
        List<Long> sorted = robotIds.stream().sorted().toList();
        if (sorted.stream().distinct().count() != sorted.size()) {
            throw new IllegalArgumentException("Robot IDs must be unique");
        }
        Map<Long, SectionSpan> result = new LinkedHashMap<>();
        for (int index = 0; index < sorted.size(); index++) {
            double start = perimeterLength * ((double) index / sorted.size());
            double end = index + 1 == sorted.size() ? perimeterLength
                    : perimeterLength * ((double) (index + 1) / sorted.size());
            result.put(sorted.get(index), new SectionSpan(start, end));
        }
        return result;
    }

    /** Copies an existing section, including normalized circular seam crossings. */
    public SectionSpan snapshot(PatrolSection section) {
        Objects.requireNonNull(section, "Patrol section");
        double start = section.getStartPosition().getX();
        double end = section.getEndPosition().getX();
        if (!Double.isFinite(start) || !Double.isFinite(end)) {
            throw new IllegalArgumentException("Section coordinates must be finite");
        }
        if (perimeterType == PerimeterType.LINEAR) {
            return checked(new SectionSpan(Math.min(start, end), Math.max(start, end)));
        }
        double difference = end - start;
        if (!Double.isFinite(difference) || Math.abs(difference) > perimeterLength) {
            throw new IllegalArgumentException("Circular section cannot exceed one perimeter");
        }
        double length = difference == 0 ? 0 : Math.abs(difference) == perimeterLength ? perimeterLength
                : normalize(difference);
        double normalizedStart = normalize(start);
        return checked(new SectionSpan(normalizedStart, normalizedStart + length));
    }

    public Redistribution redistribute(long firstId, SectionSpan first, double firstSpeed,
            long secondId, SectionSpan second, double secondSpeed) {
        if (firstId == secondId) {
            throw new IllegalArgumentException("Encounter requires different robot IDs");
        }
        requireSpeed(firstSpeed);
        requireSpeed(secondSpeed);
        checked(first);//Проверка на секциите дали не излизат от периметъра
        checked(second);
        SectionSpan region = connectedRegion(firstId, first, secondId, second);
        double firstOffset = offset(region.start(), first.start());
        double secondOffset = offset(region.start(), second.start());
        boolean firstLeads = firstOffset < secondOffset || (firstOffset == secondOffset && firstId < secondId);
        double scale = Math.max(firstSpeed, secondSpeed);
        double firstWeight = firstSpeed / scale;
        double secondWeight = secondSpeed / scale;
        double leadingWeight = firstLeads ? firstWeight : secondWeight;
        double boundary = region.start() + region.length() * (leadingWeight / (firstWeight + secondWeight));
        boundary = Math.max(region.start(), Math.min(region.end(), boundary));
        SectionSpan leading = new SectionSpan(region.start(), boundary);
        SectionSpan trailing = new SectionSpan(boundary, region.end());
        return firstLeads ? new Redistribution(leading, trailing) : new Redistribution(trailing, leading);
    }

    private SectionSpan connectedRegion(long firstId, SectionSpan first,  long secondId, SectionSpan second) {
        if (perimeterType == PerimeterType.LINEAR) {
            return new SectionSpan(Math.min(first.start(), second.start()), Math.max(first.end(), second.end()));
        }
        double firstStart = normalize(first.start());
        double secondStart = normalize(second.start());
        double fromFirst = Math.max(first.length(),  normalize(secondStart - firstStart) + second.length());
        double fromSecond = Math.max(second.length(), normalize(firstStart - secondStart) + first.length());
        if (fromFirst >= perimeterLength && fromSecond >= perimeterLength) {
            double start = firstId < secondId ? firstStart : secondStart;
            return new SectionSpan(start, start + perimeterLength);
        }
        boolean useFirst = fromFirst < fromSecond
                || (fromFirst == fromSecond && (firstStart < secondStart
                || (firstStart == secondStart && firstId < secondId)));
        double start = useFirst ? firstStart : secondStart;
        return new SectionSpan(start, start + (useFirst ? fromFirst : fromSecond));
    }

    private SectionSpan checked(SectionSpan span) {
        Objects.requireNonNull(span, "Section span");
        if (perimeterType == PerimeterType.LINEAR) {
            if (span.start() < 0 || span.end() > perimeterLength) {
                throw new IllegalArgumentException("Linear section is outside the perimeter");
            }
        } else if (span.length() > perimeterLength) {
            throw new IllegalArgumentException("Circular section cannot exceed one perimeter");
        }
        return span;
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