package bg.uni.sofia.fmi.simulator.domain;

/**
 * Описва характеристиките на комуникационния модул на робота.
 *
 * @param communicationRange максимален обхват на връзката
 * @param speed скорост на предаване на данни в bit/s
 * @param latency латентност на предаването в милисекунди
 * @param reliability вероятност за успешно предаване на съобщение [0.0, 1.0]
 */
public record CommunicationModule(
        double communicationRange,
        double speed,
        double latency,
        double reliability
) {
    public CommunicationModule {
        if (communicationRange < 0 || !Double.isFinite(communicationRange)) {
            throw new IllegalArgumentException(
                    "Communication range must be finite and non-negative");
        }

        if (speed <= 0 || !Double.isFinite(speed)) {
            throw new IllegalArgumentException(
                    "Communication speed must be finite and positive");
        }

        if (latency < 0 || !Double.isFinite(latency)) {
            throw new IllegalArgumentException(
                    "Communication latency must be finite and non-negative");
        }

        if (!Double.isFinite(reliability)
                || reliability < 0.0
                || reliability > 1.0) {
            throw new IllegalArgumentException(
                    "Communication reliability must be between 0.0 and 1.0");
        }
    }
}