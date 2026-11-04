package bg.uni.sofia.fmi.simulator.domain.enums;

/** Controls when a previously completed encounter may be repeated. */
// Типа на политиката за това кога бот може отново да обмени информация (да преразпределят периметъра) с друг участник
public enum EncounterRearmPolicy {
    EXIT_EITHER_RANGE, //След излизане от визуалния и комуникациония обсег
    AFTER_OTHER_ENCOUNTER //след среща и договаряне с друг бот
}