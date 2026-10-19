
package bg.uni.sofia.fmi.simulator.behavior.communication;

import bg.uni.sofia.fmi.simulator.behavior.communication.messages.PatrolSectionMessage;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.CommunicationModule;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

import java.util.Optional;

// @TODO Security:
// - Authentication: удостоверяване на роботите.
// - Authorization: проверка на правата за обмен.
// - Integrity: проверка за промяна на съобщенията.
// - Encryption: криптиране на предаваните данни.
// - Replay protection: защита срещу повторно изпращане.

//Управлява установяването на логическа комуникационна връзка между роботи в симулацията.
public class CommunicationController {
    //Проверява дали два робота могат да комуникират според обхвата на техните комуникационни модули.
    //Връзката се проверява при всяко извикване и не се запазва.
    public boolean canCommunicate(Bot first, Bot second) {
        if (first == null || second == null || first == second) {
            return false;
        }

        CommunicationModule firstModule = first.getCommunicationModule();
        CommunicationModule secondModule = second.getCommunicationModule();
        if (firstModule == null || secondModule == null) {
            return false;
        }
        double distance = GeometryUtils.perimeterDistance(first.getPosition(), second.getPosition());
        double effectiveRange = Math.min(firstModule.communicationRange(), secondModule.communicationRange());

        return distance <= effectiveRange;
    }

    /**
     * Опитва да изпрати информация за патрулната секция от един робот към друг.
     * Засега се проверява само дали роботите са в обхват.
     * Не се симулират латентност, скорост или загуба на съобщения.
     *
     * @param sender роботът, който изпраща съобщението
     * @param receiver роботът, който трябва да го получи
     * @param patrolSection патрулната секция на изпращащия робот
     * @return съобщението при успешна проверка, иначе Optional.empty()
     */
    public Optional<PatrolSectionMessage> sendPatrolSection(Bot sender,
            Bot receiver, PatrolSection patrolSection) {
        if (patrolSection == null || !canCommunicate(sender, receiver)) {
            return Optional.empty();
        }
        return Optional.of(new PatrolSectionMessage(sender.getId(), sender.getMaxSpeed(),
                new PatrolSection(patrolSection.getStartPosition(), patrolSection.getEndPosition())));
    }
}
