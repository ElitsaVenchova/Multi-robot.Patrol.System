package bg.uni.sofia.fmi.simulator.behavior.communication.messages;

import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;

// Съобщение с информация за патрулната секция на робот.
public record PatrolSectionMessage(
        long senderId, //изпращач на съобщението
        double maxSpeed, //максимална скорост на робота
        PatrolSection patrolSection //секция за патрулиране
) {
}