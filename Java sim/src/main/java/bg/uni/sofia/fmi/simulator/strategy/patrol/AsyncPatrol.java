package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

//Патрулиране по секции като всеко робот се вдижи независимо в своята секция
public class AsyncPatrol implements PatrolModel {
    private final double deviationProbability; // опционално, определя колко силно стратегията ще използва случайността на движениеацията
    private final int maxDeviationDuration; // колко най-много ticks да продължи движението в другата посока

    private PatrolSection patrolSection;//секцията, в която ще патрулира робота
    private Direction patrolDirection;//посока на патрулирането
    private Direction movementDirection; //текуща посока на движение
    private int remainingDeviationTicks; // оставащ брой ticks за движение в другата посока

    public AsyncPatrol(PatrolConfig config) {
        this.deviationProbability = config.getDeviationProbability();
        this.maxDeviationDuration = config.getMaxDeviationDuration();
    }

    @Override
    public void initialize(Bot bot, World world) {
        // Assign patrol section based on bot ID
        int perimeterSize = world.getPerimeter().getSize();
        int numBots = world.getBots().size();
        double sectionSize = (double) perimeterSize / numBots;

        // Calculate start and end positions for this bot
        long botId = bot.getId();
        double startPos = (botId - 1) * sectionSize;
        double endPos = botId * sectionSize - 1;

        Position startPosition = new Position(startPos);
        Position endPosition = new Position(endPos);

        this.patrolSection = new PatrolSection(startPosition, endPosition);

        // Initial patrol direction
        this.patrolDirection = Direction.RIGHT;
        this.movementDirection = this.patrolDirection;
        this.remainingDeviationTicks = 0;
    }

    @Override
    public Position execute(Bot bot, World world, int currentTime) {
        //Ако стратегията няма секция за патрулиране, инициализираме патрулирането
        if(patrolSection == null) {
            initialize(bot, world);
        }
        Position goalPosition = null;//Някой от крайщатата на patrolSection като цел на робота
        //Ако роботът не е в секцията за патрулиране, то трябва да се предвижи до нея.
        if(!bot.isInPatrolSection(patrolSection)) {
            bot.setState(BotState.GOING_TO_PATROL);
            goalPosition = bot.getTargetToPatrolSection(patrolSection);
        } else {
            bot.setState(BotState.PATROLLING);
            PatrolModel.scan(bot, world, currentTime);

            goalPosition = getPatrolTarget(bot);
        }
        if (currentTime < 2500) { System.out.print(patrolSection + " to " + patrolDirection + "/" +  movementDirection);  }
        return goalPosition;
    }

    //Посока, към която да се движи робота докато патрулира.
    //Има добавена малко случайност, за да може движението да е напълно предвидимо.
    public Position getPatrolTarget(Bot bot) {
        double currentX = bot.getPosition().getX();
        double sectionStart = patrolSection.getStartPosition().getX();
        double sectionEnd = patrolSection.getEndPosition().getX();
        //Промяна на посоката на движение, когато е достигнат края на секцията
        if (currentX >= sectionEnd) {
            patrolDirection = Direction.LEFT;
            movementDirection = patrolDirection;
            remainingDeviationTicks = 0;
        } else if (currentX <= sectionStart) {
            patrolDirection = Direction.RIGHT;
            movementDirection = patrolDirection;
            remainingDeviationTicks = 0;
        }
        //Започни случайно отклонение, ако роботът в момента не е отклонен
        if (remainingDeviationTicks == 0 && RandomProvider.nextDouble() < deviationProbability) {
            movementDirection = patrolDirection.reverse();
            remainingDeviationTicks = RandomProvider.nextInt(maxDeviationDuration) + 1;
        }
        double targetX = movementDirection == Direction.RIGHT ? sectionEnd : sectionStart;
        //Приключването на движение в обратната посока
        if (remainingDeviationTicks > 0) {
            remainingDeviationTicks--;
            if (remainingDeviationTicks == 0) {
                movementDirection = patrolDirection;
            }
        }
        return new Position(targetX);
    }

    @Override
    public PatrolSection getPatrolSection() { return this.patrolSection; }
}