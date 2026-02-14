package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class AutoClimbCommand extends Command {
    private final double speed;
    private final ClimberSubsystem climber;

    public AutoClimbCommand(ClimberSubsystem climber, double speed) {
        this.speed = speed;
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setClimberSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}