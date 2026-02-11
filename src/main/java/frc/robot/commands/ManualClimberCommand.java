package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManualClimberSubsystem;

public class ManualClimberCommand extends Command {
    private final double speed;
    private final ManualClimberSubsystem climber;

    public ManualClimberCommand(ManualClimberSubsystem climber, double speed) {
        this.speed = speed;
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setClimberSpeed(speed);
    }

    @Override
    public void end(boolean interrupted){
        climber.setClimberSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
