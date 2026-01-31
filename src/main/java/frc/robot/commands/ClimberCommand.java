package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ClimberPosition;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    private ClimberPosition target;
    private ClimberSubsystem climber;

    public ClimberCommand(ClimberSubsystem climber, ClimberPosition target) {
        this.target = target;
        this.climber = climber;
    }

    @Override
    public void execute() {
        climber.setClimberPosition(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
