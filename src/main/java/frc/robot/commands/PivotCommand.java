package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PivotCommand extends Command {

    private final PivotSubsystem pivotSubsystem;
    private final double speed;

    public PivotCommand(PivotSubsystem pivotSubsystem, double speed) {
        this.pivotSubsystem = pivotSubsystem;
        this.speed = speed;
        addRequirements(pivotSubsystem);
    }


    @Override
    public void execute() {
        pivotSubsystem.setSpeed(speed);
    }


    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
