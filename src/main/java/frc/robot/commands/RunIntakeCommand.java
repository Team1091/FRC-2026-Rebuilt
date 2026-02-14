package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final IntakeState intakeState;

    public RunIntakeCommand(IntakeSubsystem intakeSubsystem, IntakeState intakeState) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeState = intakeState;
        addRequirements(intakeSubsystem);
    }


    @Override
    public void execute() {
        intakeSubsystem.setState(intakeState);
    }


    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setState(IntakeState.EXTENDED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
