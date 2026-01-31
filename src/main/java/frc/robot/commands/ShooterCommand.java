package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private ShooterState shooterState;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, ShooterState shooterState) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterState = shooterState;
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterState(shooterState);
    }


    @Override
    public boolean isFinished() {
        // TODO: This will need to be ended elsewhere?
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterState(ShooterState.IDLE);
    }

}
