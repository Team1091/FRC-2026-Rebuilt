package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManualShooterSubsystem;

public class SetSpeedCommand extends Command {
    private final double newSpeed;
    private final ManualShooterSubsystem manualShooterSubsystem;

    public SetSpeedCommand(double newSpeed, ManualShooterSubsystem manualShooterSubsystem) {
        this.newSpeed = newSpeed;
        this.manualShooterSubsystem = manualShooterSubsystem;
        addRequirements(manualShooterSubsystem);
    }
    public void execute() {
        manualShooterSubsystem.setSpeed(newSpeed);
    }

    public boolean isFinished() {
        return true;
    }
}
