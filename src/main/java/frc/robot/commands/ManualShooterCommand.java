package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManualShooterSubsystem;

public class ManualShooterCommand extends Command {
    private final double speed;
    private final ManualShooterSubsystem manualShooterSubsystem;

    public ManualShooterCommand(ManualShooterSubsystem manualShooterSubsystem, double speed) {
        this.speed = speed;
        this.manualShooterSubsystem = manualShooterSubsystem;
        addRequirements(manualShooterSubsystem);
    }

    @Override
    public void execute() {
        manualShooterSubsystem.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}