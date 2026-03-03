package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManualHoodSubsystem;

public class ManualHoodCommand extends Command {
    private final double hoodSpeed;
    private final ManualHoodSubsystem manualHoodSubsystem;

    public ManualHoodCommand(ManualHoodSubsystem manualHoodSubsystem, double hoodSpeed) {
        this.manualHoodSubsystem = manualHoodSubsystem;
        this.hoodSpeed = hoodSpeed;

        addRequirements(manualHoodSubsystem);
    }

    @Override
    public void execute() {
        manualHoodSubsystem.setSpeed(hoodSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        manualHoodSubsystem.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
