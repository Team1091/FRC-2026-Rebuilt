package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManualHoodSubsystem;

public class ManualHoodCommand extends Command {
    private final double leftSpeed;
    private final double rightSpeed;
    private final ManualHoodSubsystem manualHoodSubsystem;

    public ManualHoodCommand(ManualHoodSubsystem manualHoodSubsystem, double leftSpeed, double rightSpeed){
        this.manualHoodSubsystem = manualHoodSubsystem;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;

        addRequirements(manualHoodSubsystem);
    }
    @Override
    public void execute() {
        manualHoodSubsystem.setSpeed(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        manualHoodSubsystem.setSpeed(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
