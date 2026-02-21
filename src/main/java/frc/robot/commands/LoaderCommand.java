package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoaderSubsystem;

public class LoaderCommand extends Command {
    private final double speed;
    private final LoaderSubsystem loaderSubsystem;

    public LoaderCommand(LoaderSubsystem loaderSubsystem, double speed) {
        this.speed = speed;
        this.loaderSubsystem = loaderSubsystem;
        addRequirements(loaderSubsystem);
    }

    @Override
    public void execute() {
        loaderSubsystem.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}