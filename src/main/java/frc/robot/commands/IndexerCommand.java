package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command {
    private final double speed;
    private final IndexerSubsystem indexerSubsystem;

    public IndexerCommand(IndexerSubsystem indexerSubsystem, double speed) {
        this.speed = speed;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}