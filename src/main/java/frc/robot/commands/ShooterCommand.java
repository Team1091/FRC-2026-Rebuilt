package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ShooterState;

public class ShooterCommand extends Command {

    private ShooterState shooterState;

    public ShooterCommand(ShooterState shooterState) {
        this.shooterState = shooterState;
    }
}
