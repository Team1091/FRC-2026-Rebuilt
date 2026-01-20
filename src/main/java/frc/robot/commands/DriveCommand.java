package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

public class DriveCommand {
    private DriveCommand() {
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier
    ) {
        return Commands.run(
                () -> {
                    double x = xSupplier.getAsDouble();
                    double y = ySupplier.getAsDouble();
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Swerve.rotationalDeadband);

                    // Apply deadband
                    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.Swerve.linearDeadband);
                    Rotation2d linearDirection = new Rotation2d(-x, y);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calculate new linear velocity
                    Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);

                    drive.runVelocity(linearVelocity, omega);
                },
                drive
        );
    }
}