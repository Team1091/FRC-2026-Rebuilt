// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public final class Autos {

    // Spin wildly for a second.
    public static Command spinAuto(Drive drive) {
        return Commands.race(
                DriveCommand.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 1.0),
                new TimerCommand(2000)
        );
    }

    public static Command driveForward(Drive drive) {
        // Make a spot 10 units in front, drive to there.
        // How much is a unit?  Good question.
        var currentPose = drive.getPose();
        var newPos = currentPose.transformBy(new Transform2d(10.0, 0.0, Rotation2d.fromDegrees(0)));

        // TODO: While the 10 units in front of us wont be useful,
        //  This will allow us to drive directly to a pose.
        // We could have a driveToClimb, driveToShootSpot, driveToHopperFillSpot, etc
        return new DriveToPoseCommand(drive, newPos);
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
