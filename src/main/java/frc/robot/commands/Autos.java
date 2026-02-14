// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.enums.StartPosish;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

import java.time.Duration;

public final class Autos {


    // Spin wildly for a second.
    public static Command spinAuto(Drive drive) {
        return Commands.race(
                DriveCommand.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 1.0),
                new TimerCommand(Duration.ofSeconds(2))
        );
    }

    public static Command driveForward(Drive drive) {
        // Make a spot 10 units in front, drive to there.
        // How much is a unit?  Good question. Don't stand near it when we find out.
        var currentPose = drive.getPose();
        var newPos = currentPose.transformBy(new Transform2d(10.0, 0.0, Rotation2d.fromDegrees(0)));

        // TODO: While the 10 units in front of us wont be useful,
        //  This will allow us to drive directly to a pose.
        // We could have a driveToClimb, driveToShootSpot, driveToHopperFillSpot, etc
        return new DriveToPoseCommand(drive, newPos);
    }

    public static Command win(
            Drive drive,
            HoodSubsystem hoodSubsystem,
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            ClimberSubsystem climberSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            StartPosish startPosish
    ) {

        return switch (startPosish) {
            case LEFT -> Commands.sequence(
                    new RunIntakeCommand(intakeSubsystem, IntakeState.EXTENDED),
                    new DriveToPoseCommand(drive, leftLine),
                    new DriveToPoseCommand(drive, leftBalls),

                    new DriveToPoseCommand(drive, rightBalls),
                    new DriveToPoseCommand(drive, rightLine),
                    // Extend climber
                    new AutoClimbCommand(climberSubsystem, Constants.Climber.climbingSpeed),
                    // Move to shooting position and spin up
                    new ParallelCommandGroup(
                            new DriveToPoseCommand(drive, shootingBalls),
                            new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    // spin up shooter, shoot
                    new ParallelDeadlineGroup(
                            new TimerCommand(Duration.ofSeconds(5)),
                            new ShooterCommand(shooterSubsystem, ShooterState.FIRE),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    new DriveToPoseCommand(drive, climb),
                    // Retract climber
                    new AutoClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)
            );

            case RIGHT -> Commands.sequence(
                    new DriveToPoseCommand(drive, rightLine),
                    new DriveToPoseCommand(drive, rightBalls),
                    new DriveToPoseCommand(drive, leftBalls),
                    new DriveToPoseCommand(drive, leftLine),
                    // Extend climber
                    new AutoClimbCommand(climberSubsystem, Constants.Climber.climbingSpeed),
                    // Move to shooting position and spin up
                    new ParallelCommandGroup(
                            new DriveToPoseCommand(drive, shootingBalls),
                            new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    // spin up shooter, shoot
                    new ParallelDeadlineGroup(
                            new TimerCommand(Duration.ofSeconds(5)),
                            new ShooterCommand(shooterSubsystem, ShooterState.FIRE),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    new DriveToPoseCommand(drive, climb),
                    new AutoClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)
            );

            case CENTER -> Commands.sequence();

        };
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    final static Pose2d rightLine = new Pose2d(3.67, 0.68, Rotation2d.fromDegrees(0));
    final static Pose2d rightBalls = new Pose2d(7.71, 0.68, Rotation2d.fromDegrees(0));
    final static Pose2d leftBalls = new Pose2d(7.71, 7.32, Rotation2d.fromDegrees(0));
    final static Pose2d leftLine = new Pose2d(3.67, 7.32, Rotation2d.fromDegrees(0));

    final static Pose2d shootingBalls = new Pose2d(2.24, 5.05, Rotation2d.fromDegrees(0));
    final static Pose2d climb = new Pose2d(1.56, 3.70, Rotation2d.fromDegrees(180));
}
