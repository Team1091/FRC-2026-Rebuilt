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
import frc.robot.enums.ShooterState;
import frc.robot.enums.StartPosish;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.PivotSubsystem;
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
        var newPos = currentPose.transformBy(new Transform2d(1.0, 0.0, Rotation2d.fromDegrees(0)));

        // TODO: While the specific forward movement isnt useful
        //  This will allow us to drive directly to a pose.
        // We could have a driveToClimb, driveToShootSpot, driveToHopperFillSpot, etc
        return new DriveToPoseCommand(drive, newPos);
    }

    public static Command driveBack(
            Drive drive,
            PoseEstimationSubsystem poseEstimationSubsystem
    ){
        var currentPose = drive.getPose();
        var newPos = currentPose.transformBy(new Transform2d(-0.3, 0.0, Rotation2d.fromDegrees(0)));

        return new DriveToPoseCommand(drive, newPos);
    }

    public static Command manualAlign(Drive drive){
        return new DriveToPoseCommand(drive, leftManualAlign);
    }

    public static Command driveToOpponent(
            Drive drive,
            HoodSubsystem hoodSubsystem,
            ShooterSubsystem shooterSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            StartPosish startPosish
    ) {
        return switch (startPosish) {
            case LEFT -> Commands.sequence(
                    new ParallelDeadlineGroup(
                            new TimerCommand(Duration.ofSeconds(1)),
                            new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    new ParallelDeadlineGroup(
                            new TimerCommand(Duration.ofSeconds(5)),
                            new DriveWhilePointingAtCommand(drive, poseEstimationSubsystem, Constants.Locations.hubPose, () -> 0.0, () -> 0.0),
                            new ShooterCommand(shooterSubsystem, ShooterState.FIRE),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    new DriveToPoseCommand(drive, leftBalls),
                    new TimerCommand(Duration.ofSeconds(3)),
                    new DriveToPoseCommand(drive, leftOpponent)
            );
            case RIGHT -> Commands.sequence(
                    new ParallelDeadlineGroup(
                            new TimerCommand(Duration.ofSeconds(1)),
                            new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    new ParallelDeadlineGroup(
                            new TimerCommand(Duration.ofSeconds(5)),
                            new DriveWhilePointingAtCommand(drive, poseEstimationSubsystem, Constants.Locations.hubPose, () -> 0.0, () -> 0.0),
                            new ShooterCommand(shooterSubsystem, ShooterState.FIRE),
                            new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                    ),
                    new DriveToPoseCommand(drive, rightBalls),
                    new TimerCommand(Duration.ofSeconds(3)),
                    new DriveToPoseCommand(drive, rightOpponent)
            );
            case CENTER -> Commands.sequence();
        };
    }

    public static Command humanPickup(
            Drive drive,
            HoodSubsystem hoodSubsystem,
            ShooterSubsystem shooterSubsystem,
            ClimberSubsystem climberSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem
    ) {
        return Commands.sequence(
                new ParallelDeadlineGroup(
                        new TimerCommand(Duration.ofSeconds(1)),
                        new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP),
                        new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                ),
                new ParallelDeadlineGroup(
                        new TimerCommand(Duration.ofSeconds(5)),
                        new DriveWhilePointingAtCommand(drive, poseEstimationSubsystem, Constants.Locations.hubPose, () -> 0.0, () -> 0.0),
                        new ShooterCommand(shooterSubsystem, ShooterState.FIRE),
                        new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                ),
                new DriveToPoseCommand(drive, humanPlayer),
                new ParallelDeadlineGroup(
                        new TimerCommand(Duration.ofSeconds(1)),
                        new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP),
                        new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                ),
                new ParallelDeadlineGroup(
                        new TimerCommand(Duration.ofSeconds(5)),
                        new DriveWhilePointingAtCommand(drive, poseEstimationSubsystem, Constants.Locations.hubPose, () -> 0.0, () -> 0.0),
                        new ShooterCommand(shooterSubsystem, ShooterState.FIRE),
                        new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                ),
                new DriveToPoseCommand(drive, shootingBalls),
                new DriveToPoseCommand(drive, climb),
                new AutoClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)
        );
    }

    public static Command swoopThroughMiddleThenShoot(
            Drive drive,
            HoodSubsystem hoodSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotSubsystem pivotSubsystem,
            ClimberSubsystem climberSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            StartPosish startPosish
    ) {

        return switch (startPosish) {
            case LEFT -> Commands.sequence(
                    new PivotCommand(pivotSubsystem, Constants.Pivot.pivotSpeedOut),
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
                            new DriveWhilePointingAtCommand(drive, poseEstimationSubsystem, Constants.Locations.hubPose, () -> 0.0, () -> 0.0),
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
                            new DriveWhilePointingAtCommand(drive, poseEstimationSubsystem, Constants.Locations.hubPose, () -> 0.0, () -> 0.0),
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
    final static Pose2d rightOpponent = new Pose2d(14.71, 0.64, Rotation2d.fromDegrees(0));
    final static Pose2d leftLine = new Pose2d(3.67, 7.32, Rotation2d.fromDegrees(0));
    final static Pose2d leftBalls = new Pose2d(7.71, 7.32, Rotation2d.fromDegrees(0));
    final static Pose2d leftOpponent = new Pose2d(14.71, 7.45, Rotation2d.fromDegrees(0));

    final static Pose2d shootingBalls = new Pose2d(2.24, 3.73, Rotation2d.fromDegrees(0));
    final static Pose2d climb = new Pose2d(1.32, 3.75, Rotation2d.fromDegrees(180));
    final static Pose2d humanPlayer = new Pose2d(0.46, 0.65, Rotation2d.fromDegrees(0));

    public final static Pose2d leftStartingPose = new Pose2d(3.71, 7.3, Rotation2d.fromDegrees(0));
    public final static Pose2d centerStartingPose = new Pose2d(3.71, 4.05, Rotation2d.fromDegrees(0));
    public final static Pose2d rightStartingPose = new Pose2d(3.71, 0.39, Rotation2d.fromDegrees(0));

    final static Pose2d leftReverse = new Pose2d(2, 7.3, Rotation2d.fromDegrees(0));
    final static Pose2d centerReverse = new Pose2d(2, 4.05, Rotation2d.fromDegrees(0));
    final static Pose2d rightReverse = new Pose2d(2, 0.39, Rotation2d.fromDegrees(0));

    public final static Pose2d leftManualAlign = new Pose2d(2.25, 7.32, Rotation2d.fromDegrees(0));

}
