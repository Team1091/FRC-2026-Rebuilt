// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.enums.ShooterState;
import frc.robot.enums.StartPosish;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ManualShooterSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FlipPose2d;

import java.time.Duration;
import java.util.Timer;

import static frc.robot.RobotContainer.isOnRed;

public final class Autos {

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

    public static Command driveBackScore(
            Drive drive,
            PoseEstimationSubsystem poseEstimationSubsystem
    ) {
        var newPos = new FlipPose2d(new Translation2d(1.94, 3.72), new Rotation2d(Units.degreesToRadians(8.1)));

        return new DriveToPoseCommand(drive, newPos);
    }

    public static Command manualAlign(Drive drive) {
        return new DriveToPoseCommand(drive, shootingBalls);
    }

    //Hard code climb
    public static Command scoreAndClimb(
            Drive drive,
            PoseEstimationSubsystem poseEstimationSubsystem,
            ManualShooterSubsystem manualShooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            LoaderSubsystem loaderSubsystem,
            ClimberSubsystem climberSubsystem,
            IntakeSubsystem intakeSubsystem){

        var readyPos = new FlipPose2d(new Translation2d(1.55, 3.72), new Rotation2d(Units.degreesToRadians(-1.5)));
        var backPos = new FlipPose2d(new Translation2d(0.57, 3.72), new Rotation2d(Units.degreesToRadians(-1.5)));
        var climbPos = new FlipPose2d(new Translation2d(1.02, 3.72), new Rotation2d(Units.degreesToRadians(-1.5)));

        return Commands.sequence(
                new ParallelDeadlineGroup(
                        driveBackAndScore(drive,
                                manualShooterSubsystem,
                                poseEstimationSubsystem,
                                indexerSubsystem,
                                loaderSubsystem,
                                intakeSubsystem)
                ),
                new ParallelDeadlineGroup(
                        new DriveToPoseCommand(drive, readyPos)
                ),
                new ParallelDeadlineGroup(
                        new DriveToPoseCommand(drive, backPos)
                ),
                new ParallelDeadlineGroup(
                        new TimerCommand(Duration.ofSeconds(3)),
                        new ManualClimbCommand(climberSubsystem, Constants.Climber.climbingSpeed)
                ),
                new ParallelDeadlineGroup(
                        new DriveToPoseCommand(drive, climbPos)
                ),
                new ParallelDeadlineGroup(
                        new TimerCommand(Duration.ofSeconds(3)),
                        new ManualClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)                )

        );
    }

    public static Command driveBackAndScore(
            Drive drive,
            ManualShooterSubsystem manualShooterSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            IndexerSubsystem indexerSubsystem,
            LoaderSubsystem loaderSubsystem,
            IntakeSubsystem intakeSubsystem
    ) {
        return Commands.sequence(
                new ParallelDeadlineGroup(
                        driveBackScore(drive, poseEstimationSubsystem)
                ),
                new ParallelRaceGroup(
                        new TimerCommand(Duration.ofSeconds(1)),
                        new ManualShooterCommand(manualShooterSubsystem, Constants.Shooter.shooterSpeedScore)
                ),
                new ParallelDeadlineGroup(
                        new TimerCommand(Duration.ofSeconds(5)),
                        new IntakeCommand(intakeSubsystem, Constants.Intake.intakeSpeed),
                        new IndexerCommand(indexerSubsystem, Constants.Indexer.indexerSpeed),
                        new ManualShooterCommand(manualShooterSubsystem, Constants.Shooter.shooterSpeedScore),
                        new LoaderCommand(loaderSubsystem, Constants.Loader.loaderSpeed)
                )
        );
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

    final static FlipPose2d rightLine = new FlipPose2d(3.67, 0.68, Rotation2d.fromDegrees(0));
    final static FlipPose2d rightBalls = new FlipPose2d(7.71, 0.68, Rotation2d.fromDegrees(0));
    final static FlipPose2d rightOpponent = new FlipPose2d(14.71, 0.64, Rotation2d.fromDegrees(0));
    final static FlipPose2d leftLine = new FlipPose2d(3.67, 7.32, Rotation2d.fromDegrees(0));
    final static FlipPose2d leftBalls = new FlipPose2d(7.71, 7.32, Rotation2d.fromDegrees(0));
    final static FlipPose2d leftOpponent = new FlipPose2d(14.71, 7.45, Rotation2d.fromDegrees(0));

    final static FlipPose2d shootingBalls = new FlipPose2d(2.24, 3.73, Rotation2d.fromDegrees(0));
    final static FlipPose2d climb = new FlipPose2d(1.32, 3.75, Rotation2d.fromDegrees(180));
    final static FlipPose2d humanPlayer = new FlipPose2d(0.46, 0.65, Rotation2d.fromDegrees(0));

    public final static FlipPose2d leftStartingPose = new FlipPose2d(3.71, 7.3, Rotation2d.fromDegrees(0));
    public final static FlipPose2d centerStartingPose = new FlipPose2d(3.71, 4.05, Rotation2d.fromDegrees(0));
    public final static FlipPose2d rightStartingPose = new FlipPose2d(3.71, 0.39, Rotation2d.fromDegrees(0));

    final static FlipPose2d leftReverse = new FlipPose2d(2, 7.3, Rotation2d.fromDegrees(0));
    final static FlipPose2d centerReverse = new FlipPose2d(2, 4.05, Rotation2d.fromDegrees(0));
    final static FlipPose2d rightReverse = new FlipPose2d(2, 0.39, Rotation2d.fromDegrees(0));

    public final static FlipPose2d leftManualAlign = new FlipPose2d(2.25, 7.32, Rotation2d.fromDegrees(0));

}
