// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FlipPose2d;

import java.time.Duration;


import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;
import static frc.robot.Constants.Locations.hubPose;

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
        var newPos = new FlipPose2d(new Translation2d(2.41, 3.78), new Rotation2d(Units.degreesToRadians(6.5)));

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
            IntakeSubsystem intakeSubsystem,
            PivotSubsystem pivotSubsystem) {

        var readyPos = new FlipPose2d(new Translation2d(1.5, 3.72), new Rotation2d(Units.degreesToRadians(1.5)));
        var backPos = new FlipPose2d(new Translation2d(0.96, 3.72), new Rotation2d(Units.degreesToRadians(2.0)));
        var climbPos = new FlipPose2d(new Translation2d(1.05, 3.72), new Rotation2d(Units.degreesToRadians(2.0)));

        return Commands.sequence(
                new ParallelDeadlineGroup(
                        driveBackAndScore(drive,
                                manualShooterSubsystem,
                                poseEstimationSubsystem,
                                indexerSubsystem,
                                loaderSubsystem,
                                intakeSubsystem,
                                pivotSubsystem)
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
                        new ManualClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)),

                        new ManualClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)
                );
    }

    public static Command driveBackAndScore(
            Drive drive,
            ManualShooterSubsystem manualShooterSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            IndexerSubsystem indexerSubsystem,
            LoaderSubsystem loaderSubsystem,
            IntakeSubsystem intakeSubsystem,
            PivotSubsystem pivotSubsystem
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
                        new TimerCommand(Duration.ofSeconds(4)),
                        new PivotCommand(pivotSubsystem, Constants.Intake.intakeSpeed),
                        new IntakeCommand(intakeSubsystem, Constants.Intake.intakeSpeed),
                        new IndexerCommand(indexerSubsystem, Constants.Indexer.indexerSpeed),
                        new ManualShooterCommand(manualShooterSubsystem, Constants.Shooter.shooterSpeedScore),
                        new LoaderCommand(loaderSubsystem, Constants.Loader.loaderSpeed)
                )
        );
    }

    public static Command yoloSwaggins(
            Drive drive,
            PoseEstimationSubsystem poseEstimationSubsystem,
            ManualShooterSubsystem manualShooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            LoaderSubsystem loaderSubsystem,
            ClimberSubsystem climberSubsystem,
            IntakeSubsystem intakeSubsystem
    ) {
        // min and max distance you can make the shot, don't waste balls
        Distance minDistance = Distance.ofBaseUnits(4, Feet);
        Distance maxDistance = Distance.ofBaseUnits(10, Feet);

        return Commands.sequence(
                new ParallelDeadlineGroup(
                        new DriveToMovingPoseCommand(drive,
                                drive.getPose(),
                                readyPos.get(),
                                Duration.ofSeconds(8)
                        ),
                        // Spins up the shooter to the correct power level for the distance
                        new ShooterPowerAdjustCommand(
                                manualShooterSubsystem,
                                poseEstimationSubsystem
                        ),
                        new ParallelCommandGroup( // This fires the balls when we are in range
                                new IntakeCommand(intakeSubsystem, Constants.Intake.intakeSpeed),
                                new IndexerCommand(indexerSubsystem, Constants.Indexer.indexerSpeed),
                                new LoaderCommand(loaderSubsystem, Constants.Loader.loaderSpeed)
                        ).unless(() -> { // unless we are out of range
                            var distanceInMeters = poseEstimationSubsystem.getCurrentPose().getTranslation().getDistance(hubPose.get().getTranslation());
                            return distanceInMeters < minDistance.in(Meter) && distanceInMeters > maxDistance.in(Meter);
                        })
                ),
                // Go through the pre-climb position
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
                        new ManualClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)
                )
        );
    }
    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

     static FlipPose2d shootingBalls = new FlipPose2d(2.24, 3.73, Rotation2d.fromDegrees(0));

    public final static FlipPose2d leftStartingPose = new FlipPose2d(3.71, 7.3, Rotation2d.fromDegrees(0));
    public final static FlipPose2d centerStartingPose = new FlipPose2d(3.71, 4.05, Rotation2d.fromDegrees(0));
    public final static FlipPose2d rightStartingPose = new FlipPose2d(3.71, 0.39, Rotation2d.fromDegrees(0));


   static final  FlipPose2d readyPos = new FlipPose2d(new Translation2d(1.55, 3.72), new Rotation2d(Units.degreesToRadians(-1.5)));
   static final FlipPose2d backPos = new FlipPose2d(new Translation2d(0.57, 3.72), new Rotation2d(Units.degreesToRadians(-1.5)));
   static final FlipPose2d climbPos = new FlipPose2d(new Translation2d(1.02, 3.72), new Rotation2d(Units.degreesToRadians(-1.5)));

}
