package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

/**
 * Given a starting pose, drive to a target pose.
 * This will drive in a straight line, but will try to ease up and down in speed.
 */
public class DriveWhilePointingAtCommand extends Command {
    private final Drive drive;
    private final PoseEstimationSubsystem poseEstimationSubsystem;
    private final Pose2d targetPose;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final PIDController thetaController = new PIDController(2.0, 0, 0);


    public DriveWhilePointingAtCommand(
            Drive drive,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Pose2d targetPose,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier
    ) {
        this.drive = drive;
        this.poseEstimationSubsystem = poseEstimationSubsystem;
        this.targetPose = targetPose;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Figure out the rotational velocity that will point us at the target
        Pose2d currentPose = drive.getPose();
        var headingToTarget = poseEstimationSubsystem.getHeadingToTarget(targetPose);
        var currentPoseHeading = currentPose.getRotation();
        double omega = thetaController.calculate(
                currentPoseHeading.getRadians(),
                headingToTarget.getRadians()
        );

        // Calculate the velocity to drive at using the left stick
        double xVelocity = xSupplier.getAsDouble();
        double yVelocity = ySupplier.getAsDouble();

        // Deadbands
        omega = MathUtil.applyDeadband(omega, Constants.Swerve.rotationalDeadband);

        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xVelocity, yVelocity), Constants.Swerve.linearDeadband);
        Rotation2d linearDirection = new Rotation2d(-xVelocity, yVelocity);

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);


        // Calculate new linear velocity
        Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);

        drive.runVelocity(linearVelocity, omega);
    }

    @Override
    public void end(boolean interrupted) {
        // Shut the motor off when we're done
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
