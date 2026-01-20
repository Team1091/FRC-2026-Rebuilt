package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

/**
 * Given a starting pose, drive to a target pose.
 * This will drive in a straight line, but will try to ease up and down in speed.
 */
public class DriveToPoseCommand extends Command {
    private final Drive drive;
    private final Pose2d targetPose;

    // TODO: these probably will need to be configured, or will go out of control all over the place
    private final PIDController xController = new PIDController(2.0, 0, 0);
    private final PIDController yController = new PIDController(2.0, 0, 0);
    private final PIDController thetaController = new PIDController(2.0, 0, 0);

    public DriveToPoseCommand(
            Drive drive,
            Pose2d targetPose
    ) {
        this.drive = drive;
        this.targetPose = targetPose;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();

        // Calculate the velocity to drive at using PID
        double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
        double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Clamp to max swerve speeds
        xVelocity = MathUtil.clamp(xVelocity, -Constants.Swerve.maxLinearSpeed, Constants.Swerve.maxLinearSpeed);
        yVelocity = MathUtil.clamp(yVelocity, -Constants.Swerve.maxLinearSpeed, Constants.Swerve.maxLinearSpeed);
        thetaVelocity = MathUtil.clamp(thetaVelocity, -Constants.Swerve.maxAngularSpeed, Constants.Swerve.maxAngularSpeed);

        // Send Velocity to the drive
        drive.runVelocity(new Translation2d(xVelocity, yVelocity), thetaVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        // Shut the motor off when we're done
        drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drive.getPose();
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        return distanceError < 0.05 && angleError < Math.toDegrees(10);
    }
}
