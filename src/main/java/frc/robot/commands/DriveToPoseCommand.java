package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FlipPose2d;

import java.time.Duration;

/**
 * Given a starting pose, drive to a target pose.
 * This will drive in a straight line, but will try to ease up and down in speed.
 */
public class DriveToPoseCommand extends Command {
    private final Drive drive;
    private final Pose2d targetPose;
    private final Duration hold;

    // TODO: these probably will need to be configured, or will go out of control all over the place
    private final PIDController xController = new PIDController(0.1, 0, 1.0);
    private final PIDController yController = new PIDController(0.1, 0, 1.0);
    private final PIDController thetaController = new PIDController(1.0, 0.5, 0);

    public DriveToPoseCommand(
            Drive drive,
            FlipPose2d targetPose
    ) {
        this(drive, targetPose.get(), Duration.ofMillis(100));
    }

    public DriveToPoseCommand(
            Drive drive,
            Pose2d targetPose
    ) {
        this(drive, targetPose, Duration.ofMillis(100));
    }

    public DriveToPoseCommand(
            Drive drive,
            FlipPose2d targetPose,
            Duration hold
    ) {
        this(drive, targetPose.get(), hold);
    }

    public DriveToPoseCommand(
            Drive drive,
            Pose2d targetPose,
            Duration hold
    ) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.hold = hold;

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
        xVelocity = MathUtil.clamp(xVelocity, -0.25, 0.25);
        yVelocity = MathUtil.clamp(yVelocity, -0.25, 0.25);
        thetaVelocity = MathUtil.clamp(thetaVelocity, -0.25, 0.25);

        // Send Velocity to the drive
        drive.runVelocity(new Translation2d(-xVelocity, -yVelocity), -thetaVelocity, Constants.Swerve.autoMaxLinearSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Shut the motor off when we're done
        drive.stop();
    }

    private Long startTime = null;

    @Override
    public boolean isFinished() {
        //

        Pose2d currentPose = drive.getPose();
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

        boolean isCloseEnough = distanceError < 0.05 && angleError < 5;

        if (isCloseEnough) {
            var now = System.currentTimeMillis();
            if (startTime == null) {
                startTime = now;
            }
            return now - startTime >= hold.toMillis();

        } else {
            startTime = null;
            return false;
        }
    }
}
