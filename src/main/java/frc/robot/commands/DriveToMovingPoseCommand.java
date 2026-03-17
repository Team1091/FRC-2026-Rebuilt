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

public class DriveToMovingPoseCommand extends Command {
    private final Drive drive;

    private final Pose2d startPose;
    private final Pose2d endPose;
    private final Duration timeToComplete;
    private Long startTime = null;

    // TODO: these probably will need to be configured, or will go out of control all over the place
    private final PIDController xController = new PIDController(0.1, 0, 1.0);
    private final PIDController yController = new PIDController(0.1, 0, 1.0);
    private final PIDController thetaController = new PIDController(1.0, 0.5, 0);

    public DriveToMovingPoseCommand(
            Drive drive,
            FlipPose2d startPose,
            FlipPose2d endPose,
            Duration timeToComplete
    ) {
        this(drive, startPose.get(), endPose.get(), timeToComplete);
    }

    public DriveToMovingPoseCommand(
            Drive drive,
            Pose2d startPose,
            Pose2d endPose,
            Duration timeToComplete
    ) {
        this.drive = drive;
        this.startPose = startPose;
        this.endPose = endPose;
        this.timeToComplete = timeToComplete;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {

        Pose2d currentPose = drive.getPose();
       var targetPose = startPose.interpolate(endPose, timeToComplete.toSeconds());

        // Calculate the velocity to drive at using PID
        double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
        double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Clamp to max swerve speeds
        xVelocity = MathUtil.clamp(xVelocity, -1, 1);
        yVelocity = MathUtil.clamp(yVelocity, -1, 1);
        thetaVelocity = MathUtil.clamp(thetaVelocity, -1, 1);

        // Send Velocity to the drive
        drive.runVelocity(new Translation2d(-xVelocity, -yVelocity), -thetaVelocity, Constants.Swerve.autoMaxLinearSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Shut the motor off when we're done
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        var now = System.currentTimeMillis();
        if (startTime == null) {
            startTime = now;
        }
        return now - startTime >= timeToComplete.toMillis();
    }
}
