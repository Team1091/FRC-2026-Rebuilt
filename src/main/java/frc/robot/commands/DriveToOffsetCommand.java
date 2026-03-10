package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class DriveToOffsetCommand extends Command {
    private final Drive drive;
    private final Translation2d offset;
    private final Rotation2d targetRotation;
    
    private SwerveDriveOdometry odometry;
    private Pose2d startPose;
    private Pose2d targetPose;
    
    private final PIDController xController = new PIDController(1.0, 0, 0);
    private final PIDController yController = new PIDController(1.0, 0, 0);
    private final PIDController thetaController = new PIDController(1.0, 0, 0);

    public DriveToOffsetCommand(
            Drive drive,
            Translation2d offset,
            Rotation2d targetRotation
    ) {
        this.drive = drive;
        this.offset = offset;
        this.targetRotation = targetRotation;
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drive);
    }

    public DriveToOffsetCommand(
            Drive drive,
            Translation2d offset
    ) {
        this(drive, offset, null);
    }

    public DriveToOffsetCommand(
            Drive drive,
            double xOffset,
            double yOffset
    ) {
        this(drive, new Translation2d(xOffset, yOffset), null);
    }

    public DriveToOffsetCommand(
            Drive drive,
            double xOffset,
            double yOffset,
            Rotation2d targetRotation
    ) {
        this(drive, new Translation2d(xOffset, yOffset), targetRotation);
    }

    @Override
    public void initialize() {
        Rotation2d currentRotation = drive.getGyroRotation();
        
        odometry = new SwerveDriveOdometry(
                Constants.Swerve.kinematics,
                currentRotation,
                drive.getModulePositions()
        );
        
        startPose = odometry.getPoseMeters();
        
        Rotation2d finalRotation = (targetRotation != null) ? targetRotation : currentRotation;
        targetPose = new Pose2d(
                startPose.getX() + offset.getX(),
                startPose.getY() + offset.getY(),
                finalRotation
        );
    }

    @Override
    public void execute() {
        odometry.update(drive.getGyroRotation(), drive.getModulePositions());
        Pose2d currentPose = odometry.getPoseMeters();

        double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
        double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        xVelocity = MathUtil.clamp(xVelocity, -Constants.Swerve.autoMaxLinearSpeed, Constants.Swerve.autoMaxLinearSpeed);
        yVelocity = MathUtil.clamp(yVelocity, -Constants.Swerve.autoMaxLinearSpeed, Constants.Swerve.autoMaxLinearSpeed);
        thetaVelocity = MathUtil.clamp(thetaVelocity, -Constants.Swerve.autoMaxAngularSpeed, Constants.Swerve.autoMaxAngularSpeed);

        drive.runVelocity(new Translation2d(xVelocity, yVelocity), thetaVelocity, Rotation2d.kZero);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = odometry.getPoseMeters();
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        return distanceError < 0.05 && angleError < Math.toRadians(10);
    }
}
