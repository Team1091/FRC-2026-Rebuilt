package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.function.Supplier;

/**
 * The purpose of this subsystem is to estimate the robot's pose based on vision measurements and estimated drive speeds.
 * <p>
 * A Pose is made up of its location (Translation2d) and orientation (Rotation2d).
 */
public class PoseEstimationSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

    private final Field2d field = new Field2d();

    public PoseEstimationSubsystem(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier) {
        this.rotationSupplier = rotationSupplier;
        this.modulePositionSupplier = modulePositionSupplier;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.kinematics,
                rotationSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(),
                Constants.PoseEstimation.stateStdDevs,
                Constants.PoseEstimation.visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

        try {
            LimelightHelpers.SetRobotOrientation("limelight", getCurrentPose().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            if (limelightMeasurement.tagCount > 0) {
                poseEstimator.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds);
            }
        } catch (Exception e) {
            // TODO: handle exception
            DataLogManager.log(e.getMessage());
        }

        // Set the field pose to show the dashboard where we think we are.
        field.setRobotPose(getCurrentPose());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("X pos", getCurrentPose().getX());
        SmartDashboard.putNumber("Y pos", getCurrentPose().getY());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    public Rotation2d getHeadingToTarget(Pose2d targetPose){
        var currentPose = getCurrentPose();
        var differenceInPos = targetPose.getTranslation().minus(currentPose.getTranslation());

        // need to calculate the heading to the target
        return Rotation2d.fromRadians( Math.atan2(differenceInPos.getY(), differenceInPos.getX()));
    }
}
