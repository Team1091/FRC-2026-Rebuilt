package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import static frc.robot.RobotContainer.isOnRed;
import static frc.robot.utils.Maths.distance;
import static java.lang.Math.abs;

/**
 * The purpose of this subsystem is to estimate the robot's pose based on vision measurements and estimated drive speeds.
 * <p>
 * A Pose is made up of its location (Translation2d) and orientation (Rotation2d).
 */
public class PoseEstimationSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<Double> rotationRateRadsSecSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

    private final Field2d field = new Field2d();


    public PoseEstimationSubsystem(
            Supplier<Rotation2d> rotationSupplier,
            Supplier<Double> rotationRateRadsSecSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Supplier<SwerveModulePosition[]> modulePositionSupplier
    ) {
        this.rotationSupplier = rotationSupplier;
        this.modulePositionSupplier = modulePositionSupplier;
        this.speedsSupplier = speedsSupplier;
        this.rotationRateRadsSecSupplier = rotationRateRadsSecSupplier;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.kinematics,
                rotationSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(),
                Constants.PoseEstimation.stateStdDevs,
                Constants.PoseEstimation.visionMeasurementStdDevs
        );
    }

    @Override
    public void periodic() {
        poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

        try {
            LimelightHelpers.SetRobotOrientation("limelight", getCurrentPose().getRotation().getDegrees(), rotationRateRadsSecSupplier.get(), 0.0, 0.0, 0.0, 0.0);

            final LimelightHelpers.PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            final LimelightHelpers.PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            var visionComplaints = new ArrayList<String>();

            boolean rejectedMeasurementMt1 = false;
            if (poseEstimate_MegaTag1 == null) {
                visionComplaints.add("mega tag 1 null");
                rejectedMeasurementMt1 = true;
            } else if (poseEstimate_MegaTag1.tagCount <= 0) {
                visionComplaints.add("mega tag 1 no tags seen");
                rejectedMeasurementMt1 = true;
            }

            boolean rejectedMeasurementMt2 = false;
            if (poseEstimate_MegaTag2 == null) {
                visionComplaints.add("mega tag 2 null");
                rejectedMeasurementMt2 = true;
            } else if (poseEstimate_MegaTag2.tagCount <= 0) {
                visionComplaints.add("mega tag 2 no tags seen");
                rejectedMeasurementMt2 = true;
            }

            // If we are rotating too fast, we can't see the tags
            if (abs(rotationRateRadsSecSupplier.get()) > 360) {
                visionComplaints.add("Rotating too fast to see");
            } else {
                // if we have both mt1 + mt2, we should use them both
                if (!rejectedMeasurementMt1 && !rejectedMeasurementMt2) {
                    poseEstimate_MegaTag2.pose = new Pose2d(
                            poseEstimate_MegaTag2.pose.getTranslation(),
                            poseEstimate_MegaTag1.pose.getRotation()
                    );
                    setVisionMeasurementStdDevs(poseEstimate_MegaTag2.rawFiducials, 0.0);
                    poseEstimator.addVisionMeasurement(
                            poseEstimate_MegaTag2.pose,
                            poseEstimate_MegaTag2.timestampSeconds);
                } else if (!rejectedMeasurementMt2) {
                    // We have a mt2 tag, let's just use that
                    setVisionMeasurementStdDevs(poseEstimate_MegaTag2.rawFiducials, 1.0);
                    poseEstimator.addVisionMeasurement(
                            poseEstimate_MegaTag2.pose,
                            poseEstimate_MegaTag2.timestampSeconds);
                } else if (!rejectedMeasurementMt1) {
                    // We have a mt1 tag, let's just use that
                    setVisionMeasurementStdDevs(poseEstimate_MegaTag2.rawFiducials, 2.0);
                    poseEstimator.addVisionMeasurement(
                            poseEstimate_MegaTag1.pose,
                            poseEstimate_MegaTag1.timestampSeconds);
                } else {
                    visionComplaints.add("No vision possible");
                }
            }

            SmartDashboard.putString("Vision", String.join(", ", visionComplaints));
        } catch (Exception e) {
            DataLogManager.log(e.getMessage());
        }

        // Set the field pose to show the dashboard where we think we are.
        field.setRobotPose(getCurrentPose());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putString("You Poser", getCurrentPose().toString());
        SmartDashboard.putNumber("X pos", getCurrentPose().getX());
        SmartDashboard.putNumber("Y pos", getCurrentPose().getY());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    // This calculates how bad our vision input is.  The worse the input, the higher the std dev
    private void setVisionMeasurementStdDevs(LimelightHelpers.RawFiducial[] rawFiducials, double methodError) {
        var chassisSpeeds = speedsSupplier.get();

        // These things will throw off our vision input, so we should use them to calculate our std dev
        double distanceToTarget = 1.0 + Arrays.stream(rawFiducials).map((f) -> f.distToCamera).min(Double::compareTo).get();
        double ambiguity = 1.0 + Arrays.stream(rawFiducials).map((f) -> f.ambiguity).min(Double::compareTo).get();
        var speed = 1.0 + distance(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        var rotation = 1.0 + abs(rotationRateRadsSecSupplier.get());

        var translationError = methodError * speed * rotation * ambiguity * distanceToTarget;
        var rotationError = methodError * speed * rotation * ambiguity * distanceToTarget * 2.0;

        poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(translationError, translationError, rotationError )
        );
    }


    public void resetDriveRotation() {
        if (isOnRed()) {
            poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), new Pose2d(getCurrentPose().getTranslation(), new Rotation2d(Math.PI)));
        } else {
            poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), new Pose2d(getCurrentPose().getTranslation(), new Rotation2d()));
        }
    }

    public Rotation2d getHeadingToTarget(Pose2d targetPose) {
        var currentPose = getCurrentPose();
        var differenceInPos = targetPose.getTranslation().minus(currentPose.getTranslation());

        // need to calculate the heading to the target
        return Rotation2d.fromRadians(Math.atan2(differenceInPos.getY(), differenceInPos.getX()));
    }
}
