package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import static frc.robot.RobotContainer.isOnRed;

/**
 * The purpose of this subsystem is to estimate the robot's pose based on vision measurements and estimated drive speeds.
 * <p>
 * A Pose is made up of its location (Translation2d) and orientation (Rotation2d).
 */
public class PoseEstimationSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<Double> rotationRateSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

    private final Field2d field = new Field2d();


    public PoseEstimationSubsystem(
            Supplier<Rotation2d> rotationSupplier,
            Supplier<Double> rotationRateSupplier,
            Supplier<SwerveModulePosition[]> modulePositionSupplier
    ) {
        this.rotationSupplier = rotationSupplier;
        this.modulePositionSupplier = modulePositionSupplier;
        this.rotationRateSupplier = rotationRateSupplier;

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
            LimelightHelpers.SetRobotOrientation("limelight", getCurrentPose().getRotation().getDegrees(), rotationRateSupplier.get(), 0.0, 0.0, 0.0, 0.0);
            final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);
            poseEstimator.setVisionMeasurementStdDevs(standardDeviations);


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
            if (Math.abs(rotationRateSupplier.get()) > 360) {
                visionComplaints.add("Rotating too fast to see");
            } else {
                // if we have both mt1 + mt2, we should use them both
                if (!rejectedMeasurementMt1 && !rejectedMeasurementMt2) {
                    poseEstimate_MegaTag2.pose = new Pose2d(
                            poseEstimate_MegaTag2.pose.getTranslation(),
                            poseEstimate_MegaTag1.pose.getRotation()
                    );
                    poseEstimator.addVisionMeasurement(
                            poseEstimate_MegaTag2.pose,
                            poseEstimate_MegaTag2.timestampSeconds);
                } else if (!rejectedMeasurementMt2) {
                    // We have a mt2 tag, let's just use that
                    poseEstimator.addVisionMeasurement(
                            poseEstimate_MegaTag2.pose,
                            poseEstimate_MegaTag2.timestampSeconds);
                } else if (!rejectedMeasurementMt1) {
                    // We have a mt1 tag, let's just use that
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
