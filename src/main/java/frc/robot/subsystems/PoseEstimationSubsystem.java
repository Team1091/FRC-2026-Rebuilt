package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.List;
import java.util.function.Supplier;

public class PoseEstimationSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

    private final Field2d field = new Field2d();
//    private String reefPosition = "right";
//    private int reefSide = 1;
//
//    private final List<Translation2d> waypoints = List.of(
//            new Translation2d(3, 4),
//            new Translation2d(3.75, 2.742),
//            new Translation2d(5.219, 2.742),
//            new Translation2d(5.956, 4),
//            new Translation2d(5.219, 5.316),
//            new Translation2d(3.75, 5.316),
//            new Translation2d(14.719, 4),
//            new Translation2d(13.969, 5.316),
//            new Translation2d(12.5, 5.316),
//            new Translation2d(11.75, 4),
//            new Translation2d(12.5, 2.742),
//            new Translation2d(13.969, 2.742)
//    );

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

//        reefSide = waypoints.indexOf(getCurrentPose().getTranslation().nearest(waypoints)) % 6 + 1;
//        SmartDashboard.putNumber("Reef Side", reefSide);
//        SmartDashboard.putString("Coral Station", getCoralStation());

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

//    public int getReefSide() {
//        return reefSide;
//    }
//
//    public String getReefPosition() {
//        return reefPosition;
//    }

//    public String getCoralStation() {
//        if (isOnRed()) {
//            return (getCurrentPose().getY() < 4) ? "Left Coral" : "Right Coral";
//        } else {
//            return (getCurrentPose().getY() > 4) ? "Left Coral" : "Right Coral";
//        }
//    }



//    public void resetFieldPosition() {
//        setCurrentPose(new Pose2d());
//    }
//
//    public void resetDriveRotation() {
//        if (isOnRed()) {
//            poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), new Pose2d(getCurrentPose().getTranslation(), new Rotation2d(Math.PI)));
//        } else {
//            poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), new Pose2d(getCurrentPose().getTranslation(), new Rotation2d()));
//        }
//    }

//    public boolean isOnRed() {
//        var alliance = DriverStation.getAlliance();
//        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
//    }
//
//    public void setReefPositionLeft() {
//        setReefPosition("left");
//    }
//
//    public void setReefPositionAlgae() {
//        setReefPosition("algae");
//    }
//
//    public void setReefPositionRight() {
//        setReefPosition("right");
//    }
//
//    public void setReefPosition(String reefPosition) {
//        this.reefPosition = reefPosition;
//        SmartDashboard.putString("Reef Position", this.reefPosition);
//    }
}
