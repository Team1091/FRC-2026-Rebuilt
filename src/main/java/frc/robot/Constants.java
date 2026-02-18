package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.config.FeedForwardParams;
import frc.robot.subsystems.drive.config.ModulePidConfig;
import frc.robot.subsystems.drive.config.PidConfig;

public final class Constants {


    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class Swerve {
        public record ModuleConfig(
                int driveMotorId,
                int turnMotorId,
                int cancoderId,
                Rotation2d absoluteEncoderOffset,
                String title,
                boolean isTurnMotorInverted,
                boolean isDriveMotorInverted
        ) {
        }

        public static final int FRONT_LEFT = 0;
        public static final int FRONT_RIGHT = 1;
        public static final int BACK_LEFT = 2;
        public static final int BACK_RIGHT = 3;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
        public static final double trackWidthX = Units.inchesToMeters(20.375);
        public static final double trackWidthY = Units.inchesToMeters(22.25);
        public static final double driveBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
        public static final double maxLinearSpeed = Units.feetToMeters(14.5);
        public static final double maxLinearAcceleration = Units.feetToMeters(10);
        public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
        public static final double maxAngularAcceleration = maxLinearAcceleration / driveBaseRadius;
        public static final double linearDeadband = 0.1;
        public static final double rotationalDeadband = 0.1;

        public static final Translation2d[] moduleTranslations = {
                new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0), // FL
                new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0), // FR
                new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0), // BL
                new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0) // BR
        };
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

        // Per-module hardware configuration (IDs, offsets, titles, inversion)
        public static final ModuleConfig[] moduleHardware = new ModuleConfig[]{
                new ModuleConfig(6, 5, 4, Rotation2d.fromDegrees(42 + 180), "FL", true, false),
                new ModuleConfig(8, 7, 2, Rotation2d.fromDegrees(37 + 180), "FR", true, false),
                new ModuleConfig(1, 2, 1, Rotation2d.fromDegrees(-195 + 180), "BL", true, false),
                new ModuleConfig(3, 4, 3, Rotation2d.fromDegrees(174 + 180), "BR", true, false)
        };

        public static final ModulePidConfig[] moduleConfigs = {
                new ModulePidConfig( //FL
                        new FeedForwardParams(0.1, 0.13),
                        new PidConfig(0.05, 0.0, 0.0), // drive
                        new PidConfig(3, 0.0, 0.0)     // turn
                ),
                new ModulePidConfig( // FR
                        new FeedForwardParams(0.1, 0.13),
                        new PidConfig(0.05, 0.0, 0.0), // drive
                        new PidConfig(3, 0.0, 0.0)     // turn
                ),
                new ModulePidConfig( // BL
                        new FeedForwardParams(0.1, 0.13),
                        new PidConfig(0.05, 0.0, 0.0), // drive
                        new PidConfig(3, 0.0, 0.0)     // turn
                ),
                new ModulePidConfig( // BR
                        new FeedForwardParams(0.1, 0.13),
                        new PidConfig(0.05, 0.0, 0.0), // drive
                        new PidConfig(3, 0.0, 0.0)     // turn
                )
        };
    }

    public static final class Shooter {
        public static final boolean disabled = true;
        public static final int leftMotorChannel = 13;
        public static final int rightMotorChannel = 14;
        public static final int indexerMotorChannel = 15;
        // TODO: we may want target RPMs here for the flywheel
    }

    public static final class Intake {
        public static final boolean disabled = false;
        public static final int intakeMotorChannel = 11;
        public static final double intakeSpeed = 0.3;
    }

    public static final class Pivot{
        public static final boolean disabled = false;
        public static final int pivotMotorChannel = 12;
        public static final double pivotSpeedOut = 0.25;
        public static final double pivotSpeedIn = 0.25;


    }

    public static final class Climber {
        public static final boolean disabled = false;
        public static final int leftMotorChannel = 9;
        public static final int rightMotorChannel = 10;
        public static final double climbingSpeed = 0.25;
        public static final int maxEncoderCount = 100;
    }

    public static final class Camera {
        public static final boolean disabled = true;
    }


    public static final class PoseEstimation {
        public final static Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public final static Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(.7, .7, 9999999);
    }

    public static final class Locations {

        // This is the target position
        public static Pose2d hubPose = new Pose2d(
                new Translation2d(
                        Units.inchesToMeters(158.6),
                        Units.inchesToMeters(317.7 / 2.0)
                ),
                Rotation2d.k180deg
        );
    }

    public static final class Hood {
        public static final boolean disabled = true;
        public static final int hoodMotorChannel = 13;
        public static final double hoodMotorPower = 0.25;
        public static final double angleCloseEnough = 0.1;
    }

}
