package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {


    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class Swerve {
        public static final int FRONT_LEFT = 0;
        public static final int FRONT_RIGHT = 1;
        public static final int BACK_LEFT = 2;
        public static final int BACK_RIGHT = 3;

        public static final double trackWidthX = Units.inchesToMeters(20.375);
        public static final double trackWidthY = Units.inchesToMeters(22.25);
        public static final double driveBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
        public static final double maxLinearSpeed = Units.feetToMeters(14.5);
        public static final double maxLinearAcceleration = Units.feetToMeters(10);
        public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
        public static final double maxAngularAcceleration = maxLinearAcceleration / driveBaseRadius;

        public static final ModuleConfig[] moduleConfigs = {
                new ModuleConfig("FL", FRONT_LEFT, 6, 5, 4, Rotation2d.fromDegrees(42)),
                new ModuleConfig("FR", FRONT_RIGHT, 8, 7, 2, Rotation2d.fromDegrees(37)),
                new ModuleConfig("BL", BACK_LEFT, 1, 2, 1, Rotation2d.fromDegrees(-195)),
                new ModuleConfig("BR", BACK_RIGHT, 3, 4, 3, Rotation2d.fromDegrees(174))
        };
        public static final Translation2d[] moduleTranslations = {
                new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0), // FL
                new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0), // FR
                new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0), // BL
                new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0) // BR
        };
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        public static final double linearDeadband = 0.1;
        public static final double rotationalDeadband = 0.1;

        public static class Module {
            public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

            // Gear ratios for SDS MK4i L2
            public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

            public static final int DRIVE_CURRENT_LIMIT_AMPS = 40;
            public static final int TURN_CURRENT_LIMIT_AMPS = 20;

            public static final double VOLTAGE_COMPENSATION = 12.0;

            public static final int ENCODER_MEASUREMENT_PERIOD_MS = 10;
            public static final int ENCODER_AVERAGE_DEPTH = 2;

            public static final int CAN_TIMEOUT_MS = 250;
        }
    }

    public static class Shooter {
        public static boolean disabled = true;
        public static int leftMotorChannel = 13;
        public static int rightMotorChannel = 14;
        public static int indexerMotorChannel = 15;
        // TODO: we may want target RPMs here for the flywheel
    }

    public static class Intake {
        public static boolean disabled = true;
        public static int intakeMotorChannel = 11;
        public static int extenderMotorChannel = 12;
        public static double extenderMotorPower = 0.25;
    }

    public static class Climber {
        public static boolean disabled = true;
        public static int leftMotorChannel = 9;
        public static double leftMotorPower = 0.25;
        public static int rightMotorChannel = 10;
        public static double rightMotorPower = 0.25;
    }

    public static class Camera {
        public static boolean disabled = true;
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
        public static boolean disabled = true;
        public static int hoodMotorChannel = 13;
        public static double hoodMotorPower = 0.25;
        public static double angleCloseEnough = 0.1;
    }

    public record ModuleConfig(
            String title,
            int index,
            int driveId,
            int turnId,
            int cancoderId,
            Rotation2d absoluteEncoderOffset
    ) {
    }


}
