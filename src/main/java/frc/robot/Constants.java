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
        public static final Translation2d[] moduleTranslations = {
                new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0), // FL
                new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0), // FR
                new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0), // BL
                new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0) // BR
        };
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        public static final double linearDeadband = 0.1;
        public static final double rotationalDeadband = 0.1;
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
        public static boolean disabled = false;
        public static int leftMotorChannel = 9;
        public static int dumbLeftMotorChannel = 17;
        public static int dumbRightMotorChannel = 21;
        public static double leftMotorPower = 0.25;
        public static int rightMotorChannel = 10;
        public static double rightMotorPower = 0.25;
        public static double climbingSpeed = 0.25;
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

}
