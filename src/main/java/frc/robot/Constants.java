package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
    public static final double trackWidthX = Units.inchesToMeters(25.0);
    public static final double trackWidthY = Units.inchesToMeters(25.0);
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

  public static final class Chute {
    public static final int motorChannel = 11;
    public static final int limitSwitchChannel = 3;
    public static final double holdSpeed = 0.15;
    public static final double shootSpeed = 0.3;
  }

  public static final class Intake {
    public static final int frontMotorChannel = 15;
    public static final double suckSpeed = .5;
    public static final double shootSpeed = .2;
  }

  public static final class Pivot {
    public static final int motorChannel = 17;
    public static final int limitSwitchChannel = 9;
    public static final double speed = 0.2;
    public static final double outPosition = 30;
    public static final double scorePosition = 10;
  }

  public static final class Elevator {
    public static final int motorChannel = 13;
    public static final int limitSwitchChannelTop = 0;
    public static final int limitSwitchChannelBottom = 4;
    public static final double downSpeed = .6;
    public static final double upSpeed = .8;
    public static final double l2Position = 32;
    public static final double l3Position = 69;
    public static final double l4Position = 118;
    public static final double algae1Position = 15;
    public static final double algae2Position = 54;
    public static final double retractPosition = 200;
  }

  public static final class Extender {
    public static final int motorChannel = 12;
    public static final int limitSwitchChannel = 2;
    public static final double speed = 0.5;
    public static final double algaePosition = 20;
    public static final double scorePosition = 0;
  }

  public static final class Climber {
    public static final int motorChannel = 14;
    public static final double speed = 1;
    public static final double outMax = 261;
  }

  public static final class PoseEstimation {
    public final static Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public final static Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(.7, .7, 9999999);
  }

//  public static final class PathPlanner {
//    public static final PPHolonomicDriveController controller = new PPHolonomicDriveController(
//            new PIDConstants(.5, 0, 0), // 2.0 Translation constants 3
//            new PIDConstants(3, 0, 0) // 1.3 Rotation constants 3
//    );
//    public static final RobotConfig config = new RobotConfig(15, 1, new ModuleConfig(0.0508, 3, 1.2, DCMotor.getNEO(1).withReduction((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)), 120.0, 1), trackWidthX);
//  }
}
