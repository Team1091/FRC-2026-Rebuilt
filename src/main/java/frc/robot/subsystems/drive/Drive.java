package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;

import static frc.robot.Constants.Swerve.BACK_LEFT;
import static frc.robot.Constants.Swerve.BACK_RIGHT;
import static frc.robot.Constants.Swerve.FRONT_LEFT;
import static frc.robot.Constants.Swerve.FRONT_RIGHT;
import static frc.robot.Constants.Swerve.kinematics;
import static frc.robot.Constants.Swerve.maxAngularSpeed;
import static frc.robot.Constants.Swerve.maxLinearSpeed;
import static frc.robot.Constants.Swerve.moduleTranslations;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIO.GyroIOInputs gyroInputs = new GyroIO.GyroIOInputs();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private boolean isFieldOriented = true;
    private boolean defenseMode = false;
    private ChassisSpeeds chassisSpeeds;
    //    private Translation2d middle;
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private PoseEstimationSubsystem poseEstimationSubsystem;
    private final StructArrayPublisher<SwerveModuleState> statePublisher;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[FRONT_LEFT] = new Module(flModuleIO, FRONT_LEFT, "FL", createDefaultPidConfig());
        modules[FRONT_RIGHT] = new Module(frModuleIO, FRONT_RIGHT, "FR", createDefaultPidConfig());
        modules[BACK_LEFT] = new Module(blModuleIO, BACK_LEFT, "BL", createDefaultPidConfig());
        modules[BACK_RIGHT] = new Module(brModuleIO, BACK_RIGHT, "BR", createDefaultPidConfig());

        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition();
        }

        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("States", SwerveModuleState.struct).publish();
    }

    public void setPoseEstimationSubSystem(PoseEstimationSubsystem poseEstimationSubsystem) {
        this.poseEstimationSubsystem = poseEstimationSubsystem;
    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Update odometry
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = modules[i].getPosition();
        }

        statePublisher.set(getModuleStates());
    }

    /**
     * Runs the drive at the desired velocity.
     */
    public void runVelocity(Translation2d linearVelocity, double omega) {
        Rotation2d rotation = isFieldOriented ? getPose().getRotation() : new Rotation2d();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * maxLinearSpeed,
                linearVelocity.getY() * maxLinearSpeed,
                omega * maxAngularSpeed,
                rotation
        );

        // If we're on the red alliance, we need to flip the inputs because the field coordinate system
        // is always blue-origin.
        if (isOnRed() && isFieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    -linearVelocity.getX() * maxLinearSpeed,
                    -linearVelocity.getY() * maxLinearSpeed,
                    omega * maxAngularSpeed,
                    rotation
            );
        }

        runVelocity(chassisSpeeds);
    }

    public void runVelocity(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void straightenWheels() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = new Rotation2d(0);
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void setIsFieldOriented(boolean bool) {
        isFieldOriented = bool;
    }

    public void toggleIsFieldOriented() {
        isFieldOriented = !isFieldOriented;
    }

    /**
     * Runs forwards at the commanded voltage.
     */
    public void runCharacterizationVolts(double volts) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
        }
    }

    /**
     * Returns the average drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    public Rotation2d getGyroRotation() {
        return gyroInputs.yawPosition;
    }

    public Pose2d getPose() {
        return poseEstimationSubsystem.getCurrentPose();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return chassisSpeeds;
    }

    public void resetGyro() {
        gyroIO.resetGyro();
    }

    public boolean isOnRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    public void toggleDefenseMode() {
        defenseMode = !defenseMode;
        SmartDashboard.putBoolean("Defense Mode", defenseMode);
    }

    // These are some defaults that work for the PIDs in the wheels
    private ModulePidConfig createDefaultPidConfig() {
        return new ModulePidConfig( //FL
                new FeedForwardParams(0.1, 0.13),
                new PidConfig(0.05, 0.0, 0.0), // drive
                new PidConfig(3, 0.0, 0.0)     // turn
        );
    }

}