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
        modules[FRONT_LEFT] = new Module(flModuleIO, 0, "FL");
        modules[FRONT_RIGHT] = new Module(frModuleIO, 1, "FR");
        modules[BACK_LEFT] = new Module(blModuleIO, 2, "BL");
        modules[BACK_RIGHT] = new Module(brModuleIO, 3, "BR");

        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition();
        }

        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("States", SwerveModuleState.struct).publish();

//        if(isOnRed()){
//            middle = new Translation2d(13.25, 4);
//        } else {
//            middle = new Translation2d(4.5, 4);
//        }
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

//    public void configureAutoBuilder(PoseEstimationSubsystem poseEstimationSubsystem) {
//        RobotConfig localConfig;
//        try {
//            localConfig = RobotConfig.fromGUISettings();
//        } catch (Exception e) {
//            localConfig = config;
//            e.printStackTrace();
//        }
//        this.poseEstimationSubsystem = poseEstimationSubsystem;
//        AutoBuilder.configure(
//                this::getPose, // Robot pose supplier
//                poseEstimationSubsystem::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
//                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//                (speeds, feedforwards) -> runVelocity(speeds),
//                controller, //The path planner controller
//                localConfig, // The robot configuration
//                this::isOnRed,
//                this // Reference to this subsystem to set requirements
//        );
//    }

    /**
     * Runs the drive at the desired velocity.
     */
    public void runVelocity(Translation2d linearVelocity, double omega) {
        Rotation2d rotation;

        int invert = 1;

        if (isOnRed() && isFieldOriented) {
            invert = -1;
        }

        if (isFieldOriented) {
            rotation = getPose().getRotation();
        } else {
            rotation = new Rotation2d(0);
        }
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * maxLinearSpeed * invert,
                linearVelocity.getY() * maxLinearSpeed * invert,
                omega * maxAngularSpeed,
                rotation
        );
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

    public void setFieldState(boolean bool) {
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

    public void toggleDefenseMode(){
        defenseMode = !defenseMode;
        SmartDashboard.putBoolean("Defense Mode", defenseMode);
    }

//    public boolean canMove() {
//        return !defenseMode;
//    }

    // public Rotation2d getHeadingToMiddle(){
    //     return getPose().getTranslation().minus(middle).getAngle();
    // }
}