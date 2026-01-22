package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.drive.config.FeedForwardParams;
import frc.robot.subsystems.drive.config.ModulePidConfig;
import frc.robot.subsystems.drive.config.PidConfig;

public class Module {
    private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final ModuleIO io;
    private final ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
    private final int index;

    // Shuffleboard entries
    private final GenericEntry realAngle;
    private final GenericEntry realVelocity;
    private final GenericEntry targetAngle;
    private final GenericEntry targetVelocity;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private SwerveModuleState targetState = new SwerveModuleState();
    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
    private double lastPositionMeters = 0.0; // Used for delta calculation

    public Module(ModuleIO io, int index, String title, ModulePidConfig config) {
        this.io = io;
        this.index = index;

        // Set up shuffleboard
        var tab = Shuffleboard.getTab(title);
        realAngle = tab.add("Real Angle" + title, 0).getEntry();
        realVelocity = tab.add("Real Velocity" + title, 0).getEntry();
        targetAngle = tab.add("Target Angle" + title, 0).getEntry();
        targetVelocity = tab.add("Target Velocity" + title, 0).getEntry();

        // Constants here may change for SIM
        driveFeedforward = config.driveFeedForward().toSimpleMotorFeedforward();
        driveFeedback = config.drivePid().toPidController();
        turnFeedback = config.turnPid().toPidController();

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    public void periodic() {
        io.updateInputs(inputs);


        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
            DriverStation.reportError(
                    "Drive/Module"
                            + index
                            + "/TurnROffset:    "
                            + turnRelativeOffset.getRadians()
                            + "   "
                            + inputs.turnAbsolutePosition
                            + "    "
                            + inputs.turnPosition,
                    false);
        }

        // Run closed loop turn control
        io.setTurnVoltage(
                turnFeedback.calculate(getAngle().getRadians(), targetState.angle.getRadians()));

        // Run closed loop drive control
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = targetState.speedMetersPerSecond * Math.cos(turnFeedback.getError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
        io.setDriveVoltage(
                driveFeedforward.calculate(velocityRadPerSec)
                        + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

        // Logging
        realAngle.setDouble(getState().angle.getDegrees());
        realVelocity.setDouble(getState().speedMetersPerSecond);
    }

    /**
     * Runs the module with the specified setpoint state.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        state.optimize(getAngle());

        targetState = state;

        // Logging
        targetAngle.setDouble(targetState.angle.getDegrees());
        targetVelocity.setDouble(targetState.speedMetersPerSecond);
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     */
    public void runCharacterization(double volts) {
        targetState = new SwerveModuleState(0.0, new Rotation2d());
        io.setDriveVoltage(volts);
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        targetState = new SwerveModuleState(0.0, getAngle());
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);
    }

    /**
     * Sets whether brake mode is enabled.
     */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset);
        }
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getPositionMeters() {
        return inputs.drivePositionRad * WHEEL_RADIUS;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Returns the module position delta since the last call to this method.
     */
    public SwerveModulePosition getPositionDelta() {
        var delta = new SwerveModulePosition(getPositionMeters() - lastPositionMeters, getAngle());
        lastPositionMeters = getPositionMeters();
        return delta;
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Returns the drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}