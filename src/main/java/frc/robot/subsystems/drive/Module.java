package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Module {
    private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final ModuleIO io;
    private final ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
    private final int index;
    private final GenericEntry realAngle;
    private final GenericEntry realVelocity;
    private final GenericEntry targetAngle;
    private final GenericEntry targetVelocity;


    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
    private double lastPositionMeters = 0.0; // Used for delta calculation


    private final static ModulePidConfig[] configs = {

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


    public Module(ModuleIO io, int index, String title) {
        this.io = io;
        this.index = index;

        // Set up shuffleboard
        var tab = Shuffleboard.getTab(title);
        realAngle = tab.add("Real Angle" + title, 0).getEntry();
        realVelocity = tab.add("Real Velocity" + title, 0).getEntry();
        targetAngle = tab.add("Target Angle" + title, 0).getEntry();
        targetVelocity = tab.add("Target Velocity" + title, 0).getEntry();

        var config = configs[index];

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
        if (angleSetpoint != null) {
            io.setTurnVoltage(
                    turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
                // Scale velocity based on turn error
                //
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getError());

                // Run drive controller
                double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
                io.setDriveVoltage(
                        driveFeedforward.calculate(velocityRadPerSec)
                                + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
            }
        }

        // Logging
        realAngle.setDouble(getState().angle.getDegrees());
        realVelocity.setDouble(getState().speedMetersPerSecond);

    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        state.optimize( getAngle());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = state.angle;
        speedSetpoint = state.speedMetersPerSecond;

        // Logging
        targetAngle.setDouble(angleSetpoint.getDegrees());
        targetVelocity.setDouble(speedSetpoint);
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     */
    public void runCharacterization(double volts) {
        // Closed loop turn control
        angleSetpoint = new Rotation2d();

        // Open loop drive control
        io.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        // Disable closed loop control for turn and drive
        angleSetpoint = null;
        speedSetpoint = null;
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
            return new Rotation2d(inputs.turnPosition.plus(turnRelativeOffset).getRadians() % Math.PI);
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