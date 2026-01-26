package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;
import frc.util.Stopwatch;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Seconds;

/**
 * Teleop manual drive command for the swerve drivetrain.
 * <p>
 * Handles field-centric driving with manual rotation input and
 * heading-hold behavior after a short delay once rotation input
 * returns to zero.
 */
public class ManualDriveCommand extends Command {
    private enum State {
        IDLING,
        DRIVING_WITH_MANUAL_ROTATION,
        DRIVING_WITH_LOCKED_HEADING
    }

    private static final Time kHeadingLockDelay = Seconds.of(0.25); // time to wait before locking heading

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Driving.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(5, 0, 0);

    private State currentState = State.IDLING;
    private Optional<Rotation2d> lockedHeading = Optional.empty();
    private Stopwatch headingLockStopwatch = new Stopwatch();
    private ManualDriveInput previousInput = new ManualDriveInput();

    public ManualDriveCommand(
            Swerve swerve,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput,
            DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
        addRequirements(swerve);
    }

    public void seedFieldCentric() {
        initialize();
        swerve.seedFieldCentric();
    }

    public void setLockedHeading(Rotation2d heading) {
        lockedHeading = Optional.of(heading);
        currentState = State.DRIVING_WITH_LOCKED_HEADING;
    }

    private void setLockedHeadingToCurrent() {
        final Rotation2d headingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d headingInOperatorPerspective = headingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        setLockedHeading(headingInOperatorPerspective);
    }

    private void lockHeadingIfRotationStopped(ManualDriveInput input) {
        if (input.hasRotation()) {
            headingLockStopwatch.reset();
            lockedHeading = Optional.empty();
        } else {
            headingLockStopwatch.startIfNotRunning();
            if (headingLockStopwatch.elapsedTime().gt(kHeadingLockDelay)) {
                setLockedHeadingToCurrent();
            }
        }
    }

    @Override
    public void initialize() {
        currentState = State.IDLING;
        lockedHeading = Optional.empty();
        headingLockStopwatch.reset();
        previousInput = new ManualDriveInput();
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        if (input.hasRotation()) {
            currentState = State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (input.hasTranslation()) {
            currentState = lockedHeading.isPresent() ? State.DRIVING_WITH_LOCKED_HEADING : State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
            currentState = State.IDLING;
        }
        previousInput = input;

        switch (currentState) {
            case IDLING:
                swerve.setControl(idleRequest);
                break;
            case DRIVING_WITH_MANUAL_ROTATION:
                lockHeadingIfRotationStopped(input);
                swerve.setControl(
                        fieldCentricRequest
                                .withVelocityX(Driving.kMaxSpeed.times(input.forward()))
                                .withVelocityY(Driving.kMaxSpeed.times(input.left()))
                                .withRotationalRate(Driving.kMaxRotationalRate.times(input.rotation()))
                );
                break;
            case DRIVING_WITH_LOCKED_HEADING:
                swerve.setControl(
                        fieldCentricFacingAngleRequest
                                .withVelocityX(Driving.kMaxSpeed.times(input.forward()))
                                .withVelocityY(Driving.kMaxSpeed.times(input.left()))
                                .withTargetDirection(lockedHeading.get())
                );
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Default drive command: runs until interrupted
        return false;
    }
}
