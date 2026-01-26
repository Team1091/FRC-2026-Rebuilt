package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Driving.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
            Swerve swerve,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        swerve.setControl(
                fieldCentricFacingAngleRequest
                        .withVelocityX(Driving.kMaxSpeed.times(input.forward()))
                        .withVelocityY(Driving.kMaxSpeed.times(input.left()))
                        .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
