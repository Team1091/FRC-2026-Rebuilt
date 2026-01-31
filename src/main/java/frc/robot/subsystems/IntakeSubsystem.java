package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// picks balls off the ground
public class IntakeSubsystem extends SubsystemBase {

    private IntakeState state = IntakeState.RETRACTED;

    private final SparkMax intakeMotor;

    private final SparkMax extenderMotor;
    private final RelativeEncoder extenderEncoder;
    private final PIDController extenderController = new PIDController(2.0, 0, 0);

    public IntakeSubsystem() {
        // set up motors
        intakeMotor = new SparkMax(Constants.Intake.intakeMotorChannel, SparkLowLevel.MotorType.kBrushless);

        extenderMotor = new SparkMax(Constants.Intake.extenderMotorChannel, SparkLowLevel.MotorType.kBrushless);
        extenderEncoder = extenderMotor.getEncoder();
        extenderEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {

        // set motor speeds
        intakeMotor.set(state.intakeMotorSpeed);

        var extenderPow = extenderController.calculate(extenderEncoder.getPosition(), state.extenderMotorPosition);
        MathUtil.clamp(extenderPow, -Constants.Intake.extenderMotorPower, Constants.Intake.extenderMotorPower);
        extenderMotor.set(extenderPow);
    }


    public void setState(IntakeState state) {
        this.state = state;
    }

    public enum IntakeState {
        RETRACTED(0.0, 0.0),
        EXTENDED(0.0, 0.5),
        HARVEST(1.0, 0.5);

        IntakeState(Double intakeMotorSpeed, Double extenderMotorPosition) {
            this.intakeMotorSpeed = intakeMotorSpeed;
            this.extenderMotorPosition = extenderMotorPosition;
        }

        final double intakeMotorSpeed;
        final double extenderMotorPosition;
    }
}
