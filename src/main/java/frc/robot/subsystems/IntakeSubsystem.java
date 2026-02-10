package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.IntakeState;

// picks balls off the ground
public class IntakeSubsystem extends SubsystemBase {

    private IntakeState state = IntakeState.RETRACTED;

    private final SparkMax intakeMotor;
    private final SparkMax extenderMotor;
    private final RelativeEncoder extenderEncoder;
    private final PIDController extenderController = new PIDController(2.0, 0, 0);

    public IntakeSubsystem() {
        if (Constants.Intake.disabled) {
            intakeMotor = null;
            extenderMotor = null;
            extenderEncoder = null;
            return;
        }

        // set up motors
        intakeMotor = new SparkMax(Constants.Intake.intakeMotorChannel, SparkLowLevel.MotorType.kBrushless);

        extenderMotor = new SparkMax(Constants.Intake.extenderMotorChannel, SparkLowLevel.MotorType.kBrushless);
        extenderEncoder = extenderMotor.getEncoder();
        extenderEncoder.setPosition(0.0);
    }

    public void resetEncoder(){
        if(Constants.Intake.disabled) return;
        extenderEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        if (Constants.Intake.disabled) return;

        // set motor speeds
        intakeMotor.set(state.intakeMotorSpeed);

        var extenderPow = extenderController.calculate(extenderEncoder.getPosition(), state.extenderMotorPosition);
        extenderPow = MathUtil.clamp(extenderPow, -Constants.Intake.extenderMotorPower, Constants.Intake.extenderMotorPower);
        extenderMotor.set(extenderPow);
    }


    public void setState(IntakeState state) {
        this.state = state;
    }

}
