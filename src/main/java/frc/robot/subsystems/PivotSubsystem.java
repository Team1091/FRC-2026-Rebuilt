package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// picks balls off the ground
public class PivotSubsystem extends SubsystemBase {

    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private double speed;

    public PivotSubsystem() {
        if (Constants.Pivot.disabled) {
            pivotMotor = null;
            pivotEncoder = null;
            return;
        }

        // set up motors

        pivotMotor = new SparkMax(Constants.Pivot.pivotMotorChannel, SparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0.0);
        speed = 0.0;
    }

    public void resetEncoder() {
        if (Constants.Pivot.disabled) return;
        pivotEncoder.setPosition(0);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        if (Constants.Pivot.disabled) return;
        pivotMotor.set(speed);

//
//        // set motor speeds
//        if (pivotEncoder.getPosition() <= Constants.Pivot.extendEncoderPos && speed > 0) {
//            // we are retracted and want to go out.  push out
//            pivotMotor.set(speed);
//        } else if (pivotEncoder.getPosition() > Constants.Pivot.retractEncoderPos && speed < 0) {
//            // We are extended and want to retract.
//            pivotMotor.set(speed);
//        } else {
//            pivotMotor.set(0.0);
//        }
    }
}
