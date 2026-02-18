package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// picks balls off the ground
public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;
    private double speed;

    public IntakeSubsystem() {
        if (Constants.Intake.disabled) {
            intakeMotor = null;
            return;
        }

        // set up motors
        intakeMotor = new SparkMax(Constants.Intake.intakeMotorChannel, SparkLowLevel.MotorType.kBrushless);
        speed = 0.0;
    }
    public void setSpeed(double speed){
        this.speed = speed;
    }

    @Override
    public void periodic() {
        if (Constants.Intake.disabled) return;

        // set motor speeds
        intakeMotor.set(speed);

    }
}
