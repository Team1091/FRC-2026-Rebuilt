package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// picks balls off the ground
public class LoaderSubsystem extends SubsystemBase {

    private final SparkMax loaderMotor;
    private double speed;

    public LoaderSubsystem() {
        if (Constants.Intake.disabled) {
            loaderMotor = null;
            return;
        }

        // set up motors
        loaderMotor = new SparkMax(Constants.Loader.loaderMotorChannel, SparkLowLevel.MotorType.kBrushless);
        speed = 0.0;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        if (Constants.Intake.disabled) return;

        // set motor speeds
        loaderMotor.set(speed);

    }
}