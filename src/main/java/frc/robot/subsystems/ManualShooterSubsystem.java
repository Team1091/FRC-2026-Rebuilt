package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// picks balls off the ground
public class ManualShooterSubsystem extends SubsystemBase {

    private final SparkFlex leftShooterMotor;
    private final SparkFlex rightShooterMotor;
    private double speed;

    public ManualShooterSubsystem() {
        if (Constants.Intake.disabled) {
            leftShooterMotor = null;
            rightShooterMotor = null;

            return;
        }

        // set up motors

        leftShooterMotor = new SparkFlex(Constants.Shooter.leftMotorChannel, SparkLowLevel.MotorType.kBrushless);
        rightShooterMotor = new SparkFlex(Constants.Shooter.rightMotorChannel, SparkLowLevel.MotorType.kBrushless);
        var rightConfig = new SparkFlexConfig();
        rightConfig.inverted(true);
        rightShooterMotor.configure(rightConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        speed = 0.0;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        if (Constants.Intake.disabled) return;

        // set motor speeds
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);


    }
}