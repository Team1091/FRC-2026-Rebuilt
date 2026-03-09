package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// climbs up, holds, and back down
public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax leftClimberMotor;
    private final SparkMax rightClimberMotor;
    private final RelativeEncoder leftClimberEncoder;
    private final RelativeEncoder rightClimberEncoder;
    private double speed;

    public ClimberSubsystem() {
        if (Constants.Climber.disabled) {
            leftClimberMotor = null;
            rightClimberMotor = null;
            leftClimberEncoder = null;
            rightClimberEncoder = null;
            return;
        }

        leftClimberMotor = new SparkMax(Constants.Climber.leftMotorChannel, SparkLowLevel.MotorType.kBrushless);
        rightClimberMotor = new SparkMax(Constants.Climber.rightMotorChannel, SparkLowLevel.MotorType.kBrushless);
        leftClimberEncoder = leftClimberMotor.getEncoder();
        rightClimberEncoder = rightClimberMotor.getEncoder();
        leftClimberEncoder.setPosition(0.0);
        rightClimberEncoder.setPosition(0.0);

    }

    public void resetEncoders() {
        if (Constants.Climber.disabled) return;
        leftClimberEncoder.setPosition(0);
        rightClimberEncoder.setPosition(0);
    }

    public void setClimberSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        if (Constants.Climber.disabled) return;
        SmartDashboard.putNumber("Climber Encoder", leftClimberMotor.getEncoder().getPosition());
        if ((leftClimberEncoder.getPosition() >= Constants.Climber.maxEncoderCount && speed > 0)
                || (leftClimberEncoder.getPosition() <= 0 && speed < 0)
        ) {
            leftClimberMotor.set(0);
        } else {
            leftClimberMotor.set(speed);
        }
        if ((rightClimberEncoder.getPosition() >= Constants.Climber.maxEncoderCount && speed > 0)
                || (rightClimberEncoder.getPosition() <= 0 && speed < 0)
        ) {
            rightClimberMotor.set(0);
        } else {
            rightClimberMotor.set(speed);
        }

    }
}
