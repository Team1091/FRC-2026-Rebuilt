package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ClimberPosition;

// climbs up, holds, and back down
public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax leftClimberMotor;
    private final SparkMax rightClimberMotor;
    private final RelativeEncoder leftClimberEncoder;
    private final RelativeEncoder rightClimberEncoder;
    private ClimberPosition climberPosition;
    private double climberTarget;
    private final PIDController leftClimberController = new PIDController(1.0, 0, 0);
    private final PIDController rightClimberController = new PIDController(1.0, 0, 0);

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
        climberPosition = ClimberPosition.DOWN;
        climberTarget = climberPosition.position;
    }

    public void resetEncoders() {
        if(Constants.Climber.disabled) return;
        leftClimberEncoder.setPosition(0);
        rightClimberEncoder.setPosition(0);
    }

    public void setClimberPosition(ClimberPosition position) {
        climberPosition = position;
    }

    @Override
    public void periodic() {
        if (Constants.Climber.disabled) return;

        if (Math.abs(climberTarget - climberPosition.position) < Constants.Climber.climbingSpeed) {
            climberTarget = climberPosition.position;
        } else if (climberTarget < climberPosition.position) {
            climberTarget += Constants.Climber.climbingSpeed;
        } else {
            climberTarget -= Constants.Climber.climbingSpeed;
        }

        var leftClimberPow = leftClimberController.calculate(leftClimberEncoder.getPosition(), climberTarget);
        leftClimberPow = MathUtil.clamp(leftClimberPow, -Constants.Climber.leftMotorPower, Constants.Climber.leftMotorPower);
        leftClimberMotor.set(leftClimberPow);

        var rightClimberPow = rightClimberController.calculate(rightClimberEncoder.getPosition(), climberTarget);
        rightClimberPow = MathUtil.clamp(rightClimberPow, -Constants.Climber.rightMotorPower, Constants.Climber.rightMotorPower);
        rightClimberMotor.set(rightClimberPow);
    }
}
