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
    private final PIDController leftClimberController = new PIDController(2.0, 0, 0);
    private final PIDController rightClimberController = new PIDController(2.0, 0, 0);

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
    }

    public void setClimberPosition(ClimberPosition position) {
        climberPosition = position;
    }

    @Override
    public void periodic() {
        if (Constants.Climber.disabled) return;

        var leftClimberPow = leftClimberController.calculate(leftClimberEncoder.getPosition(), climberPosition.climberPosition);
        leftClimberPow = MathUtil.clamp(leftClimberPow, -Constants.Climber.leftMotorPower, Constants.Climber.leftMotorPower);
        leftClimberMotor.set(leftClimberPow);

        var rightClimberPow = rightClimberController.calculate(rightClimberEncoder.getPosition(), climberPosition.climberPosition);
        rightClimberPow = MathUtil.clamp(rightClimberPow, -Constants.Climber.rightMotorPower, Constants.Climber.rightMotorPower);
        rightClimberMotor.set(rightClimberPow);
    }
}
