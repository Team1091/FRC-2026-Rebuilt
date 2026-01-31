package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// climbs up, holds, and back down
public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax leftClimberMotor;
    private final SparkMax rightClimberMotor;
    private final RelativeEncoder leftClimberEncoder;
    private final RelativeEncoder rightClimberEncoder;
    private double leftClimberSpeed;
    private double rightClimberSpeed;

    public ClimberSubsystem(){
        leftClimberMotor = new SparkMax(9,SparkLowLevel.MotorType.kBrushless);
        rightClimberMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
        leftClimberEncoder = leftClimberMotor.getEncoder();
        rightClimberEncoder = rightClimberMotor.getEncoder();
        leftClimberSpeed = 0;
        rightClimberSpeed = 0;
    }

    public void setClimberSpeed(double speed){
        leftClimberSpeed = speed;
        rightClimberSpeed = speed;
    }

}
