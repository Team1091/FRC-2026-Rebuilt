package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final PIDController hoodController = new PIDController(2.0, 0, 0);
    private double hoodAngle;

    HoodSubsystem(){
        hoodAngle = 0;
        if (Constants.Hood.disabled) {
            hoodMotor = null;
            hoodEncoder = null;
            return;
        }
        hoodMotor = new SparkMax(Constants.Hood.hoodMotorChannel, SparkLowLevel.MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
    }

    @Override
    public void periodic(){
        if (Constants.Hood.disabled){
            return;
        }
        var hoodPow = hoodController.calculate(hoodEncoder.getPosition(), hoodAngle);
        hoodPow = MathUtil.clamp(hoodPow, -Constants.Hood.hoodMotorPower, Constants.Hood.hoodMotorPower);
        hoodMotor.set(hoodPow);
    }
}


