package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManualHoodSubsystem extends SubsystemBase {
    private final VictorSPX leftHoodMotor;
    private final VictorSPX rightHoodMotor;

    private double leftSpeed;
    private double rightSpeed;


    public ManualHoodSubsystem() {
        if(Constants.Hood.disabled){
            leftHoodMotor = null;
            rightHoodMotor = null;
            return;
        }
        leftHoodMotor = new VictorSPX(Constants.Hood.leftHoodMotorChannel);
        rightHoodMotor = new VictorSPX(Constants.Hood.rightHoodMotorChannel);

        leftSpeed = 0.0;
        rightSpeed = 0.0;

    }
    public void setSpeed(double leftSpeed, double rightSpeed) {
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;

    }

    @Override
    public void periodic() {
        if (Constants.Hood.disabled) return;
        leftHoodMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightHoodMotor.set(ControlMode.PercentOutput, rightSpeed);

    }
}
