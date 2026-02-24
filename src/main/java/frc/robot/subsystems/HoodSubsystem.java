package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private final VictorSP hoodMotor;

    private final DigitalInput limitSwitchLow;
    private final DigitalInput limitSwitchHigh;
    private double approxHoodAngle;

    private static double MIN_HOOD_ANGLE = 100.00;
    private static double MAX_HOOD_ANGLE = 100.00;

    public HoodSubsystem() {
        approxHoodAngle = 0;
        if (Constants.Hood.disabled) {
            hoodMotor = null;
            limitSwitchLow = null;
            limitSwitchHigh = null;
            return;
        }

        hoodMotor = new VictorSP(Constants.Hood.hoodMotorChannel);
        limitSwitchLow = new DigitalInput(Constants.Hood.hoodLimitLow);
        limitSwitchHigh = new DigitalInput(Constants.Hood.hoodLimitHigh);
    }

    public void resetEncoder() {
        if (Constants.Hood.disabled) return;
        approxHoodAngle = 0.0;
//        hoodEncoder.setPosition(0);
    }

    public boolean isCloseEnoughToTarget() {
        if (Constants.Hood.disabled) {
            return true;
        }
        return Math.abs(approxHoodAngle - hoodEncoder.getPosition()) < Constants.Hood.angleCloseEnough;
    }

    public void setTargetAngle(double hoodAngle) {
        this.approxHoodAngle = hoodAngle;
    }

    @Override
    public void periodic() {
        if (Constants.Hood.disabled) {
            return;
        }
        var hoodPow = hoodController.calculate(hoodEncoder.getPosition(), approxHoodAngle);
        hoodPow = MathUtil.clamp(hoodPow, -Constants.Hood.hoodMotorPower, Constants.Hood.hoodMotorPower);
        hoodMotor.set(hoodPow);
    }

}


