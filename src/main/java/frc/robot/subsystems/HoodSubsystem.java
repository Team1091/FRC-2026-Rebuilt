package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private final VictorSP hoodMotor;

    private final DigitalInput limitSwitchLow;
    private final DigitalInput limitSwitchHigh;

    private double estimatedHoodAngle; // where we think we are
    private double desiredHoodAngle; // where we want to be

    private long lastTimeMs = System.currentTimeMillis();

    private static double MIN_HOOD_ANGLE = 0.0;
    private static double MAX_HOOD_ANGLE = 100.0;
    private static double SPEED_PER_SECOND = 50.0; // 50 here means it will go up or down in 2 sec

    public HoodSubsystem() {
        estimatedHoodAngle = 0;
        if (Constants.Hood.disabled) {
            hoodMotor = null;
            limitSwitchLow = null;
            limitSwitchHigh = null;
            return;
        }

        hoodMotor = new VictorSP(Constants.Hood.hoodMotorChannel);
        limitSwitchLow = new DigitalInput(Constants.Hood.hoodLimitSwitchLow);
        limitSwitchHigh = new DigitalInput(Constants.Hood.hoodLimitSwitchHigh);
    }

    public void resetEncoder() {
        if (Constants.Hood.disabled) return;
        estimatedHoodAngle = 0.0; // start down, maybe we
    }

    public boolean isCloseEnoughToTarget() {
        if (Constants.Hood.disabled) {
            return true;
        }
        return Math.abs(estimatedHoodAngle - desiredHoodAngle) < Constants.Hood.angleCloseEnough;
    }

    public void setTargetAngle(double hoodAngle) {
        this.desiredHoodAngle = hoodAngle;
    }

    @Override
    public void periodic() {
        if (Constants.Hood.disabled) {
            return;
        }

        // We are doing timing inside here, each time it gets through we calculate how long it's been
        long currentTimeMs = System.currentTimeMillis();
        double deltaTimeSeconds = ((double) (lastTimeMs - currentTimeMs)) / 1000.0;
        lastTimeMs = currentTimeMs;


        if (limitSwitchLow.get() && desiredHoodAngle <= estimatedHoodAngle) {
            // we are hitting the low limit, stop
            hoodMotor.set(0);
            estimatedHoodAngle = MIN_HOOD_ANGLE;
            return;

        } else if (limitSwitchHigh.get() && desiredHoodAngle >= estimatedHoodAngle) {
            // we are hitting the high limit, stop
            hoodMotor.set(0);
            estimatedHoodAngle = MAX_HOOD_ANGLE;
            return;
        } else if (isCloseEnoughToTarget()) {
            hoodMotor.set(0);
        } else {
            // we are moving to the target

            if (desiredHoodAngle > estimatedHoodAngle) {
                estimatedHoodAngle += deltaTimeSeconds * SPEED_PER_SECOND;
                hoodMotor.set(1);
            } else {
                estimatedHoodAngle -= deltaTimeSeconds * SPEED_PER_SECOND;
                hoodMotor.set(-1);
            }
        }

    }

}


