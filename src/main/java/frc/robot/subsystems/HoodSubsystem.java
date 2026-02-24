package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private final VictorSP hoodMotor;

    private final DigitalInput limitSwitchLow;
    private final DigitalInput limitSwitchHigh;

    private double estimatedHoodAngle; // where we think we are
    private double desiredHoodAngle;   // where we want to be

    private double lastTimeSeconds = Timer.getFPGATimestamp();

    public HoodSubsystem() {
        estimatedHoodAngle = Constants.Hood.minAngleDeg;
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
        estimatedHoodAngle = Constants.Hood.minAngleDeg; // assume reset to the low mechanical stop
        desiredHoodAngle = estimatedHoodAngle;
    }

    public boolean isCloseEnoughToTarget() {
        if (Constants.Hood.disabled) return true;
        return Math.abs(estimatedHoodAngle - desiredHoodAngle) < Constants.Hood.angleCloseEnough;
    }

    public void setTargetAngle(double hoodAngle) {
        // Clamp the target within safe bounds
        if (Constants.Hood.disabled) return;
        // set the angle, make sure we dont go over or under (though maybe we dont need this with the limit switches)
        this.desiredHoodAngle = Math.max(Constants.Hood.minAngleDeg, Math.min(Constants.Hood.maxAngleDeg, hoodAngle));
    }

    @Override
    public void periodic() {
        if (Constants.Hood.disabled) return;

        // Calculate elapsed time using FPGA timestamp for consistent robot timing
        double currentTimeSeconds = Timer.getFPGATimestamp();
        double deltaTimeSeconds = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;
        if (deltaTimeSeconds <= 0) {
            // I dont think this should happen
            return;
        }


        if (limitSwitchLow.get() && desiredHoodAngle <= estimatedHoodAngle) {
            hoodMotor.set(0);
            estimatedHoodAngle = Constants.Hood.minAngleDeg;
            return;
        } else if (limitSwitchHigh.get() && desiredHoodAngle >= estimatedHoodAngle) {
            hoodMotor.set(0);
            estimatedHoodAngle = Constants.Hood.maxAngleDeg;
            return;
        }

        if (isCloseEnoughToTarget()) {
            hoodMotor.set(0);
            estimatedHoodAngle = desiredHoodAngle; // snap to target to avoid dithering
            return;
        }

        // Move toward the target
        double step = deltaTimeSeconds * Constants.Hood.speedDegPerSec;
        if (desiredHoodAngle > estimatedHoodAngle) {
            estimatedHoodAngle = Math.min(desiredHoodAngle, estimatedHoodAngle + step);
            hoodMotor.set(Constants.Hood.hoodMotorPower);
        } else {
            estimatedHoodAngle = Math.max(desiredHoodAngle, estimatedHoodAngle - step);
            hoodMotor.set(-Constants.Hood.hoodMotorPower);
        }
    }
}


