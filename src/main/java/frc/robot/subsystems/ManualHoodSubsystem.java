package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManualHoodSubsystem extends SubsystemBase {
    private final VictorSPX hoodMotor;

    private double hoodSpeed;

    public ManualHoodSubsystem() {
        if (Constants.Hood.disabled) {
            hoodMotor = null;
            return;
        }
        hoodMotor = new VictorSPX(Constants.Hood.hoodMotorChannel);

        hoodSpeed = 0.0;

    }

    public void setSpeed(double hoodSpeed) {
        this.hoodSpeed = hoodSpeed;

    }

    @Override
    public void periodic() {
        if (Constants.Hood.disabled) return;
        hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);

    }
}
