package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.83);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    // TODO: remove this
    // private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Floor() {
        // TODO: remove this
        // motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake)
                )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(120))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(30))
                                .withSupplyCurrentLimitEnable(true)
                );

        // TODO: remove this
        // motor.getConfigurator().apply(config);
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        // TODO: remove this
//        motor.setControl(
//                voltageRequest
//                        .withOutput(speed.voltage())
//        );
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        // TODO: remove this
//        builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
//        builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
//        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
