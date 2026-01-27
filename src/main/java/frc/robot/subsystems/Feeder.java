package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Neo;
import frc.robot.Ports;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Feeder extends SubsystemBase {
    public enum Speed {
        FEED(5000);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    // TODO: undo this
    // private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Feeder() {
        // TODO: undo this
        // motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast)
                )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(120))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(50))
                                .withSupplyCurrentLimitEnable(true)
                )
                .withSlot0(
                        new Slot0Configs()
                                .withKP(1)
                                .withKI(0)
                                .withKD(0)
                                .withKV(12.0 / Neo.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
                );

        // TODO: undo this
        // motor.getConfigurator().apply(config);
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
// TODO: undo this
//         motor.setControl(
//                velocityRequest
//                        .withVelocity(speed.angularVelocity())
//        );
    }

    public void setPercentOutput(double percentOutput) {
// TODO: undo this
//         motor.setControl(
//                voltageRequest
//                        .withOutput(Volts.of(percentOutput * 12.0))
//        );
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
// TODO: undo this
//        builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
//        builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
//        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
