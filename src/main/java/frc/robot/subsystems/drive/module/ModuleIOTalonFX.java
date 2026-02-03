// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final SparkMax driveSparkMax;
    private final SparkMax turnSparkMax;
    private final SparkBaseConfig driveConfig;
    private final SparkBaseConfig turnConfig;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final CoreCANcoder cancoder;

    private final GenericEntry absoluteEncoderReading;
    private final String title;


    private final StatusSignal<Angle> turnAbsolutePosition;

    private final boolean isTurnMotorInverted = true;
    private final boolean isDriveMotorInverted = false;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(Constants.ModuleConfig config) {
        driveSparkMax = new SparkMax(config.driveId(), SparkLowLevel.MotorType.kBrushless);
        turnSparkMax = new SparkMax(config.turnId(), SparkLowLevel.MotorType.kBrushless);
        cancoder = new CoreCANcoder(config.cancoderId());
        absoluteEncoderOffset = config.absoluteEncoderOffset();
        title = config.title();

        var tab = Shuffleboard.getTab(title);
        absoluteEncoderReading = tab.add("Absolute Encoder Reading " + title, 0).getEntry();

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        turnAbsolutePosition = cancoder.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

        // Drive
        driveSparkMax.setCANTimeout(Constants.Swerve.Module.CAN_TIMEOUT_MS);
        driveEncoder = driveSparkMax.getEncoder();
        driveEncoder.setPosition(0.0);
        driveConfig = configureSparkMax();
        driveConfig.smartCurrentLimit(Constants.Swerve.Module.DRIVE_CURRENT_LIMIT_AMPS);
        driveConfig.inverted(isDriveMotorInverted);
        driveSparkMax.configure(driveConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Turn
        turnSparkMax.setCANTimeout(Constants.Swerve.Module.CAN_TIMEOUT_MS);
        turnRelativeEncoder = turnSparkMax.getEncoder();
        turnConfig = configureSparkMax();
        turnConfig.smartCurrentLimit(Constants.Swerve.Module.TURN_CURRENT_LIMIT_AMPS);
        turnConfig.inverted(isTurnMotorInverted);
        turnSparkMax.configure(turnConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private SparkMaxConfig configureSparkMax() {
        var config = new SparkMaxConfig();
        config.voltageCompensation(Constants.Swerve.Module.VOLTAGE_COMPENSATION);
        config.encoder.uvwMeasurementPeriod(Constants.Swerve.Module.ENCODER_MEASUREMENT_PERIOD_MS);
        config.encoder.uvwAverageDepth(Constants.Swerve.Module.ENCODER_AVERAGE_DEPTH);
        return config;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        absoluteEncoderReading.setDouble(cancoder.getPosition().getValueAsDouble());

        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / Constants.Swerve.Module.DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / Constants.Swerve.Module.DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[]{driveSparkMax.getOutputCurrent()};

        inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / Constants.Swerve.Module.TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / Constants.Swerve.Module.TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[]{turnSparkMax.getOutputCurrent()};

        BaseStatusSignal.refreshAll(turnAbsolutePosition);
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()).minus(absoluteEncoderOffset);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveConfig.idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnConfig.idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    }
}