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

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final boolean isTurnMotorInverted;
    private final boolean isDriveMotorInverted;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(Constants.Swerve.ModuleConfig config) {
        this.title = config.title();
        driveSparkMax = new SparkMax(config.driveMotorId(), SparkLowLevel.MotorType.kBrushless);
        turnSparkMax = new SparkMax(config.turnMotorId(), SparkLowLevel.MotorType.kBrushless);
        cancoder = new CoreCANcoder(config.cancoderId());
        absoluteEncoderOffset = config.absoluteEncoderOffset();
        isTurnMotorInverted = config.isTurnMotorInverted();
        isDriveMotorInverted = config.isDriveMotorInverted();

        var tab = Shuffleboard.getTab(title);
        absoluteEncoderReading = tab.add("Absolute Encoder Reading" + title, 0).getEntry();

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        turnAbsolutePosition = cancoder.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                turnAbsolutePosition);


        driveConfig = new SparkMaxConfig();
        turnConfig = new SparkMaxConfig();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        driveConfig.inverted(isDriveMotorInverted);
        turnConfig.inverted(isTurnMotorInverted);
        driveConfig.smartCurrentLimit(40);
        turnConfig.smartCurrentLimit(20);
        driveConfig.voltageCompensation(12.0);
        turnConfig.voltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        driveConfig.encoder.uvwMeasurementPeriod(10);
        driveConfig.encoder.uvwAverageDepth(2);

        turnRelativeEncoder.setPosition(driveEncoder.getPosition() * 2.0 * Math.PI);
        turnConfig.encoder.uvwMeasurementPeriod(10);
        turnConfig.encoder.uvwAverageDepth(2);

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.configure(driveConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnSparkMax.configure(turnConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        absoluteEncoderReading.setDouble(cancoder.getPosition().getValueAsDouble());

        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[]{driveSparkMax.getOutputCurrent()};

        //inputs.turnAbsolutePosition = new Rotation2d(driveEncoder.getPosition() * 2.0 * Math.PI).minus(absoluteEncoderOffset);
        inputs.turnPosition = new Rotation2d(Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO).getRadians() % (Math.PI * 2.0));
        //inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[]{turnSparkMax.getOutputCurrent()};

        BaseStatusSignal.refreshAll(turnAbsolutePosition);
        inputs.turnAbsolutePosition =
                Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble() % 1)
                        .minus(absoluteEncoderOffset);
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