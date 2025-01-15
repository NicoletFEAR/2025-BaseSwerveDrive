// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants.DriveConstants;

public class DeviceConfigurator {
  public static void configureSparkMaxSteerMotor(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true)
          .smartCurrentLimit(40)
          .idleMode(IdleMode.kBrake);

    config.encoder.positionConversionFactor(DriveConstants.kTurnRotationsToDegrees);

    config.closedLoop
          .p(DriveConstants.turnkp)
          .i(DriveConstants.turnki)
          .d(DriveConstants.turnkd)
          .velocityFF(DriveConstants.turnkff);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);
  }

  public static void configureSparkMaxDriveMotor(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true)
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kBrake)
          .openLoopRampRate(DriveConstants.driverampRate);

    config.encoder.positionConversionFactor(DriveConstants.kDriveRevToMeters)
                  .velocityConversionFactor(DriveConstants.kDriveRpmToMetersPerSecond);

    config.closedLoop.p(DriveConstants.drivekp)
                     .i(DriveConstants.driveki)
                     .d(DriveConstants.drivekd)
                     .velocityFF(DriveConstants.drivekff);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);
  }

  public static void configureSparkFlexDriveMotor(SparkFlex motor) {
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(true)
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kBrake)
          .openLoopRampRate(DriveConstants.driverampRate);

    config.encoder.positionConversionFactor(DriveConstants.kDriveRevToMeters)
                  .velocityConversionFactor(DriveConstants.kDriveRpmToMetersPerSecond);

    config.closedLoop.p(DriveConstants.drivekp)
                     .i(DriveConstants.driveki)
                     .d(DriveConstants.drivekd)
                     .velocityFF(DriveConstants.drivekff);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);
  }

  public static void configureCANcoder(CANcoder encoder, double offset) {
    CANcoderConfiguration configuration = new CANcoderConfiguration();

    encoder.getConfigurator().apply(configuration);

    configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    configuration.MagnetSensor.MagnetOffset = offset;
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    encoder.getConfigurator().apply(configuration);
  }
}
