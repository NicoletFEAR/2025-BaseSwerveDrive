// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.DeviceConfigurator;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.Utils;

public class SwerveModule {

  SwerveModuleConstants m_constants;

  private SparkMax m_steerMotor;
  private TalonFX m_driveMotor;
  private CANcoder m_steerAbsEncoder;

  private RelativeEncoder m_steerEncoder;

  private SparkClosedLoopController m_steerController;

  private SwerveModulePosition m_modulePosition;
  private SwerveModuleState m_moduleState;

  private double m_lastSpeed;
  private double m_lastAngle;

  private Rotation2d m_simAngle = new Rotation2d();
  private double m_simDist;
  private double m_simVel;

  public SwerveModule(SwerveModuleConstants constants) {
    m_constants = constants;

    m_steerMotor = new SparkMax(constants.steerId, MotorType.kBrushless);
    m_driveMotor = new TalonFX(constants.driveId);

    m_steerEncoder = m_steerMotor.getEncoder();

    m_steerController = m_steerMotor.getClosedLoopController();

    m_steerAbsEncoder = new CANcoder(constants.steerEncoderId);

    m_modulePosition = new SwerveModulePosition();
    m_moduleState = new SwerveModuleState();

    DeviceConfigurator.configureSparkMaxSteerMotor(m_steerMotor);
    DeviceConfigurator.configureTalonFXDriveMotor(m_driveMotor);
    DeviceConfigurator.configureCANcoder(m_steerAbsEncoder, m_constants.offset);
  }

  public void resetAngleToAbsolute() {
    m_steerEncoder.setPosition(m_steerAbsEncoder.getAbsolutePosition().getValue().abs(Rotations) * 360);
  }

  public Rotation2d getModuleHeading() {
    return m_modulePosition.angle;
  }

  public double getAbsolutePosition() {
    return m_steerAbsEncoder.getAbsolutePosition().getValue().abs(Rotations) * 360;
  }

  public SwerveModulePosition getModulePosition() {
    if (RobotBase.isReal()) {
      m_modulePosition.angle = Rotation2d.fromDegrees(m_steerEncoder.getPosition());
      m_modulePosition.distanceMeters = m_driveMotor.getPosition().getValue().abs(Rotations);
    } else {
      m_modulePosition.angle = m_simAngle;
      m_modulePosition.distanceMeters = m_simDist;
    }
    return m_modulePosition;
  }

  public SwerveModuleState getModuleState() {
    if (RobotBase.isReal()) {
      m_moduleState.angle = Rotation2d.fromDegrees(m_steerEncoder.getPosition());
      m_moduleState.speedMetersPerSecond = m_driveMotor.getVelocity().getValue().abs(RotationsPerSecond);
    } else {
      m_moduleState.angle = m_simAngle;
      m_moduleState.speedMetersPerSecond = m_simVel;
    }
    return m_moduleState;
  }

  public void runVolts(Voltage volts, double position) {
    m_steerController.setReference(position, ControlType.kPosition);
    m_driveMotor.setVoltage(volts.abs(Volts));
  }

  public void setSwerveModuleState(SwerveModuleState moduleState, boolean isOpenLoop) {

    moduleState = Utils.optimize(moduleState, getModuleHeading());

    moduleState.speedMetersPerSecond *= moduleState.angle.minus(getModuleHeading()).getCos();

    if (moduleState.angle.getDegrees() != m_lastAngle) {
      m_steerController.setReference(moduleState.angle.getDegrees(), ControlType.kPosition);
      m_lastAngle = moduleState.angle.getDegrees();
    }

    if (moduleState.speedMetersPerSecond != m_lastSpeed) {
      if (isOpenLoop) {
        m_driveMotor.set(moduleState.speedMetersPerSecond / DriveConstants.kMaxModuleSpeed);
      } else {
        m_driveMotor.setControl(new VelocityVoltage(moduleState.speedMetersPerSecond));
      }
      m_lastSpeed = moduleState.speedMetersPerSecond;
    }

    if (RobotBase.isSimulation()) {
      m_simAngle = moduleState.angle;
      m_simVel = moduleState.speedMetersPerSecond;
      m_simDist += moduleState.speedMetersPerSecond / (1 / Constants.kdt);
    }
  }
}
