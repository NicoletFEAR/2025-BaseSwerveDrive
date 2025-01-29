// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;



import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveDrive extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotor,
      DriveConstants.kFrontLeftSteerMotor,
      DriveConstants.kFrontLeftSteerOffset);
  private static SwerveDrive m_instance;
  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotor,
      DriveConstants.kFrontRightSteerMotor,
      DriveConstants.kFrontRightSteerOffset);
  private DriveMode m_driveMode = DriveMode.TELEOP;
  private final SwerveModule m_backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotor,
      DriveConstants.kBackLeftSteerMotor,
      DriveConstants.kBackLeftSteerOffset);

  private final SwerveModule m_backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotor,
      DriveConstants.kBackRightSteerMotor,
      DriveConstants.kBackRightSteerOffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonId);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValue().abs(Degrees)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public SwerveDrive() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putNumber("yaw", 0);
    m_gyro.setYaw(0);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValue().abs(Degrees)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

        SmartDashboard.putNumber("yaw", getYaw().getDegrees());
  }
  public static SwerveDrive getInstance() {
    if (m_instance == null) {
      m_instance = new SwerveDrive();
    }

    return m_instance;
  }

  public void zeroYaw() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getYaw(){
    return m_gyro.getRotation2d();
  }
public SwerveModulePosition[] getModulePositions() {return new SwerveModulePosition[]{m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition() };}
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValue().abs(Degrees)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }
public void runVolts(Voltage voltage) {m_frontLeft.runVolts(voltage, 0);
  m_frontRight.runVolts(voltage, 0);
  m_backLeft.runVolts(voltage, 0);
  m_backRight.runVolts(voltage, 0);}
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
     if (xSpeed + ySpeed + rot != 0 && m_driveMode == DriveMode.XWHEELS) {
      m_driveMode = DriveMode.TELEOP;
    }

    if (m_driveMode == DriveMode.TELEOP) {
      // Convert the commanded speeds into the correct units for the drivetrain
      double xSpeedDelivered = xSpeed * DriveConstants.kMaxModuleSpeed;
      double ySpeedDelivered = ySpeed * DriveConstants.kMaxModuleSpeed;
      double rotDelivered = rot * -1 * DriveConstants.kMaxRotationsPerSecond;

      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  getYaw())
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxModuleSpeed);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_backLeft.setDesiredState(swerveModuleStates[2]);
      m_backRight.setDesiredState(swerveModuleStates[3]);
    } else {
      SwerveModuleState[] swerveModuleStates = DriveConstants.kXWheels.clone();
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxModuleSpeed);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_backLeft.setDesiredState(swerveModuleStates[2]);
      m_backRight.setDesiredState(swerveModuleStates[3]);
    }
  }
public enum DriveMode {TELEOP, XWHEELS}
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxModuleSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }
 public void setDriveMode(DriveMode driveMode) {
    m_driveMode = driveMode;
  }
  
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValue().abs(Degrees)).getDegrees();
  }
  public Command characterizeDrivebase(BooleanSupplier finishRoutine) {
    var sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage), null, this));

    return new SequentialCommandGroup(
        new PrintCommand("Starting"),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(finishRoutine),
        new WaitCommand(1.0),
        new PrintCommand("Starting"),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(finishRoutine),
        new PrintCommand("Starting"),
        new WaitCommand(1.0),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(finishRoutine),
        new PrintCommand("Starting"),
        new WaitCommand(1.0),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(finishRoutine));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValue().abs(DegreesPerSecond);
  }
}
