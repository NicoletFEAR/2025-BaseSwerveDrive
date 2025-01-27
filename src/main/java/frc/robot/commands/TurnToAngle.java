// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.Utils;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
  private PIDController angleController = new PIDController(.016, 0.003, 0.0);

  private SwerveDrive m_drivebase = SwerveDrive.getInstance();
  private double m_targetAngle = -1;
  private double deadBand = 2;

  /**
   *
   *
   * <h3>TurnToAngle</h3>
   *
   * Turns to a specified angle using a pid controller
   *
   * @param m_drivebase The swerve drive that moves the robot
   * @param targetAngle Target angle to turn to
   */
  public TurnToAngle(double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_targetAngle = targetAngle;

    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.drive(0, 0, 0,  true);

    angleController.setIZone(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speeds =
        angleController.calculate(
            Utils.getAdjustedYawDegrees(m_drivebase.getYaw().getDegrees(), m_targetAngle), 180);

    speeds = MathUtil.clamp(speeds, -1, 1);
    m_drivebase.drive(0, 0, speeds,  true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(
            Utils.getAdjustedYawDegrees(m_drivebase.getYaw().getDegrees(), m_targetAngle) - 180)
        < deadBand) {
      m_drivebase.drive(0, 0, 0,  true);
      return true;
    } else {
      return false;
    }
  }
}
