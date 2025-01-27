// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.characterization;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;
import org.littletonrobotics.junction.Logger;

public class WheelCharacterization extends Command {

  final double kRotations = 6;
  double kTurnSpeed = 0.1;

  double m_wheelRadiusEstimation = 0.0;
  SwerveDrive m_drivebase;

  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(0.01);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;
  double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  /** Creates a new WheelRadiusCharacterization. */
  public WheelCharacterization(SwerveDrive drivebase) {
    m_drivebase = drivebase;

    addRequirements(m_drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastGyroYawRads = m_drivebase.getYaw().getRadians();
    accumGyroYawRads = 0.0;

    startWheelPositions = new double[4];
    for (int i = 0; i < 4; i++) {
      startWheelPositions[i] = m_drivebase.getModulePositions()[i].distanceMeters;
    }

    omegaLimiter.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.drive(0, 0, omegaLimiter.calculate(kTurnSpeed),  true);

    accumGyroYawRads += MathUtil.angleModulus(m_drivebase.getYaw().getRadians() - lastGyroYawRads);
    lastGyroYawRads = m_drivebase.getYaw().getRadians();
    double averageWheelPosition = 0.0;

    for (int i = 0; i < 4; i++) {
      averageWheelPosition +=
          Math.abs(m_drivebase.getModulePositions()[i].distanceMeters - startWheelPositions[i]);
    }

    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius =
        (accumGyroYawRads * DriveConstants.kDrivebaseRadius) / averageWheelPosition;

    Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.drive(0, 0, 0,  true);

    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return accumGyroYawRads >= (Math.PI * 2.0) * kRotations;
  }
}
