// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.characterization.WheelCharacterization;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.DriveMode;

public class RobotContainer {

  private CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private SwerveDrive m_driveBase = SwerveDrive.getInstance();

  public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");

  public RobotContainer() {
    m_driveBase.setDefaultCommand(
        new TeleopSwerve(
            m_driverController,
            OperatorConstants.kThrottleAxis,
            OperatorConstants.kStrafeAxis,
            OperatorConstants.kSteerAxis,
            OperatorConstants.kDefaultSpeed,
            true,
            true));

    configureBindings();
  }

  private void configureBindings() {

    m_driverController
        .leftBumper()
        .whileTrue(
            new TeleopSwerve(
                m_driverController,
                OperatorConstants.kThrottleAxis,
                OperatorConstants.kStrafeAxis,
                OperatorConstants.kSteerAxis,
                OperatorConstants.kSlowSpeed,
                true,
                true));

    m_driverController
        .a()
        .onTrue(Commands.runOnce(() -> m_driveBase.setDriveMode(DriveMode.XWHEELS), m_driveBase));

    // m_driverController
    //     .b()
    //     .onTrue(Commands.runOnce(() -> m_driveBase.setAngleToSnap(90), m_driveBase));

    // m_driverController.b().onTrue(new TurnToAngle(100));
    // m_driverController.x().onTrue(new TurnToAngle(200));

    m_driverController.y().onTrue(new WheelCharacterization(m_driveBase));

    m_driverController
        .start()
        .onTrue(m_driveBase.characterizeDrivebase(() -> m_driverController.back().getAsBoolean()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
