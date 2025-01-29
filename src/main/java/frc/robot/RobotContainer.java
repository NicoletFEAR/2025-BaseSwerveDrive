// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.characterization.WheelCharacterization;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.DriveMode;

public class RobotContainer {

  private CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

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
        .L1()
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
        .cross()
        .onTrue(Commands.runOnce(() -> m_driveBase.setDriveMode(DriveMode.XWHEELS), m_driveBase));

    // m_driverController
    //     .b()
    //     .onTrue(Commands.runOnce(() -> m_driveBase.setAngleToSnap(90), m_driveBase));

    // m_driverController.b().onTrue(new TurnToAngle(100));
    // m_driverController.x().onTrue(new TurnToAngle(200));

    // m_driverController.circle().onTrue(new WheelCharacterization(m_driveBase));

    // m_driverController
    //     .create()
    //     .onTrue(
    //         m_driveBase.characterizeDrivebase(() -> m_driverController.options().getAsBoolean()));

    m_driverController.create().onTrue(Commands.runOnce(() -> m_driveBase.zeroYaw(), m_driveBase));
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("New Auto");
  }
}
