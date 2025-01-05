// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveMovementCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.aesthetic.Colors;
import frc.robot.subsystems.aesthetic.Colors.Effect;
import frc.robot.util.AgnosticController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveDriveSubsystem);
  // REN CODE!!!
  private final Colors m_color = new Colors();
  // private final Music m_music = new Music();

  private final AgnosticController m_controller = new AgnosticController();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_color.startRGB(Effect.CHROMA);
  }

  public void configureBindings() {
    SwerveMovementCommand movementCommand = new SwerveMovementCommand(
            m_swerveDriveSubsystem,
            () -> -m_controller.getLeftY(),
            () -> -m_controller.getLeftX(),
            () -> -m_controller.getRightX()
    );

    m_swerveDriveSubsystem.setDefaultCommand(
            movementCommand
    );
    m_controller.resetHeading().onTrue(m_swerveDriveSubsystem.runOnce(() -> m_swerveDriveSubsystem.seedFieldRelative()));

    SmartDashboard.putData(movementCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
