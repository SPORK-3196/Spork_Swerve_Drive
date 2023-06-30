// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public static final XboxController primaryController = new XboxController(0);
  public static double RJSX_PRIM = primaryController.getRightX();
  public static double RJSY_PRIM = primaryController.getRightY();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriveWithJoy(swerveSubsystem,
    () -> -primaryController.getLeftX(),
    () -> primaryController.getLeftY(),
    () -> primaryController.getRightX(),
    () -> !primaryController.getAButton()));
    configureBindings();
}

  private void configureBindings() {
    new JoystickButton(primaryController,Button.kX.value).whileTrue(new InstantCommand(swerveSubsystem::zeroHeading,swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
