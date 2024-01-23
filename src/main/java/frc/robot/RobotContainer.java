// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  public Swerve mSwerve = new Swerve();

  public XboxController driver = new XboxController(0);

  public JoystickButton a_Button = new JoystickButton(driver, XboxController.Button.kA.value);

  public RobotContainer() {

    mSwerve.setDefaultCommand(
    mSwerve.teleDrive(
    () -> driver.getLeftY(), 
    () -> driver.getLeftX(), 
    () -> driver.getRightX()));

    configureBindings();
  }

  private void configureBindings() {
    
    a_Button.toggleOnTrue(new InstantCommand(() -> mSwerve.ZeroGyro(), mSwerve));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
