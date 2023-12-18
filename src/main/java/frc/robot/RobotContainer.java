// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.holoDrive;

public class RobotContainer {

  private final holoDrive m_HoloDrive = new holoDrive();

  public XboxController Driver = new XboxController(0);

  public JoystickButton A_button = new JoystickButton(Driver, XboxController.Button.kA.value); 
  public JoystickButton x_Button = new JoystickButton(Driver, XboxController.Button.kX.value);

  public RobotContainer() {

    m_HoloDrive.setDefaultCommand(
      new Drive(m_HoloDrive,
      () -> Driver.getLeftY(),
      () -> Driver.getRightX(),
      () -> Driver.getLeftX(),
      () -> x_Button.getAsBoolean())
    );

    configureBindings();
  }

  private void configureBindings() {


    A_button.toggleOnTrue(new InstantCommand(() -> m_HoloDrive.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
