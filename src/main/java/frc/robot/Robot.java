// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Drive;
import frc.robot.Subsys.Swerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public Swerve m_swerve = new Swerve();
  public XboxController Driver = new XboxController(0);


  @Override
  public void robotInit() {
    configureBindings();

    
  }
  

  private void configureBindings() {
    m_swerve.setDefaultCommand(new Drive(m_swerve,
     () -> -modifyAxis(Driver.getLeftX()),
      () -> -modifyAxis(Driver.getLeftY()),
       () -> -modifyAxis(Driver.getRightX())));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();


    OI.modules.backLeftAngle = m_swerve.backLeft.getTurnPos().getRadians();
    OI.modules.backRightAngle = m_swerve.backRight.getTurnPos().getRadians();
    OI.modules.frontLeftAngle = m_swerve.frontLeft.getTurnPos().getRadians();
    OI.modules.frontRightAngle = m_swerve.frontRight.getTurnPos().getRadians();


    OI.modules.backLeftAngleEntry.setDouble(OI.modules.backLeftAngle);
    OI.modules.backRightAngleEntry.setDouble(OI.modules.backRightAngle);
    OI.modules.frontLeftAngleEntry.setDouble(OI.modules.frontLeftAngle);
    OI.modules.frontRightAngleEntry.setDouble(OI.modules.frontRightAngle);


    
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private static double modifyAxis(double value) 
  {
    // Deadband
    value = deadband(value, 0.10);

    
    value = Math.copySign(value * value, value);

    return value;
      
    }


    private static double deadband(double value, double deadband) 
    {
     return Math.abs(value) > deadband ? value : 0.0;
    }
}

