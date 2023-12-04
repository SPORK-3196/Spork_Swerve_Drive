// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoy;
import frc.robot.commands.FieldOriented;
import frc.robot.commands.LightingModes.Spork;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public final static XboxController primaryController = new XboxController(Constants.kDriverControllerPort);
  
  public static double PLJS_X = primaryController.getLeftX();
  public static double PLJS_Y = primaryController.getLeftY();
  public static double PRJS_X = primaryController.getRightX();
  public static double PRJS_Y = primaryController.getRightY();
  public static JoystickButton X_Button = new JoystickButton(primaryController, XboxController.Button.kX.value);
  public static JoystickButton Y_Button = new JoystickButton(primaryController, XboxController.Button.kY.value);
  public static JoystickButton B_Button = new JoystickButton(primaryController, XboxController.Button.kB.value);
  public static boolean fieldOriented = false;
  private final Lights lighting = new Lights();

  public RobotContainer() {
    configureBindings();
    swerveSubsystem.setDefaultCommand(new DriveWithJoy(swerveSubsystem,
    () -> -primaryController.getRawAxis(Axis.kLeftY.value),
    () -> primaryController.getRawAxis(Axis.kLeftX.value),
    () -> primaryController.getRawAxis(Axis.kRightX.value),
    () -> !fieldOriented));
}

  private void configureBindings() {
    X_Button.whileTrue(new FieldOriented());
    B_Button.whileTrue(new Spork(lighting));
    Y_Button.whileTrue(new InstantCommand(swerveSubsystem::zeroHeading,swerveSubsystem));
  }
  public Command getAutonomousCommand(){
      TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.kDriveKinematics);
      Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), List.of(
        new Translation2d(1,0),
        new Translation2d(0,-1)
      ), new Pose2d(1,-1, new Rotation2d(180)) , config);
      

      PIDController xController = new PIDController(Constants.kPXController, 0, 0);
      PIDController yController = new PIDController(Constants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPThetaController, PLJS_Y, PLJS_X, Constants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI); 


      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traj, swerveSubsystem::getPose2d, Constants.kDriveKinematics, xController,yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
    
    
      return new SequentialCommandGroup(new InstantCommand(()->swerveSubsystem.resetOdometry()),
      swerveControllerCommand,
      new InstantCommand(()-> swerveSubsystem.stopModules()));
  }

}