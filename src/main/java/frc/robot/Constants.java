package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

public class Constants{   
  public static SimMode simMode = SimMode.DESKTOP;

  // Operator interface constants
  public static class kOI {
    public static double translationDeadzone = 0.08;
    public static double rotationDeadzone = 0.08;
  }

  // Sim Modes
  public enum SimMode {
    HARDWARE,
    DESKTOP_VISION,
    DESKTOP
  }

  // Swerve subsystem constants (module constants included)
  public static class kSwerve {
    public static double width = 28;
    public static double length = 28;

    // Speed & accel limits (m/s, rad/s, m/s^2 & rad/s^2)
    public static double maxTransSpeed = 5;
    public static double maxAngSpeed = 3 * Math.PI;

    public static double maxTransAccel = 0.2 * 9.81;
    public static double maxAngAccel = 2;

    // Operator interface constants
    public static class Teleop {
      public static double translationGain = 0.7;
      public static double rotationGain = 0.5;
    }

    // NavX
    public static boolean invertGyro = false;
    public static Port navxPort = Port.kMXP;

    // Swerve uses ccw+ angular quanities and a coordinate plane with 0,0 at the robot's center
    // , forward is +x, and a module order based on the quadrant system (front left is first)
    public static SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(length / 2, width / 2),
            new Translation2d(-length / 2, width / 2),
            new Translation2d(-length / 2, -width / 2),
            new Translation2d(length / 2, -width / 2));

    // Module angular offsets (deg)
    public static class Offsets {
      public static double frontLeft = 6.04 * (Math.PI/180);
      public static double backLeft = 3.7 * (Math.PI/180);
      public static double backRight = 1.97 * (Math.PI/180);
      public static double frontRight = -1.44 * (Math.PI/180);

      //------------------------------------------------------------------------------------------------------------------
    }
    
    // Controller PID values for x/y translation, and z rotation
    public static class Auton {
      public static final double xP = 0.1;
      public static final double xD = 0;

      public static final double yP = 0.1;
      public static final double yD = 0;

      public static final double zP = 5;
      public static final double zD = 0;

      public static final double maxAccel = 1.5;
      public static final double maxVel = 1.5;
    }

    public static class kModule {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
      public static final int drivePinionTeeth = 13;
      public static final boolean invertSteerEncoder = true;

      // Controls Constants
      public static class kDrive {
        public static final double kP = 0.1;
        public static final double kD = 0;
        public static final double kS = 0.04642;
        public static final double kV = 2.6757;
        public static final double kA = 0.11358;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      public static class kSteer {
        public static final double kP = 1;
        public static final double kD = 0;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      public static final int driveSmartCurrentLimit = 50; // amps
      public static final int driveMaxCurrent = 80; // amps
      public static final int steerSmartCurrentLimit = 20; // amps
      public static final int steerMaxCurrent = 35; // amps

      // Physical dimensions/values
      public static final double wheelDiameter = Units.inchesToMeters(3);
      public static final double wheelCircumference = wheelDiameter * Math.PI; // meters
      public static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);
      public static final double steerMotorReduction = 9424.0 / 203.0;

      // Motor physics
      public static final double neoFreeSpeed = 5820.0 / 60; // rot/s
      public static final double neoFreeCurrent = 1.7; // amps
      public static final double neoStallTorque = 3.28; // Nm
      public static final double neoStallCurrent = 181; // amps

      public static final double neo550FreeSpeed = 11710.0 / 60;
      public static final double neo550FreeCurrent = 1.1;
      public static final double neo550StallTorque = 1.08;
      public static final double neo550StallCurrent = 111;

      public static final double maxWheelSpeed =
          (neoFreeSpeed / driveMotorReduction) * (wheelDiameter * Math.PI); // m/s

      // Encoders
      public static final double drivingEncoderPositionFactor =
          (wheelDiameter * Math.PI) / driveMotorReduction; // meters
      public static final double drivingEncoderVelocityFactor =
          ((wheelDiameter * Math.PI) / driveMotorReduction) / 60.0; // m/s

      public static final double steeringEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double steeringEncoderVelocityFactor = (2 * Math.PI) / 60.0; // rad/s

      public static final double steeringEncoderPositionPIDMinInput = 0; // radians
      public static final double steeringEncoderPositionPIDMaxInput =
          steeringEncoderPositionFactor; // radians
    }

    // Motor CAN IDs
    public static class CANID {
      public static int frontLeftDrive = 1;
      public static int frontLeftSteer =5;
      public static int backLeftDrive = 2;
      public static int backLeftSteer = 6;
      public static int backRightDrive = 4;
      public static int backRightSteer = 8;
      public static int frontRightDrive = 3;
      public static int frontRightSteer = 7;

      public static int kFrontLeftDriveAbsoluteEncoderPort = 1;
      public static int kBackLeftDriveAbsoluteEncoderPort = 3;
      public static int kFrontRightDriveAbsoluteEncoderPort = 2;
      public static int kBackRightDriveAbsoluteEncoderPort = 4;
    }
  }
}

