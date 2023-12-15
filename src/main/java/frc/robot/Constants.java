package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants{

    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(25);
    private static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(25);

    public static class kSwerve{

        public static class Offsets {

            public static double frontLeft = -Math.toDegrees(0);
            public static double backLeft = -Math.toDegrees(0);
            public static double backRight = -Math.toDegrees(0);
            public static double frontRight = -Math.toDegrees(0);
          }
        
        public static final int frontLeftDrive = 1;
        public static final int frontLeftSteer =5;
        public static final int backLeftDrive = 2;
        public static final int backLeftSteer = 6;
        public static final int backRightDrive = 4;
        public static final int backRightSteer = 8;
        public static final int frontRightDrive = 3;
        public static final int frontRightSteer = 7;
  
        public static int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
                // Front left
                new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Front right
                new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Back left
                new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Back right
                new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
        );
    }}