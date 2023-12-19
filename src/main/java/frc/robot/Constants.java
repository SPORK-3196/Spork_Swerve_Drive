package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(25);
    private static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(25);

    public static class kSwerve{
      
//TODO offsets
    public static class Offsets {

        public static Rotation2d frontLeft = Rotation2d.fromRadians(0);//-Units.radiansToDegrees(-0.24);
        public static Rotation2d backLeft = Rotation2d.fromRadians(0);//-Units.radiansToDegrees(0.54);
        public static Rotation2d backRight = Rotation2d.fromRadians(0);//-Units.radiansToDegrees(1.97);
        public static Rotation2d frontRight = Rotation2d.fromRadians(0);//-Units.radiansToDegrees(-1.2);
        }

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double WheelCircumference = wheelDiameter * Math.PI;
//TODO change the gear ratio
        public static final double RotationGearRatio = 6.5 / 1;//6.5:1
        public static final double DriveGearRatio = 6.5/1;

        public static final double DrivePositionCoversionFactor = WheelCircumference / DriveGearRatio;
        public static final double DriveVelocityConversionFactor = DrivePositionCoversionFactor / 60;

        public static final double AngleConversionFactor = 360 / RotationGearRatio;
        
        //Max Speeds (Vroom Vroom)
        public static final double MaxSpeedMetersPerSecond = 4.5;
        public static final double MaxTranslationX = 1;
        public static final double MaxTranslationY = 1;
        public static final double Maxrotation = 11.5;
        
        //Spark Max IDs 
        public static final int frontLeftDrive = 1;
        public static final int frontLeftSteer =5;
        public static final int backLeftDrive = 2;
        public static final int backLeftSteer = 6;
        public static final int backRightDrive = 4;
        public static final int backRightSteer = 8;
        public static final int frontRightDrive = 3;
        public static final int frontRightSteer = 7;
  
        //CRTE CANcoder IDs
        public static int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final SwerveDriveKinematics kinematics =
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
    }
}
