package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class constants {

    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(25);
    private static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(25);
    
    public static final double kDeadband = 0.01;

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double WheelCircumference = wheelDiameter * Math.PI;
        public static final double RotationGearRatio = (150/7)/1;
        public static final double DriveGearRatio = 8.14/1;

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
        public static int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static int kFrontRightDriveAbsoluteEncoderPort = 4;
        public static int kBackRightDriveAbsoluteEncoderPort = 1;

        // Swerve module Offsets
        public static double FlOffset = Units.rotationsToDegrees(0);  // from offset in rotations
        public static double FrOffset = Units.rotationsToDegrees(0);  //  if in degrees remove and put the raw degree offset
        public static double BlOffset = Units.rotationsToDegrees(0);
        public static double BrOffset = Units.rotationsToDegrees(0);
        

        public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
                // Front left
                new Translation2d(constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Front right
                new Translation2d(constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Back left
                new Translation2d(-constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                // Back right
                new Translation2d(-constants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                        -constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
        );
    


}
