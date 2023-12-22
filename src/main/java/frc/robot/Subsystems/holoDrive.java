package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class holoDrive extends SubsystemBase {

    //Hardware
private final AHRS gyro;

public MK4I frontLeft = new MK4I(
        kSwerve.frontLeftDrive,
         kSwerve.frontLeftSteer,
          kSwerve.kFrontLeftDriveAbsoluteEncoderPort,
           kSwerve.Offsets.frontLeft);

// public MK4I frontRight = new MK4I(
//     kSwerve.frontRightDrive,
//      kSwerve.frontRightSteer,
//       kSwerve.kFrontRightDriveAbsoluteEncoderPort,
//        kSwerve.Offsets.frontRight);

// public MK4I backLeft = new MK4I(
//     kSwerve.backLeftDrive,
//      kSwerve.backLeftSteer,
//       kSwerve.kBackLeftDriveAbsoluteEncoderPort,
//        kSwerve.Offsets.backLeft);

// public MK4I backRight = new MK4I(
//     kSwerve.backRightDrive,
//      kSwerve.backRightSteer,
//       kSwerve.kBackRightDriveAbsoluteEncoderPort,
//        kSwerve.Offsets.backRight);

private SwerveDriveOdometry odometry;
private SwerveModuleState[] states;

private Field2d field2d;

public holoDrive(){
        gyro = new AHRS(Port.kMXP);
        zeroGyro();
        
        odometry = new SwerveDriveOdometry(
        kSwerve.kinematics,
        getYaw(), 
        getPositions());
    
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);

    }

    public void drive(Translation2d translation, double rotation, boolean FieldOren, boolean isOpenLoop){
        states = 
            kSwerve.kinematics.toSwerveModuleStates(
                FieldOren 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
            
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kSwerve.MaxSpeedMetersPerSecond);

        frontLeft.setState(states[0], isOpenLoop, translation.getX());
        // frontRight.setState(states[1], isOpenLoop);
        // backLeft.setState(states[2], isOpenLoop);
        // backRight.setState(states[3], isOpenLoop);
    }

    private SwerveModulePosition[] getPositions(){
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            // frontRight.getPosition(),
            // backLeft.getPosition(),
            // backRight.getPosition()
        };
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(getYaw() , getPositions() , pose);
    }

    public SwerveModuleState[] getStates(){
        return new SwerveModuleState[]{
            frontLeft.getState(),
            // frontRight.getState(),
            // backLeft.getState(),
            // backRight.getState()
        };
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw(){
        return new Rotation2d(gyro.getYaw());
    }
}