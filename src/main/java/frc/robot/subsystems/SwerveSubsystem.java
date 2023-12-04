package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.speedconst;
import frc.robot.Constants.swervemoduleconst;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        swervemoduleconst.kFrontLeftDriveMotorPort,
        swervemoduleconst.kFrontLeftTurningMotorPort,
        swervemoduleconst.kFrontLeftDriveEncoderReversed,
        swervemoduleconst.kFrontLeftTurningEncoderReversed,
        swervemoduleconst.kFrontLeftDriveAbsoluteEncoderPort,
        swervemoduleconst.kFrontLeftDriveAbsoluteEncoderOffset,
        swervemoduleconst.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        swervemoduleconst.kFrontRightDriveMotorPort,
        swervemoduleconst.kFrontRightTurningMotorPort,
        swervemoduleconst.kFrontRightDriveEncoderReversed,
        swervemoduleconst.kFrontRightTurningEncoderReversed,
        swervemoduleconst.kFrontRightDriveAbsoluteEncoderPort,
        swervemoduleconst.kFrontRightDriveAbsoluteEncoderOffset,
        swervemoduleconst.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        swervemoduleconst.kBackLeftDriveMotorPort,
        swervemoduleconst.kBackLeftTurningMotorPort,
        swervemoduleconst.kBackLeftDriveEncoderReversed,
        swervemoduleconst.kBackLeftTurningEncoderReversed,
        swervemoduleconst.kBackLeftDriveAbsoluteEncoderPort,
        swervemoduleconst.kBackLeftDriveAbsoluteEncoderOffset,
        swervemoduleconst.kBackLeftDriveAbsoluteEncoderReversed);
        
    private final SwerveModule backRight = new SwerveModule(
        swervemoduleconst.kBackRightDriveMotorPort,
        swervemoduleconst.kBackRightTurningMotorPort,
        swervemoduleconst.kBackRightDriveEncoderReversed,
        swervemoduleconst.kBackRightTurningEncoderReversed,
        swervemoduleconst.kBackRightDriveAbsoluteEncoderPort,
        swervemoduleconst.kBackRightDriveAbsoluteEncoderOffset,
        swervemoduleconst.kBackRightDriveAbsoluteEncoderReversed);

    private PigeonIMU gyro = new PigeonIMU(Constants.PigeonIMUId);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    });

    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    };

    public void zeroHeading(){
        gyro.setYaw(0);
    }

  
    public double getHeading(){
        return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }

    public void resetOdometry(){
        odometry.resetPosition(getRotation2d(), new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
            }, getPose2d());
    }

    @Override
    public void periodic(){
        odometry.update(getRotation2d(), new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
        });
        SmartDashboard.putNumber("robot heading", getHeading());
    }
// X formation 
    public void Xswerve(){
        
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }


    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, speedconst.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    
}