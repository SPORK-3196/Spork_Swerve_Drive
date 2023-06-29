package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.kFrontLeftDriveMotorPort,
        Constants.kFrontLeftTurningMotorPort,
        Constants.kFrontLeftDriveEncoderReversed,
        Constants.kFrontLeftTurningEncoderReversed,
        Constants.kFrontLeftDriveAbsoluteEncoderPort,
        Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        Constants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        Constants.kFrontRightDriveMotorPort,
        Constants.kFrontRightTurningMotorPort,
        Constants.kFrontRightDriveEncoderReversed,
        Constants.kFrontRightTurningEncoderReversed,
        Constants.kFrontRightDriveAbsoluteEncoderPort,
        Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        Constants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        Constants.kBackLeftDriveMotorPort,
        Constants.kBackLeftTurningMotorPort,
        Constants.kBackLeftDriveEncoderReversed,
        Constants.kBackLeftTurningEncoderReversed,
        Constants.kBackLeftDriveAbsoluteEncoderPort,
        Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        Constants.kBackLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule backRight = new SwerveModule(
        Constants.kBackRightDriveMotorPort,
        Constants.kBackRightTurningMotorPort,
        Constants.kBackRightDriveEncoderReversed,
        Constants.kBackRightTurningEncoderReversed,
        Constants.kBackRightDriveAbsoluteEncoderPort,
        Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
        Constants.kBackRightDriveAbsoluteEncoderReversed);

    private PigeonIMU gyro = new PigeonIMU(Constants.PigeonIMUId);

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

    @Override
    public void periodic(){
        SmartDashboard.putNumber("robot heading", getHeading());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }


    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    

}
