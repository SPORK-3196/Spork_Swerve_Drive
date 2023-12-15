package frc.robot.Subsys;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.MK4i;
import frc.robot.Utils.limiter;

public class Swerve extends SubsystemBase{
    private final limiter limiter;
    private final SwerveModulePosition[] positions;
    private final SwerveDrivePoseEstimator estimator;
    private final AHRS gyro;
    private ChassisSpeeds speeds = new ChassisSpeeds(0,0,0);

    public final MK4i frontLeft = new MK4i(
        Constants.kSwerve.frontLeftDrive, 
        Constants.kSwerve.frontLeftSteer, 
        Constants.kSwerve.kFrontLeftDriveAbsoluteEncoderPort, 
        true, 
        Constants.kSwerve.Offsets.frontLeft, 
        false);
      public final MK4i frontRight = new MK4i(
        Constants.kSwerve.frontRightDrive,
        Constants.kSwerve.frontRightSteer,
        Constants.kSwerve.kFrontRightDriveAbsoluteEncoderPort, 
        false, 
        Constants.kSwerve.Offsets.frontRight, 
        false);
      public final MK4i backLeft = new MK4i(
        Constants.kSwerve.backLeftDrive,
        Constants.kSwerve.backLeftSteer,
        Constants.kSwerve.kBackLeftDriveAbsoluteEncoderPort, 
        true, 
        Constants.kSwerve.Offsets.backLeft, 
        false);
      public final MK4i backRight = new MK4i(
        Constants.kSwerve.backRightDrive,
        Constants.kSwerve.backRightSteer,
        Constants.kSwerve.kBackRightDriveAbsoluteEncoderPort, 
        false, 
        Constants.kSwerve.Offsets.backRight, 
        false);

    public Swerve()
    {
        limiter = new limiter(0.02 * 9.81, 2);
        gyro = new AHRS(Port.kMXP);
        estimator = new SwerveDrivePoseEstimator(
        Constants.kSwerve.DRIVE_KINEMATICS,
        getGyroHeading(),
        getPositions(),
        new Pose2d(4, 4, new Rotation2d()));
        positions = getPositions();
        updateSwerveModulePositions();
        zeroGyro();

    }
    public double getFrontLeftAngle(){
        return frontLeft.getTurnPos();
    }

    public double getBackLeftAngle(){
        return backLeft.getTurnPos();
    }

    public double getBackRightAngle(){
        return backRight.getTurnPos();
    }

    public double getFrontRightAngle(){
        return frontRight.getTurnPos();
    }


    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
            frontLeft.getModPos(),
            frontRight.getModPos(),
            backLeft.getModPos(),
            backRight.getModPos()
        };
    }

    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getGyroHeading(){
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Pose2d getPose(){
        return estimator.getEstimatedPosition();
    }

    public void resetEstimator(Pose2d pose){

        estimator.resetPosition(getGyroHeading(), positions, pose);
    }

    //use with vision ONLY
    public void resetEstimatorWithVision(Pose2d visPose, double timestamp){
        estimator.addVisionMeasurement(visPose, timestamp);
    }

    //P.S. quite important
    public void Drive(SwerveModuleState... desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,Constants.kSwerve.MaxSpeedMetersPerSecond);
        frontLeft.setstates(desiredStates[0]);
        frontRight.setstates(desiredStates[1]);
        backLeft.setstates(desiredStates[2]);
        backRight.setstates(desiredStates[3]);
    }

    public ChassisSpeeds getChassisSpeeds(){
        speeds = Constants.kSwerve.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getstate(),
            frontRight.getstate(),
            backLeft.getstate(),
            backRight.getstate()
        );
        speeds = limiter.calculate(speeds);
        return speeds;
    }

    public void stopAll(){
        Drive(
            Constants.kSwerve.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds())
        );
    }

    public double deadband(double value){
        if(Math.abs(value)<= 0.1){
            return 0;
        }
        return value;
    }

    public void updateSwerveModulePositions() {
        positions[0] = frontLeft.getModPos();
        positions[1] = frontRight.getModPos();
        positions[2] = backLeft.getModPos();
        positions[3] = backRight.getModPos();
    }
}