package frc.robot.Subsys;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;
import frc.robot.Utils.MK4i;

public class Swerve extends SubsystemBase{

    NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    NetworkTable ntTable = ntInst.getTable("system/drivetrain");
    NetworkTable modulesTable = ntTable.getSubTable("modules");
    NetworkTable poseTable = ntTable.getSubTable("pose");
    
    private final SwerveModulePosition[] positions;
    private final SwerveDrivePoseEstimator estimator;
    private final AHRS gyro;
    private ChassisSpeeds speeds = new ChassisSpeeds(0,0,0);

// Logging
private final Field2d field2d = new Field2d();
private final FieldObject2d autonRobot = field2d.getObject("Autonomous Pose");
private final FieldObject2d autonPath = field2d.getObject("Autonomous Path");
private final DoublePublisher rawGyroPub = ntTable.getDoubleTopic("Raw Gyro").publish();
private final DoublePublisher offsetGyroPub = ntTable.getDoubleTopic("Offset Gyro").publish();
private final DoubleArrayPublisher chassisVelPub =
      ntTable.getDoubleArrayTopic("Commanded Chassis Velocity").publish();
private final DoubleArrayPublisher measuredVelPub =
      ntTable.getDoubleArrayTopic("Actual Chassis Velocity").publish();
private final DoublePublisher autonXError = poseTable.getDoubleTopic("Path x Error").publish();
private final DoublePublisher autonYError = poseTable.getDoubleTopic("Path y Error").publish();
private final DoublePublisher autonZError = poseTable.getDoubleTopic("Path z Error").publish();
private final DoubleArrayPublisher swerveStatesPub =
      ntTable.getDoubleArrayTopic("Swerve Module States").publish();
private Rotation2d gyroOffset;

    public final MK4i frontLeft = new MK4i(
        Constants.kSwerve.frontLeftDrive, 
        Constants.kSwerve.frontLeftSteer, 
        Constants.kSwerve.kFrontLeftDriveAbsoluteEncoderPort, 
        true, 
        Constants.kSwerve.Offsets.frontLeft, 
        false,
        modulesTable.getSubTable("FrontLeft"));
      public final MK4i frontRight = new MK4i(
        Constants.kSwerve.frontRightDrive,
        Constants.kSwerve.frontRightSteer,
        Constants.kSwerve.kFrontRightDriveAbsoluteEncoderPort, 
        false, 
        Constants.kSwerve.Offsets.frontRight, 
        false,
        modulesTable.getSubTable("FrontRight"));
      public final MK4i backLeft = new MK4i(
        Constants.kSwerve.backLeftDrive,
        Constants.kSwerve.backLeftSteer,
        Constants.kSwerve.kBackLeftDriveAbsoluteEncoderPort, 
        true, 
        Constants.kSwerve.Offsets.backLeft, 
        false,
        modulesTable.getSubTable("BackLeft"));
      public final MK4i backRight = new MK4i(
        Constants.kSwerve.backRightDrive,
        Constants.kSwerve.backRightSteer,
        Constants.kSwerve.kBackRightDriveAbsoluteEncoderPort, 
        false, 
        Constants.kSwerve.Offsets.backRight, 
        false,
        modulesTable.getSubTable("BackRight"));


        final SimDeviceSim gyroSim = new SimDeviceSim("gyro", 0 );
        final SimDouble gyroYawSim = gyroSim.getDouble("yaw");


    public Swerve()
    {
        gyro = new AHRS(Port.kMXP);
        estimator = new SwerveDrivePoseEstimator(
        Constants.kSwerve.DRIVE_KINEMATICS,
        getGyroHeading(),
        getPositions(),
        new Pose2d(0, 0, new Rotation2d()));
        positions = getPositions();
        updateSwerveModulePositions();
    }


    public void periodic() {
        if (RobotBase.isSimulation())
        gyroYawSim.set(
            gyroYawSim.get() + speeds.omegaRadiansPerSecond * -360/ (2 * Math.PI) * 0.02
        );
        estimator.update(getGyroHeading(), getPositions());
        estimator.addVisionMeasurement(getPose(), getGyroYawRate());
        
      }

    public double getFrontLeftAngle(){
        return frontLeft.getTurnPos().getRadians();
    }

    public double getBackLeftAngle(){
        return backLeft.getTurnPos().getRadians();
    }

    public double getBackRightAngle(){
        return backRight.getTurnPos().getRadians();
    }

    public double getFrontRightAngle(){
        return frontRight.getTurnPos().getRadians();
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
            frontLeft.getModPos(),
            frontRight.getModPos(),
            backLeft.getModPos(),
            backRight.getModPos()
        };
    }

    public void zeroGyro() {
        gyroOffset = getGyroRaw();
      }    

    private Rotation2d getGyroRaw(){
        return gyro.getRotation2d(); 
    }

    private Rotation2d getGyro() {
        return getGyroRaw().minus(gyroOffset);
      }
    
      // Get gyro yaw rate (radians/s CCW +)
      private double getGyroYawRate() {
        return Units.degreesToRadians(-gyro.getRate());
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
        return speeds; 
    }

    public void stopAll(){
        Drive(
            Constants.kSwerve.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds())
        );
    }

    public double deadband(double value){
        if(Math.abs(value)<= 0.15){
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

    private SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
          frontLeft.getModuleState(), backLeft.getModuleState(), backRight.getModuleState(), frontRight.getModuleState()
        };
      }

    private void log() {
        frontLeft.updateNT();
        backLeft.updateNT();
        backRight.updateNT();
        frontRight.updateNT();
    
        rawGyroPub.set(getGyroRaw().getRadians());
        offsetGyroPub.set(getGyro().getRadians());
        // Send the chassis velocity as a double array (vel_x, vel_y, omega_z)
        chassisVelPub.set(
            new double[] {
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond
            });
        var measuredVel =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                kSwerve.DRIVE_KINEMATICS.toChassisSpeeds(getStates()), getGyro().unaryMinus());
        measuredVelPub.set(
            new double[] {
              measuredVel.vxMetersPerSecond, measuredVel.vyMetersPerSecond, getGyroYawRate()
            });
    
        swerveStatesPub.set(
            new double[] {
              frontLeft.getModuleState().angle.getRadians(),
                  frontLeft.getModuleState().speedMetersPerSecond,
              backLeft.getModuleState().angle.getRadians(),
                  backLeft.getModuleState().speedMetersPerSecond,
              backRight.getModuleState().angle.getRadians(),
                  backRight.getModuleState().speedMetersPerSecond,
              frontRight.getModuleState().angle.getRadians(),
                  frontRight.getModuleState().speedMetersPerSecond
            });
    
        field2d.setRobotPose(getPose());
        autonRobot.setPose(new Pose2d(8, 4, Rotation2d.fromDegrees(90)));
      }
}