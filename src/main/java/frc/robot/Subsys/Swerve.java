

package frc.robot.Subsys;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class Swerve extends SubsystemBase{

    public AHRS gyro = new AHRS(Port.kMXP);

    final SwerveModule frontLeft; 
    final SwerveModule backLeft; 
    final SwerveModule frontRight; 
    final SwerveModule backRight; 

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

    private static final int MAX_VOLTAGE = 10;


    public Swerve(){
        frontLeft = Mk4iSwerveModuleHelper.createNeo(Mk4iSwerveModuleHelper.GearRatio.L1,
        kSwerve.CANID.frontLeftDrive,
        kSwerve.CANID.frontLeftSteer,
          kSwerve.CANID.kFrontLeftDriveAbsoluteEncoderPort,
          kSwerve.Offsets.frontLeft);
        
        backLeft = Mk4iSwerveModuleHelper.createNeo(Mk4iSwerveModuleHelper.GearRatio.L1,
        kSwerve.CANID.backLeftDrive,
        kSwerve.CANID.backLeftSteer,
        kSwerve.CANID.kBackLeftDriveAbsoluteEncoderPort,
        kSwerve.Offsets.backLeft);
        
        frontRight = Mk4iSwerveModuleHelper.createNeo(Mk4iSwerveModuleHelper.GearRatio.L1,
        kSwerve.CANID.frontRightDrive,
         kSwerve.CANID.frontRightSteer,
            kSwerve.CANID.kFrontRightDriveAbsoluteEncoderPort,
            kSwerve.Offsets.frontRight);

        backRight = Mk4iSwerveModuleHelper.createNeo(Mk4iSwerveModuleHelper.GearRatio.L1,
        kSwerve.CANID.backRightDrive,
         kSwerve.CANID.backRightSteer,
            kSwerve.CANID.kBackRightDriveAbsoluteEncoderPort,
            kSwerve.Offsets.backRight);    


    }

    public void drive(ChassisSpeeds chassisSpeeds){
        m_chassisSpeeds = chassisSpeeds;
    }

    public void setModZero(){
      frontLeft.set(0, 0);
      frontRight.set(0, 0);
      backLeft.set(0, 0);
      backRight.set(0, 0);

    }


    
    

    @Override
    public void periodic(){
      SwerveModuleState[] states = kSwerve.kinematics.toSwerveModuleStates(m_chassisSpeeds);
      //SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
  
      
      frontLeft.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      backLeft.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      frontRight.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      backRight.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
/*
      System.out.println("back left" + backLeft.getSteerAngle());
      System.out.println("front left" + frontLeft.getSteerAngle() );
      System.out.println("Back right" + backRight.getSteerAngle());
      System.out.println("front right" +  frontRight.getSteerAngle());
      */
    }


    public Rotation2d getGyroRotation(){
      return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }

    public void zeroGyro(){
      gyro.zeroYaw();
    }
  }
