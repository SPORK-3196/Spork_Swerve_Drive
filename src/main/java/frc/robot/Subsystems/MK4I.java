package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.kSwerve;

public class MK4I {

    private CANSparkMax DriveMotor;
    private CANSparkMax RotationMotor;

    private SparkMaxPIDController RotationController;
    private SparkMaxPIDController DriveController;
    private SimpleMotorFeedforward OpenLoopFF = new SimpleMotorFeedforward(
        0.667,
        2.44,
        0.27);

    private CANCoder CANcoder;
    private RelativeEncoder RotationEncoder;
    private RelativeEncoder DriveEncoder;

    private Rotation2d angleOffset;
    private SwerveModuleState optimizedState;
    private Rotation2d lastangle;
    private double count = 0;


    
    public MK4I(int driveID, int RotationID, int CANcoderID, Rotation2d Offset){
        lastangle = new Rotation2d();
        angleOffset = Offset;
        configDrive(driveID);
        configRotation(RotationID);
        configCANCoder(CANcoderID);
        resetToAbsolute();
    }

    public void setState(SwerveModuleState desiredState, boolean isOpenLoop, double joy){
        optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        if(count == 8){
        System.out.println("CanCoder Value: " + getCanCoder().getDegrees() +
         ", DesiredState: " + optimizedState.angle.getDegrees() +
          ", LastAngle " + lastangle.getDegrees());
        count = 0;
        }
        count += 1;
        
        
        setAngle(optimizedState);
        //setSpeed(desiredState, isOpenLoop);
    }
    
    // works well 
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double output = desiredState.speedMetersPerSecond / kSwerve.MaxSpeedMetersPerSecond;
            DriveMotor.setVoltage(output);
        } else {
            DriveController.setReference(desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            OpenLoopFF.calculate(desiredState.speedMetersPerSecond));
        }
    }

//Dont know if it will work
    private void setAngle(SwerveModuleState optimizedState){
        Rotation2d angle;
        if(Math.abs(optimizedState.speedMetersPerSecond) <= (kSwerve.MaxSpeedMetersPerSecond) * 0.01){
            angle = lastangle;
        }
        else{
            angle = optimizedState.angle;
        }

        RotationController.setReference(angle.getDegrees(), ControlType.kPosition);
    
        lastangle = optimizedState.angle;
    }


    private void resetToAbsolute(){
        double absolutePosition =  getCanCoder().getDegrees() - angleOffset.getDegrees();
        RotationEncoder.setPosition(absolutePosition);
    }


    private void configDrive(int driveID){
        DriveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        DriveEncoder = DriveMotor.getEncoder();
        DriveController = DriveMotor.getPIDController();
        DriveMotor.restoreFactoryDefaults();
        DriveMotor.setSmartCurrentLimit(80);
        DriveMotor.setInverted(false);
        DriveMotor.setIdleMode(IdleMode.kBrake);
        DriveEncoder.setVelocityConversionFactor(kSwerve.DriveVelocityConversionFactor);
        DriveEncoder.setPositionConversionFactor(kSwerve.DrivePositionCoversionFactor);
        DriveController.setP(0.1);
        DriveController.setI(0);
        DriveController.setD(0);
        DriveController.setFF(0);
        DriveMotor.enableVoltageCompensation(12);
        DriveMotor.burnFlash();
        DriveEncoder.setPosition(0.0);
    }

    private void configRotation(int RotationID){
        RotationMotor = new CANSparkMax(RotationID, MotorType.kBrushless);
        RotationController = RotationMotor.getPIDController();
        RotationEncoder = RotationMotor.getEncoder();
        RotationMotor.restoreFactoryDefaults();
        RotationMotor.setSmartCurrentLimit(20);
        RotationMotor.setInverted(true);
        RotationMotor.setIdleMode(IdleMode.kBrake);
        RotationEncoder.setPositionConversionFactor(kSwerve.AngleConversionFactor);
        RotationController.setP(0.0);
        RotationController.setI(0.0000001);
        RotationController.setD(0);
        RotationController.setFF(0.0);
        RotationMotor.enableVoltageCompensation(12);
        RotationMotor.burnFlash();
    }

    private void configCANCoder(int CANcoderID){
        CANcoder = new CANCoder(CANcoderID);
        CANcoder.configFactoryDefault();
        CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        CANcoder.configSensorDirection(false);
        CANcoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        CANcoder.setPosition(0);
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(RotationEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition()); 
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(DriveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), getAngle());
    }

}
