package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
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

    private CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();

    private Rotation2d angleOffset;
    private SwerveModuleState lastState = new SwerveModuleState();


    
    public MK4I(int driveID, int RotationID, int CANcoderID, Rotation2d Offset){
        angleOffset = Offset;
        configDrive(driveID);
        configRotation(RotationID);
        configCANCoder(CANcoderID);
        resetToAbsolute();
    }

    public void setState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
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
        DriveController.setP(0.25);
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
        RotationController.setP(0.01);
        RotationController.setI(0);
        RotationController.setD(0);
        RotationController.setFF(0.0);
        RotationMotor.enableVoltageCompensation(12);
        RotationMotor.burnFlash();
    }

    private void configCANCoder(int CANcoderID){
        CANcoder = new CANCoder(CANcoderID);
        CANcoder.configFactoryDefault();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = false;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
        CANcoder.configAllSettings(canCoderConfiguration);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double output = desiredState.speedMetersPerSecond / kSwerve.MaxSpeedMetersPerSecond;
            DriveMotor.set(output);
        } else { 
            DriveController.setReference(desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            OpenLoopFF.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d rotation = (Math.abs(desiredState.speedMetersPerSecond)
         <= (kSwerve.MaxSpeedMetersPerSecond * 0.01)) 
         ? lastState.angle 
         : desiredState.angle;

        RotationController.setReference(rotation.getDegrees(), ControlType.kPosition);
        lastState.angle = rotation;
    }

    private Rotation2d getRotation(){
        return Rotation2d.fromDegrees(RotationEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(DriveEncoder.getVelocity(),getRotation());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), getRotation());
    }


}
