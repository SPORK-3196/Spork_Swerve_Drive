package frc.robot.Subsystems;

import org.opencv.core.Mat;

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
import frc.robot.OI;
import frc.robot.Constants.kSwerve;

public class MK4I {
    private SwerveModuleState optimizedState = new SwerveModuleState();
    private SwerveModuleState Target = new SwerveModuleState();
    private double angleOffset;

    private SimpleMotorFeedforward OpenLoopFF = new SimpleMotorFeedforward(
        0.667,
        2.44,
        0.27);

    private CANSparkMax DriveMotor;
    private CANSparkMax RotationMotor;

    private SparkMaxPIDController RotationController;
    private SparkMaxPIDController DriveController;

    private CANCoder CANcoder;
    private RelativeEncoder RotationEncoder;
    private RelativeEncoder DriveEncoder;

    
    public MK4I(int driveID, int RotationID, int CANcoderID, Double Offset){
        configDrive(driveID);
        configRotation(RotationID);
        configCANCoder(CANcoderID);
        angleOffset = Offset;
        Target.angle = getAngle();
    }

    public void setState(SwerveModuleState desiredState, boolean isOpenLoop){
        optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        OI.swervevalues.CanCoder = getCanCoder().getRotations();
        OI.swervevalues.RotMotorEncoder = getAngle().getRotations();
        OI.swervevalues.desiredState = desiredState.angle.getRotations();

        setAngle(desiredState);
        //setSpeed(desiredState, isOpenLoop);

        Target = optimizedState;
    }

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

//works now okayly good 
    private void setAngle(SwerveModuleState optimizedState){
        RotationController.setReference(optimizedState.angle.minus(new Rotation2d(angleOffset)).getRadians(),
        ControlType.kPosition);
    }

    private Rotation2d GetSteerAng(){
       return new Rotation2d(CANcoder.getAbsolutePosition() + angleOffset);
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
        DriveController.setP(0.1); //0.1
        DriveController.setI(0); //0.15
        DriveController.setD(0); //0
        DriveController.setFF(0);
        DriveMotor.enableVoltageCompensation(12);
        DriveMotor.burnFlash();
        DriveEncoder.setPosition(0.0);
    }

    private void configRotation(int RotationID){
        RotationMotor = new CANSparkMax(RotationID, MotorType.kBrushless);
        RotationEncoder = RotationMotor.getEncoder();
        RotationMotor.restoreFactoryDefaults();
        RotationMotor.setSmartCurrentLimit(20);
        RotationMotor.setInverted(true);
        RotationMotor.setIdleMode(IdleMode.kBrake);
        RotationController = RotationMotor.getPIDController();
        RotationController.setP(1); //1
        RotationController.setI(0); //0
        RotationController.setD(0.1); //0.2
        RotationController.setPositionPIDWrappingEnabled(true);
        RotationController.setPositionPIDWrappingMaxInput(Math.PI);
        RotationController.setPositionPIDWrappingMinInput(-Math.PI);
        RotationMotor.enableVoltageCompensation(12);
        RotationMotor.burnFlash();
    }

    private void configCANCoder(int CANcoderID){
        CANcoder = new CANCoder(CANcoderID);
        CANcoder.configFactoryDefault();
        CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        CANcoder.configSensorDirection(false);
        CANcoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(RotationEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition()); 
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(DriveEncoder.getVelocity(), GetSteerAng());
    }

    public SwerveModuleState getTarget(){
        return Target;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), GetSteerAng());
    }

}