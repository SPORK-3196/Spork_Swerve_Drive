package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.kSwerve;

public class MK4I {

    private CANSparkMax DriveMotor;
    private CANSparkMax RotationMotor;

    private PIDController RotationController;
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
    private double angle;
    private double count = 0;
    private double lastangle;

    
    public MK4I(int driveID, int RotationID, int CANcoderID, Rotation2d Offset){
        configDrive(driveID);
        configRotation(RotationID);
        configCANCoder(CANcoderID);
        angleOffset = Offset;
        lastangle = getCanCoder().getDegrees();
        resetToAbsolute();
    }

    public void setState(SwerveModuleState desiredState, boolean isOpenLoop){
        optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        if(count == 8){
        System.out.println("CanCoder Value: " + getCanCoder().getDegrees()
         + ", motor encoder " + getAngle().getDegrees() +
          ", desiredState " + desiredState.angle.getDegrees() + 
         ", OptimizedState: " + optimizedState.angle.getDegrees() );

        System.out.println();

        count = 0;
        }
        count += 1;
        
        setAngle(optimizedState);
        setSpeed(desiredState, isOpenLoop);
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
        // angle = optimizedState.angle.getDegrees();
        // double canAng = getCanCoder().getDegrees();
        // double out = RotationController.calculate(canAng, angle);
        // //out = out > 1.0 ? 1.0 : (out < -1.0 ? -1.0 : out);
        // System.out.println("can, " + canAng + " angle, " + angle + " out " + out);
        

        // if(Math.abs(optimizedState.speedMetersPerSecond) <= (kSwerve.MaxSpeedMetersPerSecond) * 0.01){
        //     RotationMotor.set(0.0);
        // }
        // else{
        //     RotationMotor.set(out);
        // }
        
        

        if(Math.abs(optimizedState.speedMetersPerSecond) <= (kSwerve.MaxSpeedMetersPerSecond) * 0.01){
            angle = lastangle;
        }
        else{
            angle = optimizedState.angle.getDegrees();
        }
        double canAng = getCanCoder().getDegrees();
        double out = RotationController.calculate(canAng, angle);
        out = out > 0.3 ? 0.3 : (out < -0.3 ? -0.3 : out);
        System.out.println("can, " + canAng + " angle, " + angle + " out " + out);
        RotationMotor.set(out);

        lastangle = getCanCoder().getDegrees();
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
        DriveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        DriveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        DriveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

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
        
        RotationEncoder = RotationMotor.getEncoder();
        RotationMotor.restoreFactoryDefaults();
        RotationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        RotationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        RotationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);

        RotationMotor.setSmartCurrentLimit(20);
        RotationMotor.setInverted(true);
        RotationMotor.setIdleMode(IdleMode.kBrake);
        RotationEncoder.setPositionConversionFactor(1.0);
        RotationController = new PIDController(0.02, 0, 0.01);
        RotationController.enableContinuousInput(0, 360);
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
        return Rotation2d.fromRotations(RotationEncoder.getPosition() * 7/150);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(RotationEncoder.getPosition() * 7/150);

        //return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition()); 
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(DriveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), getAngle());
    }

}
