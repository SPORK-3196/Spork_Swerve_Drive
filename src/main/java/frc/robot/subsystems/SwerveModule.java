package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.speedconst;

public class SwerveModule {
    
    private final CANSparkMax DriveMotor;
    private final CANSparkMax TurnMotor;
    private final CANCoder canCoder;

    private final RelativeEncoder DriveEncoder;
    private final RelativeEncoder turnEncoder;
    
    private final PIDController turnPidController;
    
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    CANCoderConfiguration config;


    public SwerveModule(int DriveMotorId, int TurnMotorId, boolean DriveMotorReversed, boolean TurnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        canCoder = new CANCoder(absoluteEncoderId);
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "Rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.sensorDirection = !absoluteEncoderReversed;
        canCoder.configAllSettings(config);
        
        DriveMotor = new CANSparkMax(DriveMotorId, MotorType.kBrushless);
        TurnMotor = new CANSparkMax(TurnMotorId, MotorType.kBrushless);
        DriveMotor.setIdleMode(IdleMode.kBrake);
        TurnMotor.setIdleMode(IdleMode.kBrake);

        DriveEncoder = DriveMotor.getEncoder();
        turnEncoder = TurnMotor.getEncoder();

        DriveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
        DriveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(Constants.kTurningEncoderRPM2RadPerSec);

        turnPidController = new PIDController(Constants.kPTurning, Constants.kITurning, Constants.kDTurning);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }

    public double getDrivePos(){
        return DriveEncoder.getPosition();
    }

    public double getTurnPos(){
        return turnEncoder.getPosition();
    }

    public double getDriveVolocity(){
        return DriveEncoder.getVelocity();
    }

    public double getTurnVolocity(){
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        return canCoder.getAbsolutePosition();
    }
    public Rotation2d getAngle(){
        return new Rotation2d(canCoder.getPosition() + absoluteEncoderOffsetRad);
    }

    public void resetEncoders(){
        DriveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), getAngle());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVolocity(), new Rotation2d(getTurnPos()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)< 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        DriveMotor.set(state.speedMetersPerSecond / speedconst.kPhysicalMaxSpeedMetersPerSecond);
        TurnMotor.set(turnPidController.calculate(getTurnPos(),state.angle.getRadians()));
        
    
    }

    public void stop() {
        DriveMotor.set(0);
        TurnMotor.set(0);
    }
}