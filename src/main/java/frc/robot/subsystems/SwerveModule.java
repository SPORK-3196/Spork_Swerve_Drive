package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    
    private final CANSparkMax DriveMotor;
    private final CANSparkMax TurnMotor;

    private final RelativeEncoder DriveEncoder;
    private final RelativeEncoder turnEncoder;
    
    private final PIDController turnPidController;

    private final CanCoderAbsoluteConfiguration absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int DriveMotorId, int TurnMotorId, boolean DriveMotorReversed, boolean TurnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CanCoderAbsoluteConfiguration(absoluteEncoderId, absoluteEncoderOffset);

        DriveMotor = new CANSparkMax(DriveMotorId, MotorType.kBrushless);
        TurnMotor = new CANSparkMax(TurnMotorId, MotorType.kBrushless);
        
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
        double angle = absoluteEncoder.getOffset();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        DriveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition();
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
        DriveMotor.set(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
        TurnMotor.set(turnPidController.calculate(getTurnPos(),state.angle.getRadians()));
        
    
    }

    public void stop() {
        DriveMotor.set(0);
        TurnMotor.set(0);
    }
}