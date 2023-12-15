package frc.robot.Utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.kSwerve;


public class MK4i {
    private SparkMaxPIDController DrivePID; 
    private SparkMaxPIDController TurnPID;
    private CANSparkMax DriveMotor;
    private CANSparkMax TurnMotor; 
    private RelativeEncoder DriveEncoder;
    private WPI_CANCoder Cancoder;
    private SwerveModuleState moduleState = new SwerveModuleState(); 
    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    private double chassisOffset;
    


    public MK4i(int DriveMotorID, int TurnMotorID, int CancoderID, boolean DriveReversed,
    double encoderOffset, boolean encoderReversed){
        chassisOffset = encoderOffset;
//Drive
        DriveMotor = new CANSparkMax(DriveMotorID, MotorType.kBrushless);
        DriveMotor.setIdleMode(IdleMode.kBrake);
        DriveMotor.setInverted(DriveReversed);
        DriveEncoder = DriveMotor.getEncoder();
        DrivePID = DriveMotor.getPIDController();

        DrivePID.setP(0.5);
        DrivePID.setD(0.02);

//Turn
        TurnMotor = new CANSparkMax(TurnMotorID, MotorType.kBrushless);
        TurnMotor.setInverted(true);
        TurnMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, CancoderID);

        TurnMotor.setSmartCurrentLimit(20);
        TurnMotor.setSecondaryCurrentLimit(35);

        TurnPID = TurnMotor.getPIDController();

        TurnPID.setP(0.5);
        TurnPID.setD(0);
        
        TurnPID.setOutputRange(-1, 1);
        TurnPID.setPositionPIDWrappingEnabled(true);
        TurnPID.setPositionPIDWrappingMaxInput(kSwerve.steeringEncoderPositionPIDMaxInput);
        TurnPID.setPositionPIDWrappingMinInput(kSwerve.steeringEncoderPositionPIDMinInput);  

//Cancoder
        Cancoder = new WPI_CANCoder(CancoderID);
        Cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        Cancoder.configMagnetOffset(encoderOffset); //in Deg
        Cancoder.configSensorDirection(encoderReversed);

        resetEncoders();
    }

    public void resetEncoders(){
        DriveEncoder.setPosition(0);
        TurnPID.setReference(0, ControlType.kPosition);
    }

    public double getTurnPos(){
        return Math.toRadians(Cancoder.getAbsolutePosition());
    }

    public double getDrivePos(){
        return DriveEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return DriveEncoder.getVelocity();
    }

    public SwerveModuleState getstate(){
        moduleState.speedMetersPerSecond = getDriveVelocity();
        moduleState.angle = new Rotation2d(getTurnPos());
        return moduleState;
    }

    public SwerveModulePosition getModPos(){
        modulePosition.distanceMeters = getDrivePos();
        modulePosition.angle = new Rotation2d(getTurnPos());
        return modulePosition;
    }

    public void setstates(SwerveModuleState desiredState){
        double driveVelocity = getDriveVelocity();
        double turnPos = getTurnPos();

        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnPos));


        if( Math.abs(desiredState.speedMetersPerSecond - driveVelocity)< 0.10 &&
        Math.abs(desiredState.angle.getRadians() - turnPos) < 0.10){
            stopModule();
            return;
        }

        DrivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);

        TurnPID.setReference(
            desiredState.angle.minus(new Rotation2d(chassisOffset)).getRadians(),
            ControlType.kPosition);
    }

    public void stopModule(){
        DriveMotor.setVoltage(0);
        TurnMotor.set(0);
    }
}