package frc.robot.Utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MK4i {

    private ProfiledPIDController DrivePID = new ProfiledPIDController(
    0.1,
    0,
    0.1,
    new TrapezoidProfile.Constraints(1, 0.5));
    private ProfiledPIDController TurnPID = new ProfiledPIDController(
    0.1, 
    0,
    0.1,
    new TrapezoidProfile.Constraints(3*Math.PI, 6*Math.PI));
    private CANSparkMax DriveMotor;
    private CANSparkMax TurnMotor; 
    private RelativeEncoder DriveEncoder;
    private WPI_CANCoder Cancoder;
    //TODO ff values 
    private SimpleMotorFeedforward DriveFF = new SimpleMotorFeedforward(
    0.1, 
    0, 
    0);
    private SimpleMotorFeedforward TurnFF = new SimpleMotorFeedforward(
    0.1, 
    0, 
    0);
    private SwerveModuleState moduleState = new SwerveModuleState(); 
    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    


    public MK4i(int DriveMotorID, int TurnMotorID, int CancoderID, boolean DriveReversed,
    double encoderOffset, boolean encoderReversed){
//Drive
        DriveMotor = new CANSparkMax(DriveMotorID, MotorType.kBrushless);
        DriveMotor.setIdleMode(IdleMode.kBrake);
        DriveMotor.setInverted(DriveReversed);
        DriveEncoder = DriveMotor.getEncoder();
//Turn
        TurnMotor = new CANSparkMax(TurnMotorID, MotorType.kBrushless);
        TurnMotor.setInverted(true);
        TurnPID.enableContinuousInput(-Math.PI, Math.PI);
//Cancoder
        Cancoder = new WPI_CANCoder(CancoderID);
        Cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        Cancoder.configMagnetOffset(encoderOffset); //in Deg
        Cancoder.configSensorDirection(encoderReversed);

        resetEncoders();
    }

    public void resetEncoders(){
        DriveEncoder.setPosition(0);
        TurnPID.calculate(Cancoder.getAbsolutePosition(), 0);
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
        if( Math.abs(desiredState.speedMetersPerSecond)< 0.10 && Math.abs(desiredState.angle.getRadians() - turnPos) < 0.05){
            stopModule();
            return;
        }

        final double Drive = DrivePID.calculate(driveVelocity,desiredState.speedMetersPerSecond)
         + DriveFF.calculate(desiredState.speedMetersPerSecond);
        final double Turn = TurnPID.calculate(turnPos, desiredState.angle.getRadians())
         + TurnFF.calculate(TurnPID.getSetpoint().velocity);

        DriveMotor.setVoltage(Drive);
        TurnMotor.setVoltage(Turn);
    }

    public void stopModule(){
        DriveMotor.setVoltage(0);
        TurnMotor.set(0);
    }
}