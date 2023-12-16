package frc.robot.Utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.kSwerve;


public class MK4i {

    private final NetworkTable moduleTable;
    private final DoublePublisher goalVelPub;
    private final DoublePublisher goalHeadingPub;
    private final DoublePublisher measVelPub;
    private final DoublePublisher measHeadingPub;
    private final DoubleArrayPublisher voltagesPub;
    
    private SparkMaxPIDController DrivePID; 
    private SparkMaxPIDController TurnPID;
    private CANSparkMax DriveMotor;
    private CANSparkMax TurnMotor; 
    private RelativeEncoder DriveEncoder;
    private WPI_CANCoder Cancoder;
    private SwerveModuleState moduleState = new SwerveModuleState(); 
    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    private SimpleMotorFeedforward DriveFF;

    private double simDrivePosition = 0;
    

    public MK4i(int DriveMotorID, int TurnMotorID, int CancoderID, boolean DriveReversed,
    double encoderOffset, boolean encoderReversed, NetworkTable table){
//Drive
        DriveFF = new SimpleMotorFeedforward(
            0.046, 
            2.67, 
            0.113);
        DriveMotor = new CANSparkMax(DriveMotorID, MotorType.kBrushless);
        DriveMotor.setIdleMode(IdleMode.kBrake);
        DriveMotor.setInverted(DriveReversed);
        DriveEncoder = DriveMotor.getEncoder();
        DrivePID = DriveMotor.getPIDController();

        DrivePID.setP(0.5);
        DrivePID.setD(0);

//Turn
        TurnMotor = new CANSparkMax(TurnMotorID, MotorType.kBrushless);
        TurnMotor.setInverted(true);
        TurnMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, CancoderID);

        TurnMotor.setSmartCurrentLimit(20);
        TurnMotor.setSecondaryCurrentLimit(35);

        TurnPID = TurnMotor.getPIDController();

        TurnPID.setP(0.5);
        TurnPID.setD(0);
        
        TurnPID.setOutputRange(-Math.PI, Math.PI);
        TurnPID.setPositionPIDWrappingEnabled(true);
        TurnPID.setPositionPIDWrappingMaxInput(kSwerve.steeringEncoderPositionPIDMaxInput);
        TurnPID.setPositionPIDWrappingMinInput(kSwerve.steeringEncoderPositionPIDMinInput);  

//Cancoder
        Cancoder = new WPI_CANCoder(CancoderID);
        Cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        Cancoder.configMagnetOffset(encoderOffset); //in Deg
        Cancoder.configSensorDirection(encoderReversed);

        if (!RobotBase.isReal()) moduleState.angle = new Rotation2d(Cancoder.getAbsolutePosition());

        moduleTable = table;
        goalVelPub = moduleTable.getDoubleTopic("Goal Velocity").publish();
        goalHeadingPub = moduleTable.getDoubleTopic("Goal Heading").publish();
        measVelPub = moduleTable.getDoubleTopic("Measured Velocity").publish();
        measHeadingPub = moduleTable.getDoubleTopic("Measured Heading").publish();
        voltagesPub = moduleTable.getDoubleArrayTopic("Voltages").publish();

    }

    public void resetEncoder(){
        DriveEncoder.setPosition(0);
        if (RobotBase.isSimulation()) simDrivePosition = 0;
    }

    public Rotation2d getTurnPos(){
        if (RobotBase.isSimulation()) return moduleState.angle;
        return new Rotation2d(Math.toRadians(Cancoder.getAbsolutePosition()));
    }

    public double getDrivePos(){
        return DriveEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return DriveEncoder.getVelocity();
    }

    public SwerveModuleState getstate(){
        if (RobotBase.isSimulation()) return moduleState;
        moduleState.speedMetersPerSecond = getDriveVelocity();
        moduleState.angle = getTurnPos();
        return moduleState;
    }

    public SwerveModulePosition getModPos(){
        if (RobotBase.isSimulation()){
      return new SwerveModulePosition(simDrivePosition, getTurnPos());
        }
        modulePosition.distanceMeters = getDrivePos();
        modulePosition.angle = getTurnPos();
        return modulePosition;
    }

    public void setstates(SwerveModuleState desiredState){
        double driveVelocity = getDriveVelocity();
    

        desiredState = SwerveModuleState.optimize(desiredState, getTurnPos());


        if( Math.abs(desiredState.speedMetersPerSecond - driveVelocity)< 0.10 &&
        Math.abs(desiredState.angle.getRadians() - getTurnPos().getRadians()) < 0.10){
            stopModule();
            return;
        }

        DriveMotor.setVoltage(DriveFF.calculate(driveVelocity, desiredState.speedMetersPerSecond));

        TurnPID.setReference(desiredState.angle.getRadians(), ControlType.kPosition);

        moduleState = desiredState;

        if (RobotBase.isSimulation()) simDrivePosition += desiredState.speedMetersPerSecond * 0.02;
    }

    public void stopModule(){
        DriveMotor.setVoltage(0);
        TurnMotor.set(0);
    }

    public void updateNT() {
        goalVelPub.set(moduleState.speedMetersPerSecond);
        goalHeadingPub.set(moduleState.angle.getRadians());
        measVelPub.set(DriveEncoder.getVelocity());
        measHeadingPub.set(getTurnPos().getRadians());
        voltagesPub.set(
            new double[] {
              DriveMotor.getAppliedOutput() * DriveMotor.getBusVoltage(),
              TurnMotor.getAppliedOutput() * TurnMotor.getBusVoltage()
            });
      }

    public SwerveModuleState getModuleState() {
        return moduleState;
    }
}