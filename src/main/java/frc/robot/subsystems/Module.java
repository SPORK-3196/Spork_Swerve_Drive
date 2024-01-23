package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class Module extends SubsystemBase{

    public SwerveModuleState State;
    private Rotation2d offset;
    
    public PIDController AzumuthPID; 

    public CANSparkMax AzumuthNEO;
    public CANSparkMax DriveNEO;

    private SimpleMotorFeedforward OpenLoopFF = new SimpleMotorFeedforward(
        0,
        1,
        0.02);

    public RelativeEncoder DriveEncoder;

    public CANCoder absoluteEncoder;

    public Module(int TurnNeoID, int DriveID, int absoluteEncoderID, Rotation2d offset){
        
        this.offset = offset;
        State = new SwerveModuleState();
        
        AzumuthNEO = new CANSparkMax(TurnNeoID, MotorType.kBrushless);
        AzumuthNEO.setInverted(true);
        AzumuthNEO.setIdleMode(IdleMode.kBrake);

        DriveNEO = new CANSparkMax(DriveID, MotorType.kBrushless);
        DriveNEO.setIdleMode(IdleMode.kBrake);
        
        DriveEncoder = DriveNEO.getEncoder();
        DriveEncoder.setPosition(0);
        
        absoluteEncoder = new CANCoder(absoluteEncoderID);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        absoluteEncoder.setPositionToAbsolute();
    
        AzumuthPID = new PIDController(1, 0, 0);
        AzumuthPID.enableContinuousInput(0, 1);
    }

    public void setState(SwerveModuleState dState){

        dState = SwerveModuleState.optimize(dState, getCANangle());

        DriveNEO.set(OpenLoopFF.calculate(dState.speedMetersPerSecond));

        var out = AzumuthPID.calculate(getCANangle().getRotations(), dState.angle.getRotations());
        
        if(dState.speedMetersPerSecond > constants.MaxSpeed * 0.1){
            AzumuthNEO.set(out);
        }
        else{
            AzumuthNEO.set(0);
        }

        State = dState;
    }

    public Rotation2d getCANangle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition() - offset.getDegrees());
    }
    public Rotation2d getCANforshuffle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition() - offset.getDegrees());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), getCANangle());
    }

    public SwerveModuleState getstate(){
        return new SwerveModuleState(RPM_TO_M_per_S(DriveEncoder.getVelocity()), getCANangle());
    }

    private double RPM_TO_M_per_S(double RPM){
        var velocity = (((2 * Math.PI) * (constants.wheelDiameter /2 )) / 60) * RPM;
        return velocity;
    }

}
