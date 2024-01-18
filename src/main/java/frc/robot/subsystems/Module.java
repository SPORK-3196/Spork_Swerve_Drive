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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase{

    public SwerveModuleState State; 
    
    public PIDController AzumuthPID; 

    public CANSparkMax AzumuthNEO;

    public AbsoluteEncoder azumuthEncoder;

    public CANCoder absoluteEncoder;

    public Module(int TurnNeoID, int absoluteEncoderID, double offset){
        State = new SwerveModuleState();
        AzumuthNEO = new CANSparkMax(TurnNeoID, MotorType.kBrushless);
        AzumuthNEO.setInverted(true);
        AzumuthNEO.setIdleMode(IdleMode.kBrake);
        azumuthEncoder = AzumuthNEO.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder = new CANCoder(absoluteEncoderID);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        AzumuthPID = new PIDController(1, 0, 0);
        //azumuthEncoder.setPositionConversionFactor(150/7);
        AzumuthPID.enableContinuousInput(0, 1);
    }

    public void setState(SwerveModuleState dState){

        SwerveModuleState.optimize(dState, getCANangle());

        var out = AzumuthPID.calculate(getCANangle().getRotations(), dState.angle.getRotations());
        AzumuthNEO.set(out);

        SmartDashboard.putNumber("CAN angle", getCANangle().getDegrees());
        SmartDashboard.putNumber("motor encoder", getMotorAng().getDegrees());
        SmartDashboard.putNumber("setpoint in Deg", dState.angle.getDegrees());
        SmartDashboard.putNumber("Azumuth PID out", out);

        State = dState;
    }

    public Rotation2d getCANangle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition());
    }

    public Rotation2d getMotorAng(){
        return Rotation2d.fromRotations(azumuthEncoder.getPosition());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(0, getCANangle());
    }

    public SwerveModuleState getstate(){
        return new SwerveModuleState(0, getCANangle());
    }

}
