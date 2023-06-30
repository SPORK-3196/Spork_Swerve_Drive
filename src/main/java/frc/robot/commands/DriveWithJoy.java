package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.RobotContainer;

public class DriveWithJoy extends CommandBase{
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xspdFunction, yspdFunction, TurningspdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    private PigeonIMU gyro = new PigeonIMU(Constants.PigeonIMUId);
    PIDController rotPidController;
    public double rotationalInput = Math.atan2(RobotContainer.RJSX_PRIM,RobotContainer.RJSY_PRIM );





    public DriveWithJoy(SwerveSubsystem swerveSubsystem,
        Supplier<Double> xspdFunction, Supplier<Double> yspdFunction, Supplier<Double> TurningspdFunction,
        Supplier<Boolean> fieldOrientedFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xspdFunction = xspdFunction;
        this.yspdFunction = yspdFunction;
        this.TurningspdFunction = TurningspdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turnLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        ((Gyro) gyro).reset();
        
    }

    @Override
    public void execute(){
        rotPidController = new PIDController(0, 0, 0);
        rotPidController.enableContinuousInput(-Math.PI, Math.PI);

        double xSpeed = xspdFunction.get();
        double ySpeed = yspdFunction.get();
        double turnSpeed = TurningspdFunction.get();

        xSpeed = Math.abs(xSpeed) > Constants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.kDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > Constants.kDeadband ? turnSpeed : 0.0;
        RobotContainer.RJSX_PRIM = RobotContainer.RJSX_PRIM > 0.8 ? RobotContainer.RJSX_PRIM : 0; 
        RobotContainer.RJSY_PRIM = RobotContainer.RJSY_PRIM > 0.8 ? RobotContainer.RJSX_PRIM : 0;

        
        xSpeed = xLimiter.calculate(xSpeed) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond;
        turnSpeed = turnLimiter.calculate(turnSpeed) * Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turnSpeed, swerveSubsystem.getRotation2d());
        }else{
            chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turnSpeed);
        }
    
        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);

        turnSetpoint();
        
        //rotPidController.setSetpoint(rotationalInput);        
    }

    public double getRotationalInput(){
        return rotationalInput;
    }

    public double getHeading(){
        return gyro.getYaw();
    }

    public void turnSetpoint(){
        rotPidController.calculate(getHeading(), rotationalInput);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    
    @Override
    public boolean isFinished(){
        return false;
    }

}
