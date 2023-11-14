package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithJoy extends CommandBase{
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xspdFunction, yspdFunction, TurningspdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public DriveWithJoy(SwerveSubsystem swerveSubsystem,
        Supplier<Double> xspdFunction, Supplier<Double> yspdFunction, Supplier<Double> TurningspdFunction,
        Supplier<Boolean> fieldOrientedFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xspdFunction = //xspdFunction;
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

    }

    @Override
    public void execute(){
        double xSpeed = xspdFunction.get();
        double ySpeed = yspdFunction.get();
        double turnSpeed = TurningspdFunction.get();

        xSpeed = Math.abs(xSpeed) > Constants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.kDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > Constants.kDeadband ? turnSpeed : 0.0;

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