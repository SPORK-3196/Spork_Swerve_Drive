package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.kSwerve;
import frc.robot.Subsystems.holoDrive;

public class Drive extends CommandBase{
    private holoDrive m_HoloDrive;
    private DoubleSupplier Translation, strafe, rotation;
    private BooleanSupplier robotCentric;
    
    private SlewRateLimiter xLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(0.5);
    
    public Drive(
        holoDrive m_HoloDrive,
        DoubleSupplier Translation,
        DoubleSupplier rotation,
        DoubleSupplier strafe,
        BooleanSupplier robotCentric)
    {
        this.m_HoloDrive = m_HoloDrive;
        addRequirements(m_HoloDrive);


        this.Translation = Translation;
        this.rotation = rotation;
        this.strafe = strafe;
        this.robotCentric = robotCentric;
    }

    @Override
    public void execute(){

        double xSpeed = Translation.getAsDouble();
        double ySpeed = strafe.getAsDouble();
        double turnSpeed = rotation.getAsDouble();

        xSpeed = Math.abs(xSpeed) > Constants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.kDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > Constants.kDeadband ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * kSwerve.MaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * kSwerve.MaxSpeedMetersPerSecond;
        turnSpeed = turnLimiter.calculate(turnSpeed) * kSwerve.Maxrotation;

        OI.swervevalues.X_Joy = xSpeed;
        OI.swervevalues.Y_Joy = ySpeed;
        OI.swervevalues.Rot_Joy = turnSpeed;

        m_HoloDrive.drive(new Translation2d(xSpeed, ySpeed).times(kSwerve.MaxSpeedMetersPerSecond),
        turnSpeed * kSwerve.Maxrotation,
        robotCentric.getAsBoolean(),
        true);
    }   
}