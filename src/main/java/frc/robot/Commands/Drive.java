package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.Subsystems.holoDrive;

public class Drive extends CommandBase{
    private holoDrive m_HoloDrive;
    private DoubleSupplier Translation, strafe, rotation;
    private BooleanSupplier robotCentric;
    
    private SlewRateLimiter translationLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(0.5);
    
    public Drive(
        holoDrive m_HoloDrive,
        DoubleSupplier translation,
        DoubleSupplier rotation,
        DoubleSupplier strafe,
        BooleanSupplier robotCentric)
    {
        this.m_HoloDrive = m_HoloDrive;
        addRequirements(m_HoloDrive);

        this.Translation = translation;
        this.rotation = rotation;
        this.strafe = strafe;
        this.robotCentric = robotCentric;
    }

    @Override
    public void execute(){

        double translationVal = 
            translationLimiter.calculate(
                MathUtil.applyDeadband(Translation.getAsDouble(), 0.1)
            );
        double strafeVal =
            strafeLimiter.calculate(
                MathUtil.applyDeadband(strafe.getAsDouble(), 0.1)
            );
        double rotationVal =
            rotationLimiter.calculate(
                MathUtil.applyDeadband(rotation.getAsDouble(), 0.1)
            );
        
        m_HoloDrive.drive(new Translation2d(translationVal, strafeVal).times(kSwerve.MaxSpeedMetersPerSecond),
        rotationVal * kSwerve.Maxrotation,
        robotCentric.getAsBoolean(),
        false);
    }
    
}
