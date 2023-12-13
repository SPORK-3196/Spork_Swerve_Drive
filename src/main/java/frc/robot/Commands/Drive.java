


package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsys.Swerve;


public class Drive extends CommandBase {
    public Swerve swerve;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public Drive(Swerve m_swerve, DoubleSupplier translationXSupplier,
     DoubleSupplier translationYSupplier,
     DoubleSupplier rotationSupplier){
    this.swerve = m_swerve;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

        addRequirements(m_swerve);
     }

     public void initial(){
       // swerve.zeroEncoders();
        
     }

    @Override
     public void execute() 
     {
        swerve.drive(new ChassisSpeeds(
        translationXSupplier.getAsDouble(),
        translationYSupplier.getAsDouble(),
        rotationSupplier.getAsDouble()));

         /*swerve.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                rotationSupplier.getAsDouble(),
                swerve.getGyroRotation())
         );
         */

     }
 
     @Override
     public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
     }
 }
    