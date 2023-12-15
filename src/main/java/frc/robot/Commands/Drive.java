package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsys.Swerve;

public class Drive extends CommandBase{


    private final Swerve m_Swerve;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public Drive(
    Swerve m_Swerve,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier){
        this.m_Swerve = m_Swerve;
        this.rotationSupplier = rotationSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        addRequirements(m_Swerve);
    }
//TODO change max velocity values 
    public void execute(){
        double xSpeed = m_Swerve.deadband(translationXSupplier.getAsDouble()) * 1;
        double ySpeed = m_Swerve.deadband(translationYSupplier.getAsDouble()) * 1;
        double thetaSpeed = m_Swerve.deadband(rotationSupplier.getAsDouble()) * 1;


//Robot
        m_Swerve.Drive( 
            Constants.kSwerve.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed))
        );

//field
    // m_Swerve.Drive(Constants.kSwerve.DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
    //     xSpeed,
    //     ySpeed,
    //     thetaSpeed,
    //     m_Swerve.getPose().getRotation())));
    }

@Override
public boolean isFinished(){
    return false;
}

@Override
public void end(boolean interrupted){
    m_Swerve.Drive( 
            Constants.kSwerve.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(
                0,
                0,
                0))
        );
}   

}
