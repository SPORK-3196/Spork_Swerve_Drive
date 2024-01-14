package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OI {


    public static final class swervevalues{

        public static ShuffleboardTab SwerveTab =  Shuffleboard.getTab("Swerve");
        

        public static double X_Joy = 0;
        public static double Y_Joy = 0;
        public static double Rot_Joy = 0;

        public static double CanCoder = 0;
        public static double RotMotorEncoder = 0; 
        public static double desiredState = 0; 


        public static GenericEntry X_JoyEntry = SwerveTab.add("X_Joy", 0.0).getEntry();
        public static GenericEntry Y_JoyEntry = SwerveTab.add("Y_Joy", 0.0).getEntry();
        public static GenericEntry Rot_JoyEntry = SwerveTab.add("Rot_Joy", 0.0).getEntry();


        public static GenericEntry CanCoderEntry = SwerveTab.add("CanCoder Value", 0.0).getEntry();
        public static GenericEntry RotMotorEncoderEntry = SwerveTab.add("Rotation Motor Encoder", 0.0).getEntry();
        public static GenericEntry desiredStateEntry = SwerveTab.add("Desired State", 0.0).getEntry();

        public static GenericEntry positionErrorEntry = SwerveTab.add("error" , 0.0).getEntry();

    }
    
}
