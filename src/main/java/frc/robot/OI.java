
package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OI {
      public static final class modules{
        public static ShuffleboardTab modules = Shuffleboard.getTab("modules");

        public static double frontRightAngle = 0.0;
        public static double backRightAngle = 0.0;
        public static double frontLeftAngle = 0.0;
        public static double backLeftAngle = 0.0;

        public static GenericEntry frontRightAngleEntry = modules.add("frontRightAngle",0.0).getEntry();
        public static GenericEntry frontLeftAngleEntry = modules.add("frontLeftAngle",0.0).getEntry();
        public static GenericEntry backRightAngleEntry = modules.add("backRightAngle",0.0).getEntry();
        public static GenericEntry backLeftAngleEntry = modules.add("backLeftAngle",0.0).getEntry();
    }
    
}
