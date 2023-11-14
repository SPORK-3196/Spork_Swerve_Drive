package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OI {
    public static final class XboxController {
        public static ShuffleboardTab Prim_Tab = Shuffleboard.getTab("primaryController");

        public static double PLJS_X = 0;
        public static double PLJS_Y = 0;
        public static double PRJS_X = 0;
        public static double PRJS_Y = 0;

        public static Boolean X_Button = false;

        public static GenericEntry PLJS_X_Entry = Prim_Tab.add("Left Joystick X", 0.0).getEntry();
        public static GenericEntry PLJS_Y_Entry = Prim_Tab.add("Left Joystick Y", 0.0).getEntry();
        public static GenericEntry PRJS_X_Entry = Prim_Tab.add("Right Joystick X", 0.0).getEntry();
        public static GenericEntry PRJS_Y_Entry = Prim_Tab.add("Right Joystick Y", 0.0).getEntry();

        public static GenericEntry x_Button_Entry = Prim_Tab.add("X_Bution", false).getEntry();
    }
}