package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class FieldOriented extends CommandBase {
    
    public FieldOriented(){

    }


    @Override
    public void initialize(){
        RobotContainer.fieldOriented = true;
        System.out.println(RobotContainer.fieldOriented);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        RobotContainer.fieldOriented = false;
        System.out.println(RobotContainer.fieldOriented);
    }
}