package frc.robot.commands.LightingModes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class Spork extends CommandBase {
    
    Lights light;

    public Spork(Lights light){
        this.light = light;
        addRequirements(light);
    }

    public void initialize(){
        light.blank();
        light.start();
    }

    public void execute(){
        light.runSpork();
        light.start();
    }

    public void end(){
        light.blank();
    }

    public boolean runswhiledisabled(){
        return true;
    }

    public boolean isFinished(){
        return false;
    }

}