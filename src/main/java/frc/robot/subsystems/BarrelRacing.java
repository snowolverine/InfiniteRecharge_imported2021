/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoTurn;


//import frc.robot.Robot;
//import frc.robot.RobotMap;
//import frc.robot.commands.AutoDrive;
//import frc.robot.commands.AutoGrabIn;

//import frc.robot.commands.RevShooter;
//import frc.robot.commands.Space;
//import frc.robot.commands.TimedShoot;
//import frc.robot.commands.AutoLeft;
//import frc.robot.commands.AutoRight;
//import frc.robot.commands.AutoTurretControl;
//import frc.robot.commands.GrabIn;
//import frc.robot.commands.LoadBall;
//import frc.robot.commands.LimeLightAimYellow;


public class BarrelRacing extends CommandGroup{
   
  public BarrelRacing() { 
    
    addSequential(new AutoDrive(1,1,4));// drive forward
    addSequential(new AutoTurn(90));
    addSequential(new AutoDrive(1,1,3));// 1st barrel
    addSequential(new AutoTurn(90));
    /*addSequential(new AutoDrive(1,1,3.5));//1st barrel
    addSequential(new AutoTurn(90));
    addSequential(new AutoDrive(1,1,3));//1st barrel
    addSequential(new AutoTurn(90));
    addSequential(new AutoDrive(1,1,6));//2nd barrel
    addSequential(new AutoTurn(270));
    addSequential(new AutoDrive(1,1,3));//2nd barrel
    addSequential(new AutoTurn(270));
    addSequential(new AutoDrive(1,1,3.5));//2nd barrel
    addSequential(new AutoTurn(270));
    addSequential(new AutoDrive(1,1,3.5));//3rd barrel
    addSequential(new AutoTurn(270));
    addSequential(new AutoDrive(1,1,4));//3rd barrel
    addSequential(new AutoTurn(270));
    addSequential(new AutoDrive(1,1,3));//finish
    addSequential(new AutoTurn(270));
    addSequential(new AutoDrive(1,1,7));*/
  }
    @Override
  protected boolean isFinished() {
    return isTimedOut();
  }
}

