
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
import frc.robot.commands.LoadBallTimed;

//import frc.robot.Robot;
//import frc.robot.RobotMap;
//import frc.robot.commands.AutoDrive;
//import frc.robot.commands.AutoGrabIn;

import frc.robot.commands.RevShooter;
import frc.robot.commands.SeekLeft;
import frc.robot.commands.Space;
import frc.robot.commands.TimedShoot;

//import frc.robot.commands.RevShooter;
//import frc.robot.commands.Space;
//import frc.robot.commands.TimedShoot;
//import frc.robot.commands.AutoLeft;
//import frc.robot.commands.AutoRight;
//import frc.robot.commands.AutoTurretControl;
//import frc.robot.commands.GrabIn;
//import frc.robot.commands.LoadBall;
//import frc.robot.commands.LimeLightAimYellow;

public class InterstellarShot extends CommandGroup{
   
  public InterstellarShot() { 
    
    
    addSequential(new RevShooter());
        addParallel(new TimedShoot());
        addSequential(new LoadBallTimed(5));
        addSequential(new Space(3));
        addSequential  (new SeekLeft(4));
   addSequential(new AutoDrive(.7,.7,.1));
   
    
    
    
    //addParallel(new AutoGrabIn());
    //addParallel(new AutoTurretControl());
    //addParallel(new AutoLoadBall(.5,.5,3));
      // addSequential(new RevShooter());

       //addSequential( new AutoDrive(.35,.35,.9));
    //addSequential(new AutoDrive(0,0,.1));
    //addSequential(new AutoRight(.3, .3, .175)); 
    //addSequential(new AutoDrive(.35,.35,.9));
    //addSequential(new AutoDrive(0,0,.05)); 
       
            
    //addSequential(new RevShooter());
    //addParallel(new TimedShoot());
    //addSequential(new LoadBallTimed());
    //addSequential(new Space(3));
    //addSequential(new AutoDrive(-.35, -.35, 2.25));
    //addSequential(new Space(3));
    //addSequential(new AutoDrive(-.35, .35, 1.18));

  }
}
