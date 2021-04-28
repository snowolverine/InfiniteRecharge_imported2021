/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
//import frc.robot.RobotMap;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveReverse;
import frc.robot.commands.AutoGrabIn;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.LoadBallTimed;
//import frc.robot.commands.SeekRight;
//import frc.robot.commands.RevShooter;
import frc.robot.commands.Space;
//import frc.robot.commands.TimedShoot;
//import frc.robot.commands.AutoLeft;
import frc.robot.commands.AutoTurretControl;
//import frc.robot.commands.GrabIn;
//import frc.robot.commands.LoadBall;
//import frc.robot.commands.LimeLightAimYellow;
import frc.robot.commands.LimeLightAimYellow;

public class GalacticSearch extends CommandGroup {
   
  public GalacticSearch() {
    

    
    addSequential(new AutoDriveReverse(.7,.7,.5));// Drop Large Grabber
    addSequential(new AutoDrive(-.7,-.7,1.2));  
        addSequential(new LimeLightAimYellow());
        addSequential(new AutoDrive(-.7,-.7,.2));
        
    addParallel(new AutoTurretControl(3));
    addParallel(new AutoGrabIn(3));  //1st ball pick up path A
    addParallel(new LoadBallTimed(1.5));
    addSequential(new Space(1));

      addSequential(new AutoTurn(270));
      addSequential(new Space(2));
      addSequential(new AutoDrive(-.7,-.7,.6));// 1st ball path B
      
    
   /*   addParallel(new AutoTurretControl(3));
    addParallel(new AutoGrabIn(3));//1st ball pick up path B
    addParallel(new LoadBallTimed(3));
    addSequential(new Space(1));

      addSequential(new AutoTurn(90));
      addSequential(new AutoDrive(-.7,-.7,.5));
      addSequential(new Space(2));
      addSequential(new LimeLightAimYellow(3));
      addSequential(new AutoDrive(-.7,-.7,.5));
      addSequential(new Space(2));

      addSequential(new AutoTurn(270));
      addSequential(new AutoDrive(-.7,-.7,.5));
      addSequential(new Space(2));
      addSequential(new LimeLightAimYellow(3));
      addSequential(new AutoDrive(-.7,-.7,.5));
      addSequential(new Space(2));

      addSequential(new AutoTurn(90));
      addSequential(new AutoDrive(-.7,-.7,.5));
      addSequential(new Space(2));
      addSequential(new LimeLightAimYellow(3));
      addSequential(new AutoDrive(-.7,-.7,.5));
      addSequential(new Space(2));*/


      



      /*addSequential(new Seek());
      addParallel(new LimeLightAimYellow());
    addSequential(new AutoDrive(-.7,-.7,.2));
    addSequential(new AutoDrive(0,0.,.3));
    addParallel(new AutoGrabIn());
      addParallel(new AutoTurretControl());
      addParallel(new LoadBallTimed());
      addSequential(new AllStop(0, 0, .1));
       /*addSequential( new AutoDrive(.35,.35,.9));
    addSequential(new AutoDrive(0,0,.1));
    addSequential(new AutoTurn(45)); 
    addSequential(new AutoDrive(.35,.35,.9));
    addParallel(new AutoGrabIn());
      addParallel(new AutoTurretControl());
      addParallel(new LoadBallTimed());
    addSequential(new AutoDrive(0,0,.05)); 
       
            
    //addSequential(new RevShooter());
    //addParallel(new TimedShoot());
    //addSequential(new LoadBallTimed());
    //addSequential(new Space(3));
    //addSequential(new AutoDrive(-.35, -.35, 2.25));
    addSequential(new Space(3));
    //addSequential(new AutoDrive(-.35, .35, 1.18));*/
  }
}
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the

