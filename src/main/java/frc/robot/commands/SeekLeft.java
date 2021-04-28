/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import oi.limelightvision.limelight.frc.LimeLightYellow;

public class SeekLeft extends Command {
 
  
        
 
 
 //private double steering_adjust = 0.0;
 //private double kp=-0.1f;
 //private double min_command=0.05f;
    
    
    
   
   

  public SeekLeft (double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    setTimeout(time);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double tx = Robot.limelightYellow.getdegRotationToTarget();
    double ty = Robot.limelightYellow.getdegVerticalToTarget();
    boolean targetFound = Robot.limelightYellow.getIsTargetFound();
    double rotatevalue;
    System.out.println("Target Found"+ targetFound);
    System.out.println("tx:" + tx);
    System.out.println("ty:" + ty);
    
    if (targetFound) {
      if (tx < -10)
        rotatevalue = -0.2;
      else if (tx > 10)
        rotatevalue = 0.2;
      else
        rotatevalue = 0;

    } else {
      rotatevalue = -0.2;
    
    }
    Robot.driveTrain.setLeftMotor( rotatevalue);
    Robot.driveTrain.setRightMotor(rotatevalue);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.setRightMotor(0);
    Robot.driveTrain.setLeftMotor(0);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

 /* private double Estimated_Distance(double a2){
    double h1 = 6.0;
    double h2 = 36.0;
    double a1 = 0.0;
    return (h2-h1)/Math.tan(a1+a2);
  }*/
}
