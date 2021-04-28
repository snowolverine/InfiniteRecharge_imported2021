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

public class LimeLightAim extends Command {
 
  
        
 
 
 //private double steering_adjust = 0.0;
 //private double kp=-0.1f;
 //private double min_command=0.05f;
    
    
    
   
   

  public LimeLightAim() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double tx = Robot.limelightshot.getdegRotationToTarget();
    double ty = Robot.limelightshot.getdegVerticalToTarget();
    double rotatevalue;
    System.out.println("tx:" + tx);
    System.out.println("ty:" + ty);
    
      if (tx < -3)
        rotatevalue = -0.175;
      else if (tx > 3)
        rotatevalue = .175;
      else
        rotatevalue = 0;

        Robot.driveTrain.setLeftMotor( rotatevalue);
        Robot.driveTrain.setRightMotor(rotatevalue);
    
    }
    
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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