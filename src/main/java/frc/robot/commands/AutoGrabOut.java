/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.RobotMap;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AutoGrabOut extends Command {
  public AutoGrabOut(double time) {
    super.requires(Robot.grabber);
    setTimeout(time);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setTimeout(2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.grabber.run();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    Robot.grabber.setGrabMotor1(-RobotMap.MAX_SPEED_GRAB);    
   
}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.grabber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}      

