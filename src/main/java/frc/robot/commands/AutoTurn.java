package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//import frc.robot.RobotMap;
//import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends Command {
  private double  angle;
  private boolean isDone;
  private double initial;
  private double target;

  public AutoTurn(double angle) {
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initial = Robot.ahrs.getYaw();
    target = (initial + angle) % 360.0;

    System.out.println("initial: " + initial + ", target: " + target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(isDone)
      return;
    
    double heading = Robot.ahrs.getYaw();
    System.out.println("heading: " + heading);
    if(target > initial) {
      if(heading >= target) {
        System.out.println("stop: heading >= angle" + target);
        isDone = true;
        Robot.driveTrain.setRightMotor(0);
        Robot.driveTrain.setLeftMotor(0);
      } else {
        System.out.println("left");
        Robot.driveTrain.setRightMotor(0.25);
        Robot.driveTrain.setLeftMotor(0.25);
      }
    } else if(target < initial) {
      if(heading <= target) {
        System.out.println("stop: heading <= angle");
        isDone = true;
        Robot.driveTrain.setRightMotor(0);
        Robot.driveTrain.setLeftMotor(0);
      } else {
        System.out.println("right");
        Robot.driveTrain.setRightMotor(-0.3);
        Robot.driveTrain.setLeftMotor(-0.3);
      }
    } else {
        System.out.println("stop: target == initial");
        isDone = true;
        Robot.driveTrain.setRightMotor(0);
        Robot.driveTrain.setLeftMotor(0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.setLeftMotor(0);
    Robot.driveTrain.setRightMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}    

