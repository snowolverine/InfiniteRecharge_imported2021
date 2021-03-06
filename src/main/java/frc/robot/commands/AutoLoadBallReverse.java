 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AutoLoadBallReverse extends Command {
  private double length;
  //private final double kDriveTick2Feet = 1.0 / 2048 * 6 * Math.PI / 12;
  double leftLength = 0;
  double rightLength = 0;
  //final double kP = 0.05;
  public AutoLoadBallReverse(double x, double y, double z) {
    length = z;
    rightLength = x;  
    leftLength = y;
    super.requires(Robot.elevator);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setTimeout(length);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    final Timer m_timer = new Timer();
    m_timer.reset();
    m_timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    final Timer m_timer = new Timer();
    if (m_timer.get() < 1.5) {
      Robot.elevator.setLiftMotor1((RobotMap.MAX_SPEED_LIFT1));
      Robot.elevator.setLiftMotor2((-RobotMap.MAX_SPEED_LIFT2));
      Timer.delay(.1);
  } else {
    Robot.elevator.setLiftMotor1(0);
    Robot.elevator.setLiftMotor2(0);
    m_timer.stop();
  }
}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}   

