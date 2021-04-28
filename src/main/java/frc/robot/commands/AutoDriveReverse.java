/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.RobotMap;
//import frc.robot.subsystems.DriveTrain;

public class AutoDriveReverse extends Command {
  private double angle;
  private double length;
  //private final double kDriveTick2Feet = 1.0 / 2048 * 6 * Math.PI / 12;
  double leftLength = 0;
  double rightLength = 0;
  //final double kP = 0.05;

  public AutoDriveReverse(double x, double y, double t) {
    length = t;
    rightLength = x;  
    leftLength = y;
    super.requires(Robot.driveTrain);
    /*
     * // Use requires() here to declare subsystem dependencies // eg.
     * requires(chassis);
     */
    setTimeout(length);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    angle = Robot.ahrs.getYaw();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // setpoint = 10;

    double heading = Robot.ahrs.getYaw();
    double dAngle1 = angle - heading;
    double dAngle2 = (angle + 360) - heading;
    double dAngleMin = Math.min(dAngle1, dAngle2);

    if(dAngleMin < -1) {
      Robot.driveTrain.setRightMotor(-0.25);
      Robot.driveTrain.setLeftMotor(0);
    } else if(dAngleMin > 1) {
      Robot.driveTrain.setRightMotor(0);
      Robot.driveTrain.setLeftMotor(0.25);
    } else {
      Robot.driveTrain.setRightMotor(.35);
      Robot.driveTrain.setLeftMotor(-.35);
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
    Robot.driveTrain.setLeftMotor( 0);
    Robot.driveTrain.setRightMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
