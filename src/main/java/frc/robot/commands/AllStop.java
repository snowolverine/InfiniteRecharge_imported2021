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

public class AllStop extends Command {
  private double length;
  //private final double kDriveTick2Feet = 1.0 / 2048 * 6 * Math.PI / 12;
  double leftLength = 0;
  double rightLength = 0;
  //final double kP = 0.05;

  public AllStop(double x, double y, double z) {
    length = z;
    rightLength = x;  
    leftLength = y;
    super.requires(Robot.driveTrain);
    super.requires(Robot.grabber);
    super.requires(Robot.shooter);
    super.requires(Robot.turret);
    super.requires(Robot.elevator);

    /*
     * // Use requires() here to declare subsystem dependencies // eg.
     * requires(chassis);
     */
    setTimeout(length);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // DriveTrain.leftMotor1.setSelectedSensorPosition(0);
    // DriveTrain.rightMotor1.setSelectedSensorPosition(0);
    // Robot.driveTrain.backwards();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // setpoint = 10;

    /*
     * double sensorPosition = DriveTrain.leftMotor1.getSelectedSensorPosition() *
     * kDriveTick2Feet; double errorLeft = leftLength - sensorPosition; double
     * errorRight = rightLength - sensorPosition; double outputSpeedLeft = kP *
     * errorLeft; double outputSpeedRight = kP * errorRight;
     */

    Robot.driveTrain.setRightMotor(0);
    Robot.driveTrain.setRightMotor(0);
    Robot.elevator.setLiftMotor1(0);
    Robot.elevator.setLiftMotor2(0);
    Robot.grabber.setGrabMotor1(0);
    Robot.turret.setTurretMotor1(0);
    Robot.shooter.setShootMotor1(0);
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

    
    
    


