/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.BlueZoneShot;
import frc.robot.commands.GrabIn;
//import frc.robot.commands.AutoDrive;
//import frc.robot.commands.AutoDrive;
//import frc.robot.commands.GrabIn;
import frc.robot.commands.GrabOut;
import frc.robot.commands.LimeLightAim;
import frc.robot.commands.LimeLightAimYellow;
import frc.robot.commands.ServoPin;
//import frc.robot.commands.LoadBall;
//import frc.robot.commands.LoadBallTimed;
//import frc.robot.commands.StopGrabber;
//import frc.robot.commands.UnloadBall;
//import frc.robot.subsystems.CenterAuto;
import frc.robot.commands.LargeGrabber;
import frc.robot.commands.LargeGrabberReverse;
//import oi.limelightvision.limelight.frc.LimeLight;
import frc.robot.commands.YawZero;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  public static XboxController driverController = new XboxController(RobotMap.DRIVER_CONTROLLER);
  public static XboxController operatorController = new XboxController(RobotMap.OPERATOR_CONTROLLER);
	
	Button dButtonA = new JoystickButton(OI.driverController, RobotMap.BUTTON_A);
	Button dButtonB = new JoystickButton(OI.driverController, RobotMap.BUTTON_B);
	Button dButtonX = new JoystickButton(OI.driverController, RobotMap.BUTTON_X);
	Button dButtonY = new JoystickButton(OI.driverController, RobotMap.BUTTON_Y);
	Button dButtonBack = new JoystickButton(OI.driverController, RobotMap.BACK_BUTTON);
	Button dButtonStart = new JoystickButton(OI.driverController, RobotMap.START_BUTTON);
	Button dButtonRightBumper = new JoystickButton(OI.operatorController, RobotMap.RIGHT_BUMPER);
	Button dButtonLeftBumper = new JoystickButton(OI.operatorController, RobotMap.LEFT_BUMPER);

	Button oButtonA = new JoystickButton(OI.operatorController, RobotMap.BUTTON_A);
	Button oButtonB = new JoystickButton(OI.operatorController, RobotMap.BUTTON_B);
	Button oButtonY = new JoystickButton(OI.operatorController, RobotMap.BUTTON_Y);
	Button oButtonX = new JoystickButton(OI.operatorController, RobotMap.BUTTON_X);
	Button oButtonBack = new JoystickButton(OI.operatorController, RobotMap.BACK_BUTTON);
	Button oButtonStart = new JoystickButton(OI.operatorController, RobotMap.START_BUTTON);
  	Button oButtonRightStick = new JoystickButton(OI.operatorController, RobotMap.RIGHT_STICK_BUTTON);
	Button oButtonRightBumper = new JoystickButton(OI.operatorController, RobotMap.RIGHT_BUMPER);
	Button oButtonLeftBumper = new JoystickButton(OI.operatorController, RobotMap.LEFT_BUMPER);

  	public boolean getOperatorButton(int axis) {
		return OI.operatorController.getRawButton(axis);
	}
	
	public boolean getDriverButton(int axis) {
		return OI.driverController.getRawButton(axis);
	}
	
	public double getOperatorRawAxis(int axis) {
		return OI.operatorController.getRawAxis(axis);
	}
	
	public double getDriverRawAxis(int axis) {
		return OI.driverController.getRawAxis(axis);
	}
	
	public int getOperatorPOV(){
		return OI.operatorController.getPOV();
	}

	public OI () {
		OI.driverController.setRumble(RumbleType.kRightRumble, 0);
		OI.driverController.setRumble(RumbleType.kLeftRumble, 0);
		OI.operatorController.setRumble(RumbleType.kRightRumble, 0);
		OI.operatorController.setRumble(RumbleType.kLeftRumble, 0);

		//this.oButtonA.whenPressed(new SetElevator(RobotMap.ELEVATOR_LOW));
		this.oButtonA.whenPressed(new GrabIn()); 
		this.oButtonA.whenPressed(new LargeGrabberReverse()); 
		this.oButtonB.whileHeld(new LargeGrabber());
		this.oButtonB.whileHeld(new GrabOut());
		//this.oButtonStart.whenPressed(new TurnToAngle(-45));
		//this.oButtonStart.whenPressed(new SetLight(true));
		//this.oButtonBack.whenPressed(new SetLight(false));
		this.oButtonX.whenPressed(new BlueZoneShot());
		//this.oButtonBack.whileHeld(new IncrementElevator(-RobotMap.ELEVATOR_INCREMENT));
		//this.oButtonStart.whileHeld(new IncrementElevator(RobotMap.ELEVATOR_INCREMENT));
		//this.oButtonBack.whenPressed(new SetElevatorSmartDashboard());

		//this.oButtonLeftBumper.whileHeld(new TurretControl());
		//this.oButtonRightBumper.whileHeld(new TurretControlReverse());

		//this.dButtonA.whenPressed(new TurnToZero());
		//this.dButtonX.whenPressed(new DriveForward(30));
		this.dButtonB.whenPressed(new YawZero(.2));
		this.dButtonX.whileHeld(new LimeLightAimYellow());
		this.dButtonB.whenPressed(new ServoPin(0));
		this.dButtonY.whileHeld(new LimeLightAim());
		//this.dButtonBack.whenPressed(new ZeroClimber(Dart.FRONT));
		//this.dButtonStart.whenPressed(new TurnToZero());
	}
}
