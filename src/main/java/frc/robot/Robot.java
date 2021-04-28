/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
//import frc.robot.commands.DriverControls;
//import frc.robot.commands.LimeLightAimYellow;
import frc.robot.commands.ServoPin;
import frc.robot.subsystems.BarrelRacing;
//import frc.robot.commands.AutoDrive;
//import frc.robot.commands.DriverControls;
import frc.robot.subsystems.CenterAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GalacticSearch;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.InterstellarShot;
import frc.robot.subsystems.RightAuto;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.DriveTrain;
import oi.limelightvision.limelight.frc.LimeLight;
//import frc.robot.RobotMap;
//import oi.limelightvision.limelight.frc.LimeLight;
import frc.robot.subsystems.Shooter;
//import edu.wpi.first.wpiutil.math.MathUtil;


//import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.cscore.CvSink;
//import edu.wpi.cscore.CvSource;

//import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.core.Point;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
/**
 * This is a demo program showing the use of the navX MXP to implement a "rotate
 * to angle" feature.
 *
 * This example will automatically rotate the robot to one of four angles (0,
 * 90, 180 and 270 degrees).
 *
 * This rotation can occur when the robot is still, but can also occur when the
 * robot is driving. When using field-oriented control, this will cause the
 * robot to drive in a straight line, in whathever direction is selected.
 *
 * This example also includes a feature allowing the driver to "reset" the "yaw"
 * angle. When the reset occurs, the new gyro angle will be 0 degrees. This can
 * be useful in cases when the gyro drifts, which doesn't typically happen
 * during a FRC match, but can occur during long practice sessions.
 *
 * Note that the PID Controller coefficients defined below will need to be tuned
 * for your drive system.
 */
public class Robot extends TimedRobot {
  public static AHRS ahrs;
  public static boolean jankMode = false;
  public static Elevator elevator = new Elevator();
  public static DriveTrain driveTrain = new DriveTrain();
  public static Turret turret = new Turret();
  public static Grabber grabber = new Grabber();
  public static Shooter shooter = new Shooter();
  public static Climber climber = new Climber();
  public static LimeLight limelightYellow = new LimeLight("limelight-yellow");
  public static LimeLight limelightshot = new LimeLight("limelight-shot");
  public PIDController turnController;
  public double rotateToAngleRate;
  
  

  public static OI m_oi;
public static Object LimeLightAim;
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  static final double kToleranceDegrees = 2.0f;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    // Robot.comp.setClosedLoopControl(true);
    m_chooser.setDefaultOption("Center Auto", new CenterAuto());
    m_chooser.addOption("Right Auto", new RightAuto());
    m_chooser.addOption("GalacticSearch", new GalacticSearch());
    m_chooser.addOption("Barrel Racing", new BarrelRacing());
    m_chooser.addOption("Interstellar Shot", new InterstellarShot());
    SmartDashboard.putData("Auto mode", m_chooser);
    //BuiltInAccelerometer accelerometer;

    CameraServer.getInstance().startAutomaticCapture();
    // Thread c = new CameraThread();
    // c.setDaemon(true);
    // c.start();

    this.initSmartDashboard();

    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      ahrs = new AHRS(SerialPort.Port.kUSB);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    /* Note that the PIDController GUI should be added automatically to */
    /* the Test-mode dashboard, allowing manual tuning of the Turn */
    /* Controller's P, I and D coefficients. */
    /* Typically, only the P value needs to be modified. */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  private void initSmartDashboard() {
  }

  public void updateSmartDashboard() {
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    
    SmartDashboard.putNumber("LeftMotor Encoder", DriveTrain.leftMotor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("RightMotor Encoder", DriveTrain.rightMotor1.getSelectedSensorPosition());
    /* Display 6-axis Processed Angle Data                                      */
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    
    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */
    
    SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
    
    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
    
    //SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
    //SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    
    //SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
    //SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
    //SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
    //SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

    /* Display estimates of velocity/displacement.  Note that these values are  */
    /* not expected to be accurate enough for estimating robot position on a    */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially      */
    /* double (displacement) integration.                                       */
    
    //SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
    //SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
    //SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
    //SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
    
    /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
    /* NOTE:  These values are not normally necessary, but are made available   */
    /* for advanced users.  Before using this data, please consider whether     */
    /* the processed data (see above) will suit your needs.                     */
    
    /*SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
    SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
    SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
    SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
    SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
    SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
    //SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
    SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
    SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
    SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());*/
    
    /* Omnimount Yaw Axis Information                                           */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
    SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
    
    /* Sensor Board Information                                                 */
    SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
    
    /* Quaternion Data                                                          */
    /* Quaternions are fascinating, and are the most compact representation of  */
    /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    /* from the Quaternions.  If interested in motion processing, knowledge of  */
    /* Quaternions is highly recommended.                                       */
    /*SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
    SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
    SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
    SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
    
    /* Connectivity Debugging Support                                           */
    SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
    SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
    this.updateSmartDashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    ServoPin.servo.setAngle(180);
    // LimeLight.setLEDMode(ControlMode.1);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    Timer.delay(2.0);  // for 2 seconds

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    this.updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    //System.out.println("angle:"+ahrs.getAngle());
    
    //System.out.println("acceleromoeter"+accelerometer.getX());

  
          


  }
  
 
  
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

public static Subsystem ahrs() {
	return null;
}



}
/*class CameraThread extends Thread {
  @Override
  public void run() {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 360, 240, 20));
    CvSink cvSink = CameraServer.getInstance().getVideo();
    CvSource outputStream = CameraServer.getInstance().putVideo("the working one", 360, 240);
    Mat source = new Mat();
    Mat output = new Mat();
    int height = 3;
    Point a = new Point(0, 239 - height);
    Point b = new Point(359, 239 - height);
    camera.close();
    while(!Thread.interrupted()){
      cvSink.grabFrame(source);
      Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
      Imgproc.line(output, a, b, new Scalar(0), 1);
      outputStream.putFrame(output);
    }
  }
}
*/