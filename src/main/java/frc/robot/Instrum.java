package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Instrum {

    //private static int _loops =0;
    //private static int _timesInMotionMagic =0;

    public static void Process(TalonFXSensorCollection tal, StringBuilder sb){
        SmartDashboard.putNumber("SensorVel", tal.getIntegratedSensorVelocity());
        SmartDashboard.putNumber("SensorPos", tal.getIntegratedSensorPosition());
        SmartDashboard.putNumber("MotorOutputPercent", tal.getIntegratedSensorAbsolutePosition());
    }

    
}
