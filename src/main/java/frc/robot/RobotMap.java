/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.*;
import frc.robot.subsystems.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  

  // public static AHRS ahrs;

  public static WPI_TalonSRX leftFrontTalon = new WPI_TalonSRX(2); //should be 2 for 2019
  public static WPI_TalonSRX rightFrontTalon = new WPI_TalonSRX(3); //should be 3 for 2019
  public static WPI_VictorSPX leftBackVictor = new WPI_VictorSPX(4);
  public static WPI_VictorSPX rightBackVictor = new WPI_VictorSPX(5);

  
  //coding a solenoid causes a CTRE CAN Timeout if there isn't a pcm on the robot 
   public static DoubleSolenoid hatchSolenoid = new DoubleSolenoid(1,2);
   public static Solenoid launchCatapultSolenoid = new Solenoid(0);
   public static DoubleSolenoid shootHatchSolenoid = new DoubleSolenoid(3,4);


  public static Spark intakeSpark = new Spark(0);

  public static DistancePID distancePID = new DistancePID();
  public static HeadingPID headingPID = new HeadingPID();

  public static Compressor robotCompressor = new Compressor(0);

  public static boolean init = false;


  public static void init(){

    init = true;

    SmartDashboard.putBoolean("RobotMapInit", init);



    rightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

   rightFrontTalon.config_kF(0, 0.146484375, 0);
      leftFrontTalon.config_kF(0, 0.146484375, 0);
    rightFrontTalon.config_kP(0, 0.03, 0); //0.00787568807
    leftFrontTalon.config_kP(0, 0.03, 0); //0.008254326923

          leftFrontTalon.configNominalOutputForward(0, 0);
    			leftFrontTalon.configNominalOutputReverse(0, 0);
    			rightFrontTalon.configNominalOutputForward(0,0);
    			rightFrontTalon.configNominalOutputReverse(0, 0);
    			
          leftFrontTalon.configPeakOutputForward(1, 0);
    		  leftFrontTalon.configPeakOutputReverse(-1, 0);
    			rightFrontTalon.configPeakOutputForward(1, 0);
    			rightFrontTalon.configPeakOutputReverse(-1, 0);
    	
     
    
  }
 



  
}
