/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.HashMap;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ReachDistance;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.*;
import frc.robot.RobotMap;
import edu.wpi.first.networktables.NetworkTable;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static ReachDistance reachDistance = new ReachDistance(0);
  public static DriveBase driveBase = new DriveBase(); 
  public static IntakeBase intakeBase = new IntakeBase();
  public static HatchBase hatchBase = new HatchBase();
  public static CatapultBase catapultBase = new CatapultBase();
  //declare and initialize subsystems 
  VisionBase pixyCam = new VisionBase();
  public static double pixyX0;
  public static double pixyX1;
  public static double pixyY0;
  public static double pixyY1;

 
  //how to use a second pcm is necessary: 
  //first parameter specifies which pcm to use
  //second two parameters are the ports on that pcm 
  //the two pcms need to have different CAN IDs
  //wire second pcm into pdp with smaller ports with 20 amp breaker 

  
  public static AHRS ahrs;

  //auto chooser 
  Command m_autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();


  
//vision processing methods, return distance and angle 
  public static double calculateDistance(){

    //more vision variables 
  



  //test with Vision subsystem and pixy

    //variables for vision: 
  //for use with both distance and angle calculations
  double centerXDifference; //declared

  //for use with distance calculations 
  double optimalPixelDifference = 214; //need to SET - CALIBRATE
  double optimalRobotDistance = 1; //need to SET - CALIBRATE
  double actualRobotDistance = 0; //set to zero to start 
  double distanceSetpoint = 0.0; //thing to set the PID setpoint to 
  HashMap <Integer, Double> visionMap = new HashMap<>(10);
  //create a hashmap to store calibrated values 

    visionMap.put(0, 295.0);
    visionMap.put(1, 207.0);
    visionMap.put(2, 115.0);
    visionMap.put(3, 79.0);
    visionMap.put(4, 57.0);
    visionMap.put(5, 45.0);
    visionMap.put(6, 38.0);
    visionMap.put(7, 33.0);
    visionMap.put(8, 30.0);
    visionMap.put(9, 26.0);
    visionMap.put(10, 25.0);
//VISION PROCESSING CALCULATIONS:


  


//display the data from the pixy
  SmartDashboard.putNumber("pixyX0DistanceMethod", pixyX0);
  SmartDashboard.putNumber("pixyX1DistanceMethod", pixyX1);
  SmartDashboard.putNumber("pixyY0DistanceMethod", pixyY0);
  SmartDashboard.putNumber("pixyY1DistanceMethod", pixyY1);



//DISTANCE CALCULATIONS
  double centerXnonABS = pixyX1 - pixyX0;
  centerXDifference =  Math.abs(centerXnonABS);

  SmartDashboard.putNumber("centerXDifferentDistance", centerXDifference);


  int indexOfCenter = 0;
  int indexOfAdjacent = 0;
  double compare = 0;
  double smallest = 10000; //large number so the first number in the array will always be smaller 
  double m = 0;
  double b = 0; 


//to prevent nullpointer exception, do a preliminary check to see if pixels are smaller than 
//highest index. If so, just use top 2 indexes 
if(centerXDifference < visionMap.get(visionMap.size() -1)){
  indexOfCenter = visionMap.size() -1;
  indexOfAdjacent = visionMap.size();
}


//can do -1 here because of preliminary check up top 
for (int i = 0; i < visionMap.size() - 1; i++){
  compare = centerXDifference - visionMap.get(i);
  double compareABS = Math.abs(centerXDifference - visionMap.get(i));


        if (compareABS < smallest){
             smallest = compareABS;
              indexOfCenter = i;
        }

}

  SmartDashboard.putNumber("compare", compare);
  SmartDashboard.putNumber("Smallest", smallest);
  SmartDashboard.putNumber("IndexOfCenter", indexOfCenter);


  if (compare < 0){
    indexOfAdjacent = indexOfCenter -1;
    double upper = visionMap.get(indexOfCenter);
    double lower = visionMap.get(indexOfAdjacent);


      //new linear interpolation 
      m = (lower - upper) / (indexOfAdjacent - indexOfCenter);
      b = upper - (m * indexOfCenter);
      actualRobotDistance = (centerXDifference - b)/m;


    SmartDashboard.putNumber("indexOfAdjacent", indexOfAdjacent);
    SmartDashboard.putNumber("upper", upper);
    SmartDashboard.putNumber("lower", lower);

  }

    else {
        indexOfAdjacent = indexOfCenter +1;
        double upper = visionMap.get(indexOfAdjacent); //causing crash when add one 
        double lower = visionMap.get(indexOfCenter);


        m = (upper - lower)/(indexOfAdjacent - indexOfCenter);
        b = lower - (m * indexOfCenter);
        actualRobotDistance = (centerXDifference - b) / m;

      SmartDashboard.putNumber("indexOfAdjacent", indexOfAdjacent);
      SmartDashboard.putNumber("upper", upper);
      SmartDashboard.putNumber("lower", lower);
    }


    distanceSetpoint = actualRobotDistance - optimalRobotDistance;

        SmartDashboard.putNumber("actualRobotDistance", actualRobotDistance);
        SmartDashboard.putNumber("distanceSetpoint2019", distanceSetpoint);

      return distanceSetpoint;

  } //end of method 



  //vision processing to return angle 
  public static double calculateAngle(){
    //more vision variables 
   

      //variables for vision: 
    //for use with both distance and angle calculations
    double centerXDifference; //declared



    //for use with angle calculations 
    double PIXY_FOV_DEGREES = 75; //got from website 
    double IMAGE_WIDTH_PIXELS = 320; //got from website
    double DEGREES_PER_PIXEL = PIXY_FOV_DEGREES / IMAGE_WIDTH_PIXELS; //degrees per pixel is the constant to modify by 
    double optimalCenterXPixel = 160; //change as calibrated
    double imgCenterXPixel;
    double centerPixelDifference;
    double headingSetpoint; //thing to set the PID setpoint to 

  //test with Vision subsystem and pixy

  

  SmartDashboard.putNumber("pixyX0AngleMethod", pixyX0);
  SmartDashboard.putNumber("pixyX1AngleMethod", pixyX1);
  SmartDashboard.putNumber("pixyY0AngleMethod", pixyY0);
  SmartDashboard.putNumber("pixyY1AngleMethod", pixyY1);


      //ANGLE CALCULATIONS 
      imgCenterXPixel = (pixyX0 + pixyX1)/2;  //pixel in the center of the two targets
      SmartDashboard.putNumber("imgCenterPixel", imgCenterXPixel);
      centerPixelDifference = imgCenterXPixel - optimalCenterXPixel; 
      SmartDashboard.putNumber("centerPixelDifference", centerPixelDifference);
      //distance this center is from the actual center of the image 

      

      headingSetpoint = centerPixelDifference * DEGREES_PER_PIXEL;
      //multiply by constant to convert to degrees  
      SmartDashboard.putNumber("headingSetpoint2019", headingSetpoint);

  //end of vision calculations 

        return headingSetpoint;

  } //end of method 

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    RobotMap.init();
    
  
    chooser.setDefaultOption("Default Auto", new DriveWithJoystick());
    //default auto is using the joysticks during the 15 second period 
    chooser.addOption("Reach Distance", reachDistance);
   
    SmartDashboard.putData("Auto Side", chooser);
    
   
  
   
    ahrs = new AHRS(I2C.Port.kMXP, (byte) 100);

    SmartDashboard.putNumber("navXYaw", ahrs.getYaw());
   //need to import I2C above 
   
   Robot.driveBase.enableDriveBase();

 
    //streaming camera 
   CameraServer.getInstance().startAutomaticCapture();

   
  
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Robot.driveBase.disableDriveBase();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {



   driveBase.enableDriveBase();

    m_autonomousCommand = chooser.getSelected();

    
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
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    driveBase.enableDriveBase();
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

    driveBase.enableDriveBase();
    Scheduler.getInstance().run();

    pixyCam.read();
  pixyX0= pixyCam.getCenterX0();
  pixyX1= pixyCam.getCenterX1();
  pixyY0= pixyCam.getCenterY0();
  pixyY1= pixyCam.getCenterY1();

    //call vision methods
    calculateDistance();
    calculateAngle();

    SmartDashboard.putNumber("teleopYaw", ahrs.getYaw());
    
  }

  


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Robot.driveBase.enableDriveBase();
		Robot.driveBase.DriveAutonomous();
  }

}
