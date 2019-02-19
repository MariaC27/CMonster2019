/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.JoystickSensitivity;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;



/**
 * Add your docs here.
 */
public class DriveBase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  DistancePID distancePID = RobotMap.distancePID;
  HeadingPID headingPID = RobotMap.headingPID;

  ArcadeDrive arcadeDrive = new ArcadeDrive();
  JoystickSensitivity joystickSensitivity = new JoystickSensitivity();
 

  WPI_TalonSRX leftTalon = RobotMap.leftFrontTalon;
  WPI_TalonSRX rightTalon = RobotMap.rightFrontTalon;
  WPI_VictorSPX leftVictor = RobotMap.leftBackVictor;
  WPI_VictorSPX rightVictor = RobotMap.rightBackVictor;
 
  public boolean toggleCheck = false;
  double joystickScale = 1;


  Compressor compressor = RobotMap.robotCompressor;
  double rightMotorSpeed = 0;
  double leftMotorSpeed = 0; 
  double[] returnData = new double[2];

  double rightDistance = 0;
  double leftDistance = 0;
  public static double averageDistance = 0;
  double DISTANCE_PER_ENCODER_PULSE = (Math.PI * 0.5)/4096;  //for 6 inch wheels 
  double VELOCITY_UNITS_PER_100MS = 6570;

  double moveSpeed;
  double rotateSpeed;

  boolean checkAuto = false;
  
 


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoystick());
    //DriveWithJoystick is the default command 
  }

  public void enableDriveBase(){
    rightTalon.setSelectedSensorPosition(0);
    leftTalon.setSelectedSensorPosition(0);
    distancePID.enable();
    headingPID.enable();
    leftTalon.setNeutralMode(NeutralMode.Brake);
    rightTalon.setNeutralMode(NeutralMode.Brake);
  
    //turns on the compressor 

    compressor.setClosedLoopControl(true);

    leftDistance = 0;
    rightDistance = 0;

  }

  public void disableDriveBase(){
    rightTalon.disable();
    leftTalon.disable();
    rightVictor.disable();
    leftVictor.disable();
    distancePID.disable();
    distancePID.disable(); 

  }

  public void toggleSpeed(){

    if(toggleCheck == true){
      joystickScale = 1;
    }

    else {
      joystickScale = 0.25;
    }

  }


public void DriveAutonomous(){

  checkAuto = true;
  SmartDashboard.putBoolean("driveAutoMethod", checkAuto);


    moveSpeed = distancePID.getOutput(); 
    rotateSpeed = headingPID.getOutput();
    
    SmartDashboard.putNumber("moveSpeed", moveSpeed);
    SmartDashboard.putNumber("rotateSpeed", rotateSpeed);
		
		returnData = arcadeDrive.calculateSpeed(moveSpeed, rotateSpeed);
		leftMotorSpeed = returnData[0];
    rightMotorSpeed = returnData[1];

    SmartDashboard.putNumber("leftMotorSpeed", leftMotorSpeed);
    SmartDashboard.putNumber("rightMotorSpeed", rightMotorSpeed);
    

    //need to multiply by max RPM 

    rightTalon.set(ControlMode.Velocity, rightMotorSpeed * VELOCITY_UNITS_PER_100MS);
    leftTalon.set(ControlMode.Velocity, leftMotorSpeed * VELOCITY_UNITS_PER_100MS);
   
    rightVictor.follow(rightTalon);
    leftVictor.follow(leftTalon);

  rightDistance = (rightTalon.getSelectedSensorPosition()) *DISTANCE_PER_ENCODER_PULSE;
  leftDistance = (leftTalon.getSelectedSensorPosition()) *DISTANCE_PER_ENCODER_PULSE;

  

  SmartDashboard.putNumber("rightSelectedSensorPosAuto", rightTalon.getSelectedSensorPosition());
  SmartDashboard.putNumber("leftSelectedSensorPosAuto", leftTalon.getSelectedSensorPosition());

  //1024 count encoders -->  4096 native units per rotation (4X encoder)

  leftDistance *= -1; //invert one side so values are positive 
  averageDistance = (rightDistance + leftDistance) / 2;

  SmartDashboard.putNumber("rightDistance", rightDistance);
  SmartDashboard.putNumber("leftDistance", leftDistance);
  SmartDashboard.putNumber("averageDistance", averageDistance);



}


public void JoystickInputs(Joystick LeftJoystick, Joystick RightJoystick, Joystick Logitech){

  SmartDashboard.putNumber("rightSelectedSensorPosTeleop", rightTalon.getSelectedSensorPosition());
  SmartDashboard.putNumber("leftSelectedSensorPosTeleop", leftTalon.getSelectedSensorPosition());

  rightMotorSpeed = (RightJoystick.getY() *-1) * joystickScale;
  leftMotorSpeed = LeftJoystick.getY() * joystickScale;


//closed loop - doesn't work currently because both sides move when one joystick is pressed 
  // rightTalon.set(ControlMode.Velocity, rightMotorSpeed * VELOCITY_UNITS_PER_100MS); //multiply by velocity units per 100 ms
  //   leftTalon.set(ControlMode.Velocity, leftMotorSpeed * VELOCITY_UNITS_PER_100MS);

    SmartDashboard.putNumber("rightTalonTeleopOutput", (rightMotorSpeed * VELOCITY_UNITS_PER_100MS));
    SmartDashboard.putNumber("leftTalonTeleopOutput", (leftMotorSpeed * VELOCITY_UNITS_PER_100MS));
    


  leftTalon.set(leftMotorSpeed);
  rightTalon.set(rightMotorSpeed); //percent VBus open loop

  //assign values from the talons to the joysticks
  //call this method in DriveWithJoystick

  leftMotorSpeed = joystickSensitivity.GetOutput(leftMotorSpeed);
	rightMotorSpeed = joystickSensitivity.GetOutput(rightMotorSpeed);

  
  rightVictor.follow(rightTalon);
  leftVictor.follow(leftTalon);

}



}
