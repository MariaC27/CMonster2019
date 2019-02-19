/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*---------------------------------------------- ------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

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

  //declare joysticks 
  public Joystick rightJoystick;
  public Joystick leftJoystick;
  public Joystick logitech;

  public JoystickButton setDistance;
  public JoystickButton setAngle;
  public JoystickButton intakeButton;
  public JoystickButton shootHatch;
  public JoystickButton retractHatch;
  public JoystickButton launchCatapultButton;
  public JoystickButton retractCatapultButton;
  public JoystickButton quarterSpeedButton;



  public OI(){
    //constructor
    //set whileHeld, whilePressed conditions in here
    rightJoystick = new Joystick(0);
    leftJoystick = new Joystick(1);
    logitech = new Joystick(2);
    
    setDistance = new JoystickButton(logitech, 1); //check these buttons
    setAngle = new JoystickButton(logitech, 2); //check these buttons 
    
    shootHatch = new JoystickButton(logitech, 6);
    retractHatch = new JoystickButton(logitech, 8);
    launchCatapultButton = new JoystickButton(logitech, 5);
    retractCatapultButton = new JoystickButton(logitech, 7);

    intakeButton = new JoystickButton(logitech, 3); 
    quarterSpeedButton = new JoystickButton(leftJoystick, 1); //on driver joystick 


    //when the button is pressed, robot should automatically move to setpoint, activating PID control
    setDistance.whenPressed(new ReachDistance(Robot.calculateDistance()));
    setAngle.whenPressed(new Turn(Robot.calculateAngle()));

    SmartDashboard.putNumber("distanceFromOI", Robot.calculateDistance());
    SmartDashboard.putNumber("angleFromOI", Robot.calculateAngle());



    intakeButton.whileHeld(new IntakeStart());
    intakeButton.whenReleased(new IntakeStop());

    shootHatch.whenPressed(new ShootHatch());
    retractHatch.whenPressed(new RetractHatch());

    launchCatapultButton.whenPressed(new LaunchCatapult());
    retractCatapultButton.whenPressed(new RetractCatapult());

    quarterSpeedButton.whenPressed(new ToggleQuarterSpeed());



  }


  //accessor methods 

  public Joystick getRightJoystick(){
    return rightJoystick;
  }

  public Joystick getLeftJoystick(){
    return leftJoystick;
  }

  public Joystick getLogitech(){
    return logitech;
  }

}
