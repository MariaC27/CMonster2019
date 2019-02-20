/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class HeadingPID extends PIDSubsystem {

  double Output;
  double yawOutput;
  /**
   * Add your docs here.
   */
  public HeadingPID() {
    // Intert a subsystem name and PID values here
    super("HeadingPID", 0.04, 0, 0);
    setAbsoluteTolerance(0.35);
    setOutputRange(-0.4, 0.4); //can make this half speed, etc 
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  public void resetPID(){
    getPIDController().reset();
  }

 
  public double getOutput() {
    return Output;
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
   
    yawOutput = (double) Robot.ahrs.getYaw();
		yawOutput *= -1;
		return yawOutput;
     
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    Output = output;
  }
}
