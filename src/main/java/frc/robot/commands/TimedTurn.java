/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.*;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class TimedTurn extends TimedCommand {

  private double angle = 0;
  private final HeadingPID headingPID = RobotMap.headingPID;
  private final AHRS ahrs = Robot.ahrs;
  /**
   * Add your docs here.
   */
  public TimedTurn(double timeout, double angleSetpoint) {
    super(timeout);
    angle = angleSetpoint;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    headingPID.enable();
    headingPID.resetPID();
    Robot.driveBase.enableDriveBase();
    ahrs.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    headingPID.setSetpoint(-angle); //should be -angle 
    Robot.driveBase.DriveAutonomous();
  }

  // Called once after timeout
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
