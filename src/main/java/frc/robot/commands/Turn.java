/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.HeadingPID;

public class Turn extends Command {

  private final HeadingPID headingPID = RobotMap.headingPID;

  private double angle = 0;

  public Turn(double a) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    setTimeout(2.5);

    angle = a;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    headingPID.enable();
    headingPID.resetPID();
    Robot.driveBase.enableDriveBase();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    headingPID.setSetpoint(angle);
    Robot.driveBase.DriveAutonomous();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut()/*headingPID.onTarget()*/;
    //have time out so can return to driver control even if PIDs don't finish
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
