/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive {

	double leftMotorSpeed;
	double rightMotorSpeed;
	double moveSpeed;
	double rotateSpeed;
	double[] robotSpeed = new double[2];
	static double Output;
	static double usePIDOutput;
	
	public double[] calculateSpeed (double moveSpeed, double rotateSpeed) {
		
		if (moveSpeed > 0.0) {
            if (rotateSpeed > 0.0) {
                leftMotorSpeed = moveSpeed - rotateSpeed;
                rightMotorSpeed = Math.max(moveSpeed, rotateSpeed);
            } else {
                leftMotorSpeed = Math.max(moveSpeed, -rotateSpeed);
                rightMotorSpeed = moveSpeed + rotateSpeed;
            }
        } else {
            if (rotateSpeed > 0.0) {
                leftMotorSpeed = -Math.max(-moveSpeed, rotateSpeed); 
                rightMotorSpeed = moveSpeed + rotateSpeed;
            } else {
                leftMotorSpeed = moveSpeed - rotateSpeed;
                rightMotorSpeed = -Math.max(-moveSpeed, -rotateSpeed);
            }
        }
		
		
		rightMotorSpeed *= -1; // invert motor speed command.
        
        
        robotSpeed[0] = leftMotorSpeed;  // array element 0 is always the left motor speed.
        robotSpeed[1] = rightMotorSpeed;  // array element 1 is always the right motor speed.
		
		
		return robotSpeed;
		
	}
	
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		Output = (output);
				
	}
	
	public static double getOutput(){
		return Output;
	}
	
	
	
	
}
