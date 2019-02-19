/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

/**
 * Add your docs here.
 */
public class JoystickSensitivity {

    double Sensitivity = 1.0;
	double Output;
	
	public double GetOutput(double Input) {
		
		
		
		
		Output = Sensitivity * Math.pow(Input, 3) + (1-Sensitivity) * Input;
			
		return Output;
		
		
	}




}
