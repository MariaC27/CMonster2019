/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/* Originally from team 558. We just added comments.
See Object block format section in
http://www.cmucam.org/projects/cmucam5/wiki/Porting_Guide#Object-block-format
	
Each object is sent in an "object block" (see below).
All values in the object block are 16-bit words, sent least-signifcant byte first (little endian). So, 
for example, when sending the sync word 0xaa55, Pixy sends 0x55 (first byte) then 0xaa (second byte).


Object block format

Bytes    16-bit word    Description
----------------------------------------------------------------
0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
4, 5     y              signature number
6, 7     y              x center of object
8, 9     y              y center of object
10, 11   y              width of object
12, 13   y              height of object

To mark between frames an extra sync word (0xaa55) is inserted. This means that a new image frame is indicated by either:

Two sync words sent back-to-back (0xaa55, 0xaa55), or
a normal sync followed by a color-code sync (0xaa55, 0xaa56).

So, a typical way to parse the serial stream is to wait for two sync words and then start parsing the object block, 
using the sync words to indicate the start of the next object block, and so on.
 */

import java.util.ArrayList; // ArrayList supports dynamic arrays that can grow as needed.
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class VisionBase extends Subsystem {
	
	private static final int[] distances = { 0, 113, 81, 62, 49, 41, 35, 30,
			28, 25, 22, 21 }; //are in pixels difference between block0 center and block1 center.
	public static final double PIXY_FOV = 75; //Changes with lens change - field-of-view horizontal in degrees
	public static final double IMAGE_WIDTH = 320.0;
	public static final double GEAR_WIDTH_FT = 1.166;
	public static final int BLOCK_SIZE = 14;    //Each block is 14 bytes
	static final int PIXY_START_BYTE_1 = 0x55;  // used to find a syncword = (0xaa55)
	static final int PIXY_START_BYTE_2 = 0xaa;
	private static final double DEGREES_PER_PIXEL = PIXY_FOV / IMAGE_WIDTH;
	public static int PIXY_ADDRESS = 0x54;
	private I2C port;
	private boolean inRange;
	private double offset;
	private int test = 0;
	private double CenterX0;
	private double CenterX1;
	private double CenterY0;
	private double CenterY1;

	
	
	private ArrayList<PixyBlock> pixyBlocks = new ArrayList<>(); 
	// Declare pixyBlocks as an ArrayList of type PixyBlock. <>  infers PixyBlock
	// ArrayList supports dynamic arrays that can grow as needed.

	public VisionBase()
	{
		try
		{
			port = new I2C(I2C.Port.kOnboard, PIXY_ADDRESS);
		} catch (Exception e)
		{
			System.out.println("e : " + e.getLocalizedMessage());
		}
	}

	public void initDefaultCommand()
	{
		//setDefaultCommand(new ReadCommand());
	}

	public void setLastOffset(double offset)
	{
		if (offset > 1){
		this.offset = offset;
		setInRange(true);
		}
	}

    //accessor methods 
	public double getLastOffset()
	{		
		return offset;
	}
	
	
	public double getCenterX0()  
	{
		return CenterX0;
	}
	
	public double getCenterX1()  
	{
		return CenterX1;
		}
	
	public double getCenterY0()  
	{
		return CenterY0;
	}
	
	public double getCenterY1()  
	{
		return CenterY0;
	}

	public void setInRange(boolean inRange)
	{
		this.inRange = inRange;
	}

	public double getDistance(double width, double targetCenter)
	{
		int index = 0;
		int smallest = 1000;
		for (int i = 0; i < distances.length; i++)
		{
			if (Math.abs(width - distances[i]) < smallest)
			{
				index = i;
				smallest = Math.abs((int) (width - distances[i]));
			}
		}
		double distance = index;
		return index;
	}

	public ArrayList<PixyBlock> read()  // return ArrayList of type PixtBlock  
	{

		//initial variables for use in the method 
		pixyBlocks.clear();              // clears old data?
		pixyBlocks = new ArrayList<>();  //<> infers type PixyBlock
		byte[] bytes = new byte[64]; //array 
		port.read(0x54, 64, bytes);    // read pixy data and assign to bytes.
		int index = 0;
		
/*		
		for (; index < bytes.length - 1; ++index){  //test loop
			SmartDashboard.putNumber("Pixy data ", bytes[index]);
			SmartDashboard.putNumber("Index data ", index);
			try {
				Thread.sleep(2000, 0);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} */
		
		
		
		//for loop 
		for (; index < bytes.length - 1; ++index)  //looking for start of a sinc word
		{
			int b1 = bytes[index];  // This converts bytes to signed int.
			if (b1 < 0)				//check each value in the array to see if its negative 
				b1 += 256;          /// If < 0 add 256 to convert the magnitude to positive int.
									//if the value is negative, add 256 to guaruntee positive value 
			int b2 = bytes[index + 1];  //check the value next to b1 to see if it is negative
			if (b2 < 0)					//if not, make positive 
				b2 += 256;
			
			if (b1 == PIXY_START_BYTE_1 && b2 == PIXY_START_BYTE_2)  // Check if first 2 bytes equal a "sync
				// word", which indicates the start of a block of valid data. syncWord == 0xaa55
				break;    // index  = start of sync word.

			}    //end of for loop 
		
		if (index == 63)       //no syncWord found - no data 
			return null;
		else if (index == 0)   // syncWord found at beginning, omit first two bytes
			index += 2;
		
		//new image frame is indicated by two sync words sent back-to-back. We just found the first from above.
		//Next we parse each object block using the sync words to indicate the start of the next object block.
		
		int byteOffset = index;
		
		for (; byteOffset < bytes.length - BLOCK_SIZE - 1;)
		{
			// checking for sync block   
			int b1 = bytes[byteOffset];  
			if (b1 < 0)
				b1 += 256;
		
			int b2 = bytes[byteOffset + 1];  
			if (b2 < 0)
				b2 += 256;

			if (b1 == PIXY_START_BYTE_1 && b2 == PIXY_START_BYTE_2){  // Check if first 2 bytes equal a "syncword"
			
			
				//syncword found  - beginning of next block of data. copy block into temp buffer
				byte[] temp = new byte[BLOCK_SIZE];
				StringBuilder sb = new StringBuilder("Data : ");   //StringBuilder appends data using a single buffer.
				

				//start another for loop 
				for (int tempOffset = 0; tempOffset < BLOCK_SIZE; ++tempOffset)
				{
					temp[tempOffset] = bytes[byteOffset + tempOffset];
					sb.append(temp[tempOffset] + ", ");
				}
		
				//Declare block variable as PixyBlock. Send temp data to the method bytesToBlock 
				PixyBlock block = bytesToBlock(temp);  // make block = to method return.
				
				//Added so blocks are only added if their signature is 1 to remove noise from signal
				if (block.signature == 1)
				{
					pixyBlocks.add(block); //Store the whole block of data in pixyBlocks.
					byteOffset += BLOCK_SIZE - 1; //syncword found subtract block size -1
				} else
					++byteOffset; //Found syncword but not for signature 1. add one try again.
			} else  
				++byteOffset; //Did not find syncword try again.
		}

		if (pixyBlocks != null && pixyBlocks.size() > 0)
		{
			if (pixyBlocks.size() >= 2)
			{
				PixyBlock leftBlock;
				PixyBlock rightBlock;
				if (pixyBlocks.get(0).centerX > pixyBlocks.get(1).centerX)
					
					
				{
					leftBlock = pixyBlocks.get(1);
					rightBlock = pixyBlocks.get(0);
				} else
				{
					leftBlock = pixyBlocks.get(0);
					rightBlock = pixyBlocks.get(1);
				}
				
				
				double difference = (rightBlock.centerX + leftBlock.centerX) / 2;
				
				setLastOffset(difference);
				double total = (rightBlock.centerX) - (leftBlock.centerX);
				getDistance(total, difference);
				
				
				
				CenterX0 = pixyBlocks.get(0).centerX;  
				CenterX1 = pixyBlocks.get(1).centerX;
				CenterY0 = pixyBlocks.get(0).centerY;  
				CenterY1 = pixyBlocks.get(1).centerY;
				
				
				SmartDashboard.putNumber("pixyCenterX0VisionBase", CenterX0);
				SmartDashboard.putNumber("pixyCenterX1VisionBase ", CenterX1);
				SmartDashboard.putNumber("pixyCenterY0VisionBase", CenterY0);
				SmartDashboard.putNumber("pixyCenterY1VisionBase", CenterY1);
			
			}
			
			else
			{
				//IMAGE_WIDTH = 320.0; 160 is the center of the image
				setLastOffset(160); //Keeps robot going straight if nothing is picked up
				
				//Removed so pixy only outputs new offset if two blocks are found
				//setLastOffset(pixyBlocks.get(0).centerX);
			}
		} 
		else
		{
			setLastOffset(160); //Keeps robot going straight if nothing is picked up
			setInRange(false);
		}
		

		return pixyBlocks;
	}

	
	
	
	
	
	//This just excepts data(an array of bytes)- converts it to an Int and retuns the data in a "PixyBlock" format.
	public PixyBlock bytesToBlock(byte[] bytes) // It works no need to get into details.
	{
		PixyBlock pixyBlock = new PixyBlock();
		pixyBlock.sync = bytesToInt(bytes[1], bytes[0]);
		pixyBlock.checksum = bytesToInt(bytes[3], bytes[2]);
		pixyBlock.signature = orBytes(bytes[5], bytes[4]);
		pixyBlock.centerX = ((((int) bytes[7] & 0xff) << 8) | ((int) bytes[6] & 0xff));  // converts to int's
		pixyBlock.centerY = ((((int) bytes[9] & 0xff) << 8) | ((int) bytes[8] & 0xff));  // & is a AND bitwise operator
		pixyBlock.width = ((((int) bytes[11] & 0xff) << 8) | ((int) bytes[10] & 0xff));  // << 8 shifts bit pattern
		pixyBlock.height = ((((int) bytes[13] & 0xff) << 8) | ((int) bytes[12] & 0xff)); // | is a OR bitwise operator
		return pixyBlock;
	}

	public int orBytes(byte b1, byte b2)     // "OR" two bytes is like adding the values.
										     // "& 0xff yields the same value for the lower 8 bits
	{										// also "masks" all upper bits.
		return (b1 & 0xff) | (b2 & 0xff); 
	}

	//All values in the object block are 16-bit words, sent least-signifcant byte first (
	public int bytesToInt(int b1, int b2)  // This converts bytes to signed int.
	{
		if (b1 < 0)
			b1 += 256;     // If < 0 add 256 to convert the magnitude to positive int.

		if (b2 < 0)
			b2 += 256;    // If < 0 add 256 to convert the magnitude to positive int.
		SmartDashboard.putNumber("B1  ", b1);
		int intValue = b1 * 256; // least-signifcant byte (Why * 256 )I think this is a bug.
		intValue += b2;         // most-significant byte (b2 should be *256) because it's most-significant.
		return intValue;   // This is only used for sync and checksum signal(we're not using them.)
	}

	public class PixyBlock    // class - Variables and data types 
	{						  //These are not raw data types, but formatted data types(int's).
		public int sync;
		public int checksum;
		public int signature;
		public int centerX;
		public int centerY;
		public int width;
		public int height;
	}
	
	
}


