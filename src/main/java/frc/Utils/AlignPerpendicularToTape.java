/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.Utils;

import frc.HardwareInterfaces.LightSensor;
import frc.Utils.drive.Drive;

/**
 * Add your docs here.
 */
public class AlignPerpendicularToTape
{

private LightSensor leftBackIR = null;

private LightSensor rightBackIR = null;

private Drive drive = null;


private boolean leftIRTripped = false;

private boolean rightIRTripped = false;

private boolean leftTrippedFirst = false;


public AlignPerpendicularToTape (LightSensor leftBackIR,
        LightSensor rightBackIR, Drive drive)
{
    this.drive = drive;
    this.rightBackIR = rightBackIR;
    this.leftBackIR = leftBackIR;
}

public boolean align ()
{
    double leftSpeed = -.2;
    double rightSpeed = -.2;

    if (leftBackIR.isOn() == true)
        {
        leftIRTripped = true;
        }

    if (rightBackIR.isOn() == true)
        {
        rightIRTripped = true;
        }

    if (rightIRTripped == true)
        {
        System.out.println("WE GOT TO THE RIGHT");
        rightSpeed = 0.0;
        }
    if (leftIRTripped == true)
        {
        System.out.println("WE GOT TO THE LEFT");
        leftSpeed = 0.0;
        }
    drive.getTransmission().driveRaw(leftSpeed, rightSpeed);

    if (leftIRTripped == true && rightIRTripped == false)
        {
        leftTrippedFirst = true;
        }

    if (leftIRTripped == true && rightIRTripped == true)
        {
        if (leftTrippedFirst == true)
            {
            if (leftBackIR.isOn() == true && rightBackIR.isOn() == true)
                {
                System.out.println("WERE TRYING TO STOP");
                return true;
                } else
                {
                drive.getTransmission().driveRaw(.2, -.2);
                }
            } else
            {
            if (leftBackIR.isOn() == true && rightBackIR.isOn() == true)
                {
                System.out.println("WERE TRYING TO STOP");
                return true;
                } else
                {
                drive.getTransmission().driveRaw(-.2, .2);
                }
            }
        }

    return false;
}






}
