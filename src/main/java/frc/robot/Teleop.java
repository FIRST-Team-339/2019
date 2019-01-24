/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Autonomous.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 13, 2015
// CREATED BY: Nathanial Lydick
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for teleop mode
// should go here. Will be called each time the robot enters
// teleop mode.
// -----------------------------------------------------
// Periodic() - Periodic code for teleop mode should
// go here. Will be called periodically at a regular rate while
// the robot is in teleop mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.robot;

import frc.Hardware.Hardware;
import frc.vision.VisionProcessor;
import frc.vision.VisionProcessor.ImageType;
import frc.Utils.Forklift;
import frc.Utils.drive.Drive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class contains all of the user code for the Autonomous part of the
 * match, namely, the Init and Periodic code
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Teleop
{
/**
 * User Initialization code for teleop mode should go here. Will be called once
 * when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static boolean hasDoneTheThing = true;

public static void init ()
{

    hasDoneTheThing = false;

    LiveWindow.disableTelemetry(Hardware.pdp);

    Hardware.telemetry.printToConsole();
    Hardware.telemetry.printToShuffleboard();
    Hardware.telemetry.setTimeBetweenPrints(1000);

    // sets the gear to 0 at the beginning.
    Hardware.drive.setGear(0);

    Hardware.transmission.setJoystickDeadband(DEADBAND_VALUE);
    Hardware.transmission.enableDeadband();

    Hardware.rightFrontCANMotor.setInverted(false);
    Hardware.rightRearCANMotor.setInverted(false);
    Hardware.leftFrontCANMotor.setInverted(false);
    Hardware.leftRearCANMotor.setInverted(false);

    Hardware.drive.setGearPercentage(FIRST_GEAR_NUMBER,
            FIRST_GEAR_RATIO);
    Hardware.drive.setGearPercentage(SECOND_GEAR_NUMBER,
            SECOND_GEAR_RATIO);

    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------

    // ---------------------------------
    // Encoder resetting
    // ---------------------------------
    // Hardware.rightFrontDriveEncoder.reset();
    // Hardware.leftFrontDriveEncoder.reset();

    // ---------------------------------
    // setup motors
    // ---------------------------------
    // Hardware.rightDriveMotor.set(0);
    // Hardware.leftDriveMotor.set(0);


} // end Init

// tune pid loop

// private static boolean hasSeenTape = false;

/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */




public static void periodic ()
{

    // // UPDATES
    // Hardware.climber.climbUpdate();
    // // =================================================================
    // // OPERATOR CONTROLS
    // // =================================================================

    // if (Hardware.leftOperator.getRawButton(6) == true)
    // {
    // // Hardware.rightRearCANMotor.set(.5);
    // System.out.println("Trying to climb");
    // Hardware.climber.climb();
    // } else
    // {
    // // // Hardware.drive.drive(Hardware.leftDriver, Hardware.rightDriver);
    // }

    // if (Hardware.leftDriver.getRawButton(5) == true)
    // {
    // Hardware.climber.finishEarly();
    // }
    // Hardware.intakeDeployArm.set(Hardware.leftOperator.getY());


    // if (Hardware.leftOperator.getRawButton(4) == true)
    // {
    // Autonomous.descendFromLevelTwo();
    // }

    // @ANE

    // // TODO remove the next 3 functions once camera is tested

    // // Drive to the vision targets
    // if (Hardware.leftOperator.getRawButton(4)) {
    // hasDoneTheThing = false;
    // System.out.println("Done the thing: " + hasDoneTheThing);
    // }

    // if (!hasDoneTheThing) {
    // // System.out.println("Done the thing: " + hasDoneTheThing);
    // if (Hardware.driveWithCamera.driveToTarget(.3)) {
    // System.out.println("Has aligned hopefully");
    // hasDoneTheThing = true;
    // }
    // }
    // // check if we are getting blobs
    // if (Hardware.leftOperator.getRawButton(5)) {
    // System.out.println("Has Blobs?: " + Hardware.axisCamera.hasBlobs());

    // }
    // // turn on the ringlight
    // if (Hardware.leftOperator.getRawButton(6) && Teleop.hasDoneTheThing ==
    // true)
    // {
    // System.out.println("lets blind some wirers");
    // Hardware.axisCamera.setRelayValue(true);
    // }
    // // save image
    // if (Hardware.leftOperator.getRawButton(7)) {
    // Hardware.axisCamera.saveImage(ImageType.RAW);
    // Hardware.axisCamera.saveImage(ImageType.PROCESSED);
    // }

    // ----- Forklift controls -----

    // Hardware.lift.moveForkliftWithController(Hardware.rightOperator,
    // Hardware.rightOperator.getRawButton(5));

    // Hardware.lift.setLiftPositionByButton(Forklift.TOP_ROCKET_CARGO,
    // Forklift.DEFAULT_TELEOP_BUTTON_SPEED,
    // Hardware.rightOperator.getRawButton(6));

    // Hardware.lift.setLiftPositionByButton(Forklift.MIDDLE_ROCKET_CARGO,
    // Forklift.DEFAULT_TELEOP_BUTTON_SPEED,
    // Hardware.rightOperator.getRawButton(7));
    // else if (Hardware.rightOperator.getRawButton(7) == true)
    // Hardware.lift.setLiftPosition(Forklift.MIDDLE_ROCKET_CARGO,
    // Forklift.DEFAULT_TELEOP_BUTTON_SPEED);
    // else if (Hardware.rightOperator.getRawButton(1) == true)
    // Hardware.lift.setLiftPosition(Forklift.TOP_ROCKET_HATCH,
    // Forklift.DEFAULT_TELEOP_BUTTON_SPEED);
    // else if (Hardware.rightOperator.getRawButton(2) == true)
    // Hardware.lift.setLiftPosition(Forklift.MIDDLE_ROCKET_HATCH,
    // Forklift.DEFAULT_TELEOP_BUTTON_SPEED);
    // else if (Hardware.rightOperator.getRawButton(3) == true)
    // Hardware.lift.setLiftPosition(Forklift.LOWER_ROCKET_HATCH,
    // Forklift.DEFAULT_TELEOP_BUTTON_SPEED);

    // =================================================================
    // Driver Controls
    // =================================================================
    // @ANE
    // Teleop.teleopDrive();

    // =================================================================
    // Update State Machines
    // =================================================================
    Hardware.lift.update();

    // // =================================================================
    // // Telemetry
    // // =================================================================

    Hardware.telemetry.printToShuffleboard();
    Hardware.telemetry.printToConsole();

    // TODO untested code by Anna, Patrick, and Meghan
    // Teleop.teleopDrive();

} // end Periodic()

public static void printStatements ()
{

    if (Hardware.driverStation.isFMSAttached() == false)
        {
        // ==================================
        // Scale Alignment
        // ==================================

        // =================================
        // Motor
        // Prints the value of motors
        // =================================
        // Hardware.rightFrontCANMotor.get());
        // SmartDashboard.putNumber("Right Rear Drive Motor",
        // Hardware.rightRearCANMotor.get());
        // System.out.println("Left Front Drive Motor " +
        // Relay
        // =================================
        //
        // =================================
        // // Digital Inputs
        // =================================
        //
        // ---------------------------------

        // Switches
        // prints state of switches
        // ---------------------------------

        SmartDashboard.putBoolean("Disable SW",
                Hardware.autoLevelSwitch.isOn());

        // ---------------------------------
        // Encoders
        // ---------------------------------
        // System.out.println("Left Front Encoder Inches = "
        // + Hardware.leftFrontDriveEncoder.getDistance());
        // SmartDashboard.putNumber("Left Front Encoder Inches",
        // Hardware.leftFrontDriveEncoder.getDistance());

        // System.out.println("Left Front Encoder Ticks "
        // + Hardware.leftFrontDriveEncoder.get());
        // SmartDashboard.putNumber("Left Front Encoder Ticks",
        // Hardware.leftFrontDriveEncoder.get());

        // System.out.println("Right Front Inches = "
        // + Hardware.rightFrontDriveEncoder.getDistance());
        // SmartDashboard.putNumber("Right Front Encoder Inches",
        // Hardware.rightFrontDriveEncoder.getDistance());

        // System.out.println("Right Front Ticks "
        // + Hardware.rightFrontDriveEncoder.get());
        // SmartDashboard.putNumber("Right Front Encoder Ticks",
        // Hardware.rightFrontDriveEncoder.get());

        // ---------------------------------
        // Red Light/IR Sensors
        // prints the state of the sensor
        // ---------------------------------

        // =================================
        // Pneumatics
        // =================================

        // ---------------------------------
        // Compressor
        // prints information on the compressor
        // ---------------------------------

        // ---------------------------------
        // Solenoids
        // ---------------------------------

        // Analogs
        // =================================

        // ---------------------------------
        // pots
        // where the pot is turned to
        // ---------------------------------

        // ---------------------------------
        // GYRO
        // ---------------------------------

        // ---------------------------------
        // Sonar/UltraSonic
        // ---------------------------------

        // =========================
        // Servos
        // =========================

        // =================================
        // SPI Bus
        // =================================

        // -------------------------------------
        // Analog Interfaces
        // -------------------------------------

        // =================================
        // Connection Items
        // =================================

        // =================================
        // Cameras
        // prints any camera information required
        // =================================

        // =================================
        // Driver station
        // =================================

        // ---------------------------------
        // Joysticks
        // information about the joysticks
        // ---------------------------------
        /*
         * System.out.println("Right Driver Joystick " +
         * Hardware.rightDriver.getY());
         * SmartDashboard.putNumber("R Driver Y Joy",
         * Hardware.rightDriver.getY());
         * System.out.println("Left Driver Joystick " +
         * Hardware.leftDriver.getY());
         * SmartDashboard.putNumber("L Driver Y Joy",
         * Hardware.leftDriver.getY());
         * System.out.println("Right Operator Joystick " +
         * Hardware.rightOperator.getY());
         * SmartDashboard.putNumber("R Operator Y Joy",
         * Hardware.rightOperator.getY());
         * System.out.println("Left Operator Joystick "
         * + Hardware.leftOperator.getY());
         * SmartDashboard.putNumber("L Operator Y Joy",
         * Hardware.leftOperator.getY());
         */

        // =================================
        // KILROY ANCILLARY ITEMS
        // =================================
        // ---------------------------------
        // Gear number displayed to driver
        // ---------------------------------

        // ---------------------------------
        // timers
        // what time does the timer have now
        // ---------------------------------
        }

    SmartDashboard.updateValues();
} // end printStatements()

/**
 * Calls drive's main drive function so the robot can drive using joysticks
 *
 * Calls the shiftGears function from drive, so we can input the the gear shift
 * buttons and it will shift gears if we need it to.
 *
 * @author Anna, Meghan, and Patrick.
 */
public static void teleopDrive ()
{
    Hardware.drive.drive(Hardware.leftDriver, Hardware.rightDriver);

    Hardware.drive.shiftGears(
            Hardware.rightDriver.getRawButton(GEAR_DOWN_SHIFT_BUTTON),
            Hardware.leftDriver.getRawButton(GEAR_UP_SHIFT_BUTTON));

    // makes sure the gear never goes over 2
    if (Hardware.drive.getCurrentGear() >= MAX_GEAR_NUMBERS)
        {
        Hardware.drive.setGear(MAX_GEAR_NUMBERS - 1);
        }
}

// ================================
// Constants
// ================================

private static final int GEAR_UP_SHIFT_BUTTON = 3;

private static final int GEAR_DOWN_SHIFT_BUTTON = 3;

// The number of gears we want to not go over. There is no reason to make this
// more than 3 unless the code is fixed. Thanks McGee.
private static final int MAX_GEAR_NUMBERS = 2;

private static final int FIRST_GEAR_NUMBER = 0;

private static final int SECOND_GEAR_NUMBER = 1;

private static final double FIRST_GEAR_RATIO = .4;

private static final double SECOND_GEAR_RATIO = .7;

private static final double DEADBAND_VALUE = .2;
// ================================
// Variables
// ================================

} // end class
