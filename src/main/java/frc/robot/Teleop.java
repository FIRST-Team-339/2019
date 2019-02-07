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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Relay.Value;
// import com.sun.org.apache.xerces.internal.impl.xpath.XPath.Axis;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Utils.Forklift;
import frc.vision.VisionProcessor.ImageType;

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
public static void init ()
{

    LiveWindow.disableTelemetry(Hardware.pdp);

    Hardware.telemetry.setTimeBetweenPrints(1000);

    Hardware.transmission.setJoystickDeadband(DEADBAND_VALUE);
    Hardware.transmission.enableDeadband();

    Hardware.gyro.reset();

    // ---------------------------------
    // drive class initialization
    // ---------------------------------
    Hardware.drive.setGearPercentage(FIRST_GEAR_NUMBER,
            FIRST_GEAR_RATIO);
    Hardware.drive.setGearPercentage(SECOND_GEAR_NUMBER,
            SECOND_GEAR_RATIO);
    // sets the gear to 0 at the beginning.
    Hardware.drive.setGear(0);


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
    // =================================================================
    // OPERATOR CONTROLS
    // =================================================================

    // Forklifts

    Hardware.lift.moveForkliftWithController(Hardware.rightOperator,
            Hardware.forkliftOverride.get());

    Hardware.lift.setLiftPositionByButton(Forklift.CARGO_SHIP_CARGO,
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED,
            Hardware.cargoShipCargoButton);

    Hardware.lift.setLiftPositionByButton(Forklift.CARGO_SHIP_HATCH,
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED,
            Hardware.cargoShipHatchButton);

    Hardware.lift.setToNextHigherPreset(
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED,
            Hardware.nextHigherLiftHeightButton,
            Hardware.chooseCargoRocketHeights.get());

    Hardware.lift.setToNextLowerPreset(
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED,
            Hardware.nextLowerLiftHeightButton,
            Hardware.chooseCargoRocketHeights.get());


    // =================================================================
    Hardware.lift.update();

    Hardware.manipulator.masterUpdate();

    Hardware.climber.climbUpdate();

    teleopDrive();

    // Hardware.manipulator.moveArmByJoystick(Hardware.leftOperator);

    individualTest();

    // Hardware.telemetry.printToShuffleboard();

    // Hardware.telemetry.printToConsole();

    printStatements();
}
// end Periodic()


// Individual testing methods for each programmer. Each programmer should //put
// their testing code inside their own method.
// Author: Guido Visioni

private static void individualTest ()
{
    // ashleyTest();
    // connerTest();
    // coleTest();
    // guidoTest();
    // patrickTest();
    // annaTest();
    // meghanTest();
    // nithyaTest();
}

private static void ashleyTest ()
{
    if (Hardware.leftDriver.getRawButton(3) == true)
        {
        // if (Hardware.alignByTape.align() == true)
            {
            System.out.println(
                    "MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW");
            }
        } else
        {
        Hardware.drive.drive(Hardware.leftDriver, Hardware.rightDriver);
        }


    if (Hardware.rightDriver.getRawButton(3) == true)
        {
        Hardware.alignByTape.resetForAlign();
        }
    // if (Hardware.leftDriver.getRawButton(5) == true)
    // {
    // Autonomous.descendFromLevelTwo(true);
    // System.out
    // .println(
    // "HELP WE'VE FALLEN AND WE CAN'T GET BACK UP we hzve yoted");
    // } else if (Hardware.rightDriver.getRawButton(5) == true)
    // {
    // System.out.println("WERE DOING SOMETHING AT LEAST");
    // Hardware.climber.climb();
    // }

    // if (Hardware.leftDriver.getRawButton(3) == true)
    // {
    // Autonomous.descentState = Autonomous.DescentState.STANDBY;
    // }

    // if (Hardware.leftDriver.getRawButton(5) == true)
    // {
    // Hardware.climber.finishEarly();
    // }


    // if (Hardware.descendButton.isOnCheckNow() == true)
    // {
    // Autonomous.descendFromLevelTwo();
    // }
}

private static boolean started = false;

private static void connerTest ()
{
    Hardware.axisCamera.setRelayValue(Value.kOn);

    if (started == false && Hardware.leftOperator.getRawButton(9))
        {
        started = true;
        if (Hardware.driveWithCamera.visionTest(.4))
            {
            started = false;
            }
        }
}

private static void coleTest ()
{

    // Manipulator

    // Hardware.manipulator.intakeOuttakeByButtonsSeperated(
    // Hardware.intakeTrigger.get(),
    // Hardware.outtakeButton.get(),
    // Hardware.intakeOverride.get());


}

private static void guidoTest ()
{

}

private static void patrickTest ()
{
    SmartDashboard.putBoolean("Is Patrick Happy?",
            Hardware.rightDriver.getRawButton(4));
}

private static void annaTest ()
{

}

private static void meghanTest ()
{

}

private static void nithyaTest ()
{

}


public static void printStatements ()
{
    if (Hardware.driverStation.isFMSAttached() == false)
        {
        // ==================================
        // Scale Alignment
        // ==================================

        // =================================
        // Motors
        // =================================

        // System.out.println("Arm motor: " + Hardware.armMotor.get());
        // Hardware.telemetry.printToConsole(
        // "Arm motor: " + Hardware.armMotor.get());
        // System.out.println("Lift Motor One "
        // + Hardware.liftMotor.get());
        // System.out.println("RF Drive Motor " +
        // Hardware.rightFrontCANMotor.get());
        // System.out.println("LF Drive Motor "
        // + Hardware.leftFrontCANMotor.get());
        // System.out.println("RR Drive Motor " +
        // Hardware.rightRearCANMotor.get());
        // System.out.println("LR Drive Motor "
        // + Hardware.leftRearCANMotor.get());
        // System.out.println("Arm Roller "
        // + Hardware.armRoller.get());

        // =================================
        // Relay
        // =================================
        // System.out.println(
        // "Ring light relay: " + Hardware.ringLightRelay.get());


        // =================================
        // Digital Inputs
        // =================================
        //
        // ---------------------------------

        // Switches
        // prints state of switches
        // System.out.println(
        // "Left auto switch: " + Hardware.leftAutoSwitch.isOn());
        // System.out.println(
        // "Right auto switch: "
        // + Hardware.rightAutoSwitch.isOn());
        // System.out.println("Center auto switch: "
        // + Hardware.autoCenterSwitch.isOn());
        // System.out.println(
        // "Level one switch: " + Hardware.levelOneSwitch.isOn());
        // System.out.println(
        // "Level two switch: " + Hardware.levelTwoSwitch.isOn());
        // System.out.println("Auto disable switch: "
        // + Hardware.autoDisableSwitch.isOn());
        // System.out.println("Auto 6 position switch: "
        // + Hardware.autoSixPosSwitch.getPosition());

        // ---------------------------------

        // SmartDashboard.putBoolean("Disable SW",
        // Hardware.autoLevelSwitch.isOn());

        // ---------------------------------
        // Encoders
        // ---------------------------------
        // System.out.println("Left Front Encoder Inches = "
        // + Hardware.leftFrontDriveEncoder.getDistance());
        // SmartDashboard.putNumber("Left Front Encoder Inches"+
        // Hardware.leftFrontDriveEncoder.getDistance());

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
        // System.out.println("Arm IR: " + Hardware.armIR.get());
        // System.out
        // .println("Left back IR: " + Hardware.leftBackIR.get());
        // TODO resest right back IR
        // System.out.println(
        // "Right back IR: " + Hardware.rightBackIR.get());
        // =================================
        // Pneumatics
        // =================================

        // ---------------------------------
        // Compressor
        // prints information on the compressor
        // ---------------------------------
        // System.out.println("Compressor: " +
        // Hardware.compressor.getCompressorCurrent());

        // ---------------------------------
        // Solenoids
        // ---------------------------------
        // TODO test arm intake solenoid
        // System.out.println("Arm intake solenoid forward: "
        // + Hardware.armIntakeSolenoid.getForward());
        // System.out.println("Arm intake solenoid reverse: "
        // + Hardware.armIntakeSolenoid.getReverse());


        // Analogs
        // =================================

        // ---------------------------------
        // pots
        // where the pot is turned to
        // ---------------------------------



        // ----------------------------------
        // Potentiometers
        // ----------------------------------
        // TODO test potentiometers
        // System.out.println("Delay pot: " + Hardware.delayPot.get());
        // System.out.println("Intake deploy sensor: "
        // + Hardware.intakeDeploySensor.get());

        // ---------------------------------
        // Sonar/UltraSonic
        // ---------------------------------

        // System.out.println("ultrasonic " + Hardware.frontUltraSonic
        // .getDistanceFromNearestBumper());

        // =========================
        // Servos
        // =========================

        // =================================
        // SPI Bus
        // =================================

        // -------------------------------------
        // Analog Interfaces
        // -------------------------------------

        // ---------------------------------
        // GYRO
        // ---------------------------------

        // System.out.println("Gyro: " + Hardware.gyro.getAngle());

        // =================================
        // Connection Items
        // =================================

        // =================================
        // Cameras
        // prints any camera information required
        // =================================

        // -------------------------------------
        // Axis/USB Camera class
        // -------------------------------------


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
