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
    switch (Hardware.whichRobot)
        {
        case KILROY_2018:
            initTeleop2018();
            break;

        default:
        case KILROY_2019:
            initTeleop2019();
            break;

        case TEST_BOARD:
            break;
        } // end switch
} // end Init


/**
 * User Initialization code for teleop mode should go here. Will be called once
 * when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void initTeleop2018 ()
{
    Hardware.telemetry
            .setTimeBetweenPrints(TELEMETRY_PERIODICITY_KILROY_XIX);

    Hardware.transmission
            .setJoystickDeadband(DEADBAND_VALUE_KILROY_XIX);
    Hardware.transmission.enableDeadband();

    Hardware.gyro.reset();

    // ---------------------------------
    // drive class initialization
    // ---------------------------------
    Hardware.drive.setGearPercentage(FIRST_GEAR_NUMBER,
            FIRST_GEAR_RATIO_KILROY_XIX);
    Hardware.drive.setGearPercentage(SECOND_GEAR_NUMBER,
            SECOND_GEAR_RATIO_KILROY_XIX);
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

} // end initTeleop2018()

/**
 * User Initialization code for teleop mode should go here. Will be called once
 * when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void initTeleop2019 ()
{
    Hardware.telemetry
            .setTimeBetweenPrints(TELEMETRY_PERIODICITY_KILROY_XX);

    Hardware.transmission.setJoystickDeadband(DEADBAND_VALUE_KILROY_XX);
    Hardware.transmission.enableDeadband();

    Hardware.gyro.reset();

    // ---------------------------------
    // drive class initialization
    // ---------------------------------
    Hardware.drive.setGearPercentage(FIRST_GEAR_NUMBER,
            FIRST_GEAR_RATIO_KILROY_XX);
    Hardware.drive.setGearPercentage(SECOND_GEAR_NUMBER,
            SECOND_GEAR_RATIO_KILROY_XX);
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

} // end initTeleop2019()

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

    SmartDashboard.putString("Arm Potentiometer",
            "" + Hardware.intakeDeploySensor.get());

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

    // buttons
    if (Hardware.climbOneButton.isOnCheckNow() == true
            && Hardware.climbTwoButton.isOnCheckNow() == true)
        {
        Hardware.climber.climb();
        }
    else
        {
        teleopDrive();
        }

    // buttons to cancel everything
    if (Hardware.cancelTwoButton.get() == true
            && Hardware.cancelOneButton.get() == true)
        {
        Hardware.climber.finishEarly();
        Autonomous.endAutoPath();

        }

    // TODO pls yell at me if I puch this


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

    // if (Hardware.leftDriver.getRawButton(3) == true)
    // {
    // // if (Hardware.alignByTape.align() == true)
    // {
    // System.out.println(
    // "MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW");
    // }
    // } else
    // {
    // Hardware.drive.drive(Hardware.leftDriver, Hardware.rightDriver);
    // }


    // if (Hardware.rightDriver.getRawButton(3) == true)
    // {
    // Hardware.alignByTape.resetForAlign();
    // }
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
} // end ashleyTest()

private static boolean started = false;

private static void connerTest ()
{
    if (!started)
        {

        }
    // Hardware.axisCamera.setRelayValue(Value.kOn);

    // System.out.println("Right or left: "
    // + Hardware.driveWithCamera.getTargetSide());
    // if (Hardware.driveWithCamera.visionTest(.4))
    // {
    // System.out.println("ALigned amybe i hope");
    // }

} // end connerTest()

private static void coleTest ()
{

    // if (Hardware.outtakeButton.get())
    // Hardware.armMotor.set(.7);
    // else if (Hardware.intakeOverride.get())
    // Hardware.armMotor.set(-.7);
    // else
    // Hardware.armMotor.set(0.0);

    // Manipulator

    Hardware.manipulator.moveArmByJoystick(Hardware.leftOperator,
            Hardware.deployOverride.get());

    Hardware.manipulator.moveArmByButton(45, .5,
            Hardware.setDeploy45DegreeButton);

    Hardware.manipulator.intakeOuttakeByButtonsSeperated(
            Hardware.intakeTrigger.get(),
            Hardware.outtakeButton.get(),
            Hardware.intakeOverride.get());

} // end coleTest()

private static void guidoTest ()
{
    SmartDashboard.putNumber("Lift Encoder",
            Hardware.lift.getForkliftHeight());
} // end guidoTest()

private static void patrickTest ()
{

    if (Hardware.rightDriver
            .getRawButton(CYCLE_BACKGROUND_COLOR) == true
            && isCurrentlyChanging == false)
        {
        switch (backgroundColor)
            {
            case CLEAR:
                backgroundColor = CurrentBackground.BLUE;
                isCurrentlyChanging = true;
                break;

            case BLUE:
                backgroundColor = CurrentBackground.ORANGE;
                isCurrentlyChanging = true;
                break;

            case ORANGE:
                backgroundColor = CurrentBackground.CLEAR;
                isCurrentlyChanging = true;
                break;
            }
        }

    switch (backgroundColor)
        {
        case CLEAR:
            isBlue = false;
            isOrange = false;
            break;

        case BLUE:
            isBlue = true;
            isOrange = false;
            break;

        case ORANGE:
            isBlue = true;
            isOrange = true;
            break;
        }
    SmartDashboard.putBoolean("Blue", isBlue);
    SmartDashboard.putBoolean("Orange", isOrange);

    if (Hardware.rightDriver.getRawButton(4) == false)
        {
        isCurrentlyChanging = false;
        }
} // end patrickTest()

private static void annaTest ()
{

} // end annaTest()

private static void meghanTest ()
{

} // end meghanTest()

private static void nithyaTest ()
{

} // end nithyaTest()


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
        // Hardware.telemetry.printToConsole("Lift Motor One "
        // + Hardware.liftMotor.get());
        // System.out.println("RF Drive Motor " +
        // Hardware.rightFrontCANMotor.get());
        // Hardware.telemetry.printToConsole("RF Drive Motor " +
        // Hardware.rightFrontCANMotor.get());
        // System.out.println("LF Drive Motor "
        // + Hardware.leftFrontCANMotor.get());
        // Hardware.telemetry.printToConsole("LF Drive Motor "
        // + Hardware.leftFrontCANMotor.get());
        // System.out.println("RR Drive Motor " +
        // Hardware.rightRearCANMotor.get());
        // Hardware.telemetry.printToConsole("RR Drive Motor " +
        // Hardware.rightRearCANMotor.get());
        // System.out.println("LR Drive Motor "
        // + Hardware.leftRearCANMotor.get());
        // Hardware.telemetry.printToConsole("LR Drive Motor "
        // + Hardware.leftRearCANMotor.get());
        // System.out.println("Arm Roller "
        // + Hardware.armRoller.get());
        // Hardware.telemetry.printToConsole("Arm Roller "
        // + Hardware.armRoller.get());

        // =================================
        // Relay
        // =================================
        // System.out.println(
        // "Ring light relay: " + Hardware.ringLightRelay.get());
        // Hardware.telemetry.printToConsole(
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
        // Hardware.telemetry.printToConsole(
        // "Left auto switch: " + Hardware.leftAutoSwitch.isOn());
        // System.out.println(
        // "Right auto switch: "
        // + Hardware.rightAutoSwitch.isOn());
        // Hardware.telemetry.printToConsole(
        // "Right auto switch: "
        // + Hardware.rightAutoSwitch.isOn());
        // System.out.println("Center auto switch: "
        // + Hardware.autoCenterSwitch.isOn());
        // Hardware.telemetry.printToConsole("Center auto switch: "
        // + Hardware.autoCenterSwitch.isOn());
        // System.out.println(
        // "Level one switch: " + Hardware.levelOneSwitch.isOn());
        // Hardware.telemetry.printToConsole(
        // "Level one switch: " + Hardware.levelOneSwitch.isOn());
        // System.out.println(
        // "Level two switch: " + Hardware.levelTwoSwitch.isOn());
        // Hardware.telemetry.printToConsole(
        // "Level two switch: " + Hardware.levelTwoSwitch.isOn());
        // System.out.println("Auto disable switch: "
        // + Hardware.autoDisableSwitch.isOn());
        // Hardware.telemetry.printToConsole("Auto disable switch: "
        // + Hardware.autoDisableSwitch.isOn());
        // System.out.println("Auto 6 position switch: "
        // + Hardware.autoSixPosSwitch.getPosition());
        // Hardware.telemetry.printToConsole("Auto 6 position switch: "
        // + Hardware.autoSixPosSwitch.getPosition());

        // ---------------------------------

        // SmartDashboard.putBoolean("Disable SW",
        // Hardware.autoDisableSwitch.isOn());
        // Hardware.telemetry.printToConsole("Disable SW" +
        // Hardware.autoDisableSwitch.isOn());

        // ---------------------------------
        // Encoders
        // ---------------------------------
        // System.out.println("Left Front Encoder Inches = "
        // + Hardware.leftFrontDriveEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Left Front Encoder Inches = "
        // + Hardware.leftFrontDriveEncoder.getDistance());

        // System.out.println("Left front encoder ticks: "
        // + Hardware.leftFrontDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Left front encoder ticks: "
        // + Hardware.leftFrontDriveEncoder.get());

        // System.out.println("Right Front Inches = "
        // + Hardware.rightFrontDriveEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Right Front Inches = "
        // + Hardware.rightFrontDriveEncoder.getDistance());

        // System.out.println("Right Front Ticks "
        // + Hardware.rightFrontDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Right Front Ticks "
        // + Hardware.rightFrontDriveEncoder.get());

        // System.out.println("Left rear encoder inches: "
        // + Hardware.leftRearDriveEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Left rear encoder inches: "
        // + Hardware.leftRearDriveEncoder.getDistance());

        // System.out.println("Left rear encoder ticks: "
        // + Hardware.leftRearDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Left rear encoder ticks: "
        // + Hardware.leftRearDriveEncoder.get());

        // System.out.println("Right rear encoder distance: "
        // + Hardware.rightRearDriveEncoder.getDistance());
        // Hardware.telemetry
        // .printToConsole("Right rear encoder distance: "
        // + Hardware.rightRearDriveEncoder.getDistance());

        // System.out.println("Right rear encoder ticks: "
        // + Hardware.rightRearDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Right rear encoder ticks: "
        // + Hardware.rightRearDriveEncoder.get());

        // System.out.println("Lift encoder inches: "
        // + Hardware.liftingEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Lift encoder inches: "
        // + Hardware.liftingEncoder.getDistance());

        // System.out.println(
        // "Lift encoder ticks: " + Hardware.liftingEncoder.get());
        // Hardware.telemetry.printToConsole(
        // "Lift encoder ticks: " + Hardware.liftingEncoder.get());

        // ---------------------------------
        // Red Light/IR Sensors
        // prints the state of the sensor
        // ---------------------------------
        // System.out.println("Arm IR: " + Hardware.armIR.get());
        // Hardware.telemetry
        // .printToConsole("Arm IR: " + Hardware.armIR.get());
        // System.out
        // .println("Left back IR: " + Hardware.leftBackIR.get());
        // Hardware.telemetry.printToConsole("Left back IR: " +
        // Hardware.leftBackIR.get());

        // System.out.println(
        // "Right back IR: " + Hardware.rightBackIR.get());
        // Hardware.telemetry.printToConsole(
        // "Right back IR: " + Hardware.rightBackIR.get());
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

        // System.out.println("Arm intake solenoid forward: "
        // + Hardware.armIntakeSolenoid.getForward());
        // Hardware.telemetry
        // .printToConsole("Arm intake solenoid forward: "
        // + Hardware.armIntakeSolenoid.getForward());
        // System.out.println("Arm intake solenoid reverse: "
        // + Hardware.armIntakeSolenoid.getReverse());
        // Hardware.telemetry
        // .printToConsole("Arm intake solenoid reverse: "
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

        // System.out.println("Delay pot: " + Hardware.delayPot.get());
        // Hardware.telemetry.printToConsole("Delay pot: " +
        // Hardware.delayPot.get());
        // System.out.println("delay pot: " + Hardware.delayPot.get(0, 5));
        // Hardware.telemetry.printToConsole("delay pot: " +
        // Hardware.delayPot.get(0, 5));
        // System.out.println("Intake deploy sensor: "
        // + Hardware.intakeDeploySensor.get());
        // Hardware.telemetry.printToConsole("Intake deploy sensor: "
        // + Hardware.intakeDeploySensor.get());

        // ---------------------------------
        // Sonar/UltraSonic
        // ---------------------------------

        // SmartDashboard.putNumber("F ultrasonic: ",
        // Hardware.frontUltraSonic
        // .getDistanceFromNearestBumper());
        // System.out.println("ultrasonic " + Hardware.frontUltraSonic
        // .getDistanceFromNearestBumper());
        // Hardware.telemetry.printToConsole("ultrasonic " +
        // Hardware.frontUltraSonic
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
        // Hardware.telemetry.printToConsole("Gyro: " +
        // Hardware.gyro.getAngle());

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
} // end teleopDrive()



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

private static final double FIRST_GEAR_RATIO_KILROY_XIX = .4;

private static final double SECOND_GEAR_RATIO_KILROY_XIX = .7;

private static final double FIRST_GEAR_RATIO_KILROY_XX = .4;

private static final double SECOND_GEAR_RATIO_KILROY_XX = .5;

private static final double DEADBAND_VALUE_KILROY_XIX = .2;

private static final double DEADBAND_VALUE_KILROY_XX = .2;

private static final int TELEMETRY_PERIODICITY_KILROY_XIX = 1000;

private static final int TELEMETRY_PERIODICITY_KILROY_XX = 1000;

final static int CYCLE_BACKGROUND_COLOR = 4;

private static enum CurrentBackground
    {
    CLEAR, BLUE, ORANGE
    }

public static CurrentBackground backgroundColor = CurrentBackground.CLEAR;

private static boolean isBlue = true;

private static boolean isOrange = true;

private static boolean isCurrentlyChanging = false;

// ================================
// Variables
// ================================


} // end class
