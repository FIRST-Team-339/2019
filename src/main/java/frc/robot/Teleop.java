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
import edu.wpi.first.wpilibj.Relay.Value;
// import com.sun.org.apache.xerces.internal.impl.xpath.XPath.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Utils.ClimbToLevelTwo;
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
    Hardware.axisCamera.setRelayValue(Value.kOn);
    Hardware.telopTimer.start();
    switch (Hardware.whichRobot)
        {
        case KILROY_2018:
            DRIVE_SPEED = .4;
            TURN_SPEED = .4;
            initTeleop2018();
            break;

        default:
        case KILROY_2019:
            DRIVE_SPEED = .4;
            TURN_SPEED = .7;

            // TIME_TO_DELAY_AFTER_DRIVE_FAST = 1;
            // TIME_TO_DELAY_B4_FINISH = 4;
            // TURN_180 = 180;
            // DRIVE_AGAINST_WALL_SPEED = -.6;
            // DRIVE_BACKWARDS_SPEED = -.4;
            // SPEED_TO_DRIVE_OFF_PLATFORM = .75; // @ANE
            // REVERSE_SPEED_TO_DRIVE_OFF_PLATFORM = -.85;
            // TIME_TO_DRIVE_OFF_PLATFORM = .4; // @ANE
            // TIME_TO_STRAIGHTEN_OUT_ON_WALL = .6;
            // TIME_TO_DRIVE_BACKWARDS_TO_ALIGN = .35;
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

    Hardware.lift.resetStateMachine();
    Hardware.manipulator.resetStateMachine();

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
    // -------------------------------------
    // Resets encoder values
    // -------------------------------------
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    // DO NOT RESET THE LIFT ENCODER

    // ---------------------------------
    // setup motors
    // ---------------------------------
    // Hardware.rightDriveMotor.set(0);
    // Hardware.leftDriveMotor.set(0);

    Hardware.lift.resetStateMachine();
    Hardware.manipulator.resetStateMachine();
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

    // // Forklift
    Hardware.lift.moveForkliftWithController(Hardware.rightOperator,
            Hardware.forkliftOverride.get());

    Hardware.lift.setLiftPositionByButton(Forklift.CARGO_SHIP_CARGO,
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED,
            Hardware.cargoShipCargoButton);

    Hardware.lift.setLiftPositionByButton(Forklift.CARGO_SHIP_HATCH,
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED,
            Hardware.cargoShipHatchButton);

    Hardware.lift.setToNextHigherPreset(
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED,
            Hardware.nextHigherLiftHeightButton,
            Hardware.chooseCargoRocketHeights.get());

    Hardware.lift.setToNextLowerPreset(
            Forklift.DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED,
            Hardware.nextLowerLiftHeightButton,
            Hardware.chooseCargoRocketHeights.get());

    // Game Piece Manipulator

    Hardware.manipulator.moveArmByJoystick(Hardware.leftOperator,
            Hardware.deployOverride.get());

    Hardware.manipulator.intakeOuttakeByButtonsSeperated(
            Hardware.intakeTrigger.get(),
            Hardware.outtakeButton.get(),
            Hardware.intakeOverride.get());

    // // =================================================================
    Hardware.lift.update();

    Hardware.manipulator.masterUpdate();

    Hardware.lift.printDebugInfo();
    Hardware.manipulator.printDeployDebugInfo();

    Hardware.climber.climbUpdate();

    Hardware.depositGamePiece.depositTeleopStateMachine();


    // vision=====================================


    if (Hardware.visionHeightUpButton.get() == true
            && visionHeight < 3 && Hardware.telopTimer.get() > .25)
        {
        Hardware.telopTimer.reset();
        visionHeight++;
        Hardware.telopTimer.start();
        }
    if (Hardware.visionHeightDownButton.get() == true
            && visionHeight > 0 && Hardware.telopTimer.get() > .25)
        {
        Hardware.telopTimer.reset();
        visionHeight--;
        Hardware.telopTimer.start();
        }
    if (Hardware.alignVisionButton.isOnCheckNow() == true
            && Hardware.depositGamePiece.overrideVision() == false)
        {

        if (Hardware.depositGamePiece
                .startTeleopDeposit(visionHeight,
                        false/* Hardware.manipulator.hasCargo() */))
            {
            hasFinishedDeposit = true;
            Hardware.depositGamePiece.resetDepositTeleop();
            }
        }
    else
        {
        hasFinishedDeposit = false;
        Hardware.depositGamePiece.resetDepositTeleop();
        }

    // end vision==============================================

    // buttons

    // buttons to cancel everything ===========================
    if (Hardware.cancelTwoButton.get() == true
            && Hardware.cancelOneButton.get() == true)
        {
        Hardware.climber.finishEarly();
        Autonomous.endAutoPath();
        Hardware.lift.resetStateMachine();
        Hardware.manipulator.resetStateMachine();
        } // end if


    // Buttons to reset the forklift encoder. Should never be called during
    // a match; only is in the final code for the purpsoe of speeding up
    // testing in the pits

    if (Hardware.resetForkliftEncoderButton1.get() == true
            && Hardware.resetForkliftEncoderButton2.get() == true)
        {
        Hardware.lift.resetEncoder();
        }

    individualTest();

    takePicture();



    // Hardware.telemetry.printToShuffleboard();

    // Hardware.telemetry.printToConsole();

    // if (Hardware.climbOneButton.isOnCheckNow() == true
    // && Hardware.climbTwoButton.isOnCheckNow() == true)
    if (Hardware.leftDriver.getRawButton(6) == true)
        {
        Hardware.climber.climb();
        }

    if (Hardware.alignVisionButton.get() == false
            || Hardware.depositGamePiece.overrideVision())
        {
        if (ClimbToLevelTwo.climbState == ClimbToLevelTwo.ClimberState.STANDBY)
            {
            teleopDrive();
            if (Hardware.solenoidButtonOne.isOnCheckNow() == true
                    && Hardware.solenoidButtonTwo
                            .isOnCheckNow() == true)
                {
                Hardware.driveSolenoid.setForward(false);
                }
            else
                {
                Hardware.driveSolenoid.setForward(true);
                }
            }
        }



    printStatements();

    Hardware.lift.printDebugInfo();
    Hardware.manipulator.printDeployDebugInfo();
} // end Periodic()


// Individual testing methods for each programmer. Each programmer should //put
// their testing code inside their own method.
// Author: Guido Visioni

private static void individualTest ()
{
    // ashleyTest();
    connerTest();
    // coleTest();
    // guidoTest();
    // patrickTest();
    // annaTest();
    // meghanTest();
    // dionTest();
    // nithyaTest();
} // end individualTest()

private static void ashleyTest ()
{

    if (Hardware.armHackButton.isOnCheckNow() == true)
        {
        // Hardware.manipulator.setArmMotorSpeedManuallyForClimb(-.0);
        Hardware.armMotor.set(-.6);
        }
    else
        {
        Hardware.armMotor.set(0.0);
        }

    if (Hardware.liftHackButton.get() == true)
        {
        Hardware.lift.setLiftPosition(0, 3);
        }

    // Hardware.climber.reverseClimbUpdate();
    // if (Hardware.leftDriver.getRawButton(6) == true)
    // {
    // Hardware.climber.reverseClimb();
    // }


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

private static void connerTest ()
{

    System.out.println("usingVision: "
            + Hardware.alignVisionButton.isOnCheckNow());

    System.out.println("level "
            + visionHeight);

    System.out.println("ultrasonic: "
            + Hardware.frontUltraSonic.getDistanceFromNearestBumper());


} // end connerTest()

private static void coleTest ()
{
    // TODO retest forklift with the new way the scaling factor works
    // (applies even during override), and well as how manipulator
    // should now have scaling factor apploied to override as well
    // Then deployArm/ retractArm/ setDeploy45DegreeButton

    if (Hardware.testDeployButtonTemp.getCurrentValue())
        Hardware.manipulator.deployArm();

    if (Hardware.testRetractTemp.getCurrentValue())
        Hardware.manipulator.retractArm();

    if (Hardware.testSetManipulatorPosition.getCurrentValue())
        Hardware.manipulator.moveArmToPosition(45, 1.0);



    // Manipulator



} // end coleTest()


private static void guidoTest ()
{
    if (Hardware.leftOperator.getRawButton(3) == true)
        {
        Hardware.armIntakeSolenoid.setForward(true);
        }
    else
        {
        Hardware.armIntakeSolenoid.setReverse(true);
        }
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


    if (Hardware.lift.getForkliftHeight() % FORKLIFT_DIVISOR >= 0.0
            && Hardware.lift.getForkliftHeight()
                    % FORKLIFT_DIVISOR <= 0.2
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
    if (Hardware.lift.getForkliftHeight() % FORKLIFT_DIVISOR != 0)
        {
        isCurrentlyChanging = false;
        }

    // if (Hardware.rightDriver.getRawButton(4) == false)
    // {
    // isCurrentlyChanging = false;
    // }
} // end patrickTest()

private static void annaTest ()
{

} // end annaTest()

private static void meghanTest ()
{

} // end meghanTest()

private static void dionTest ()
{

} // end dionTest()

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
        // SmartDashboard.putNumber("Arm motor: ",
        // Hardware.armMotor.get());
        // Hardware.telemetry.printToConsole(
        // "Arm motor: " + Hardware.armMotor.get());

        // System.out.println("Lift Motor One "
        // + Hardware.liftMotor.get());
        // SmartDashboard.putNumber("Lift Motor One ",
        // Hardware.liftMotor.get());
        // Hardware.telemetry.printToConsole("Lift Motor One "
        // + Hardware.liftMotor.get());

        // System.out.println("RF Drive Motor " +
        // Hardware.rightFrontCANMotor.get());
        // SmartDashboard.putNumber("RF Drive Motor ",
        // Hardware.rightFrontCANMotor.get());
        // Hardware.telemetry.printToConsole("RF Drive Motor " +
        // Hardware.rightFrontCANMotor.get());

        // System.out.println("LF Drive Motor "
        // + Hardware.leftFrontCANMotor.get());
        // SmartDashboard.putNumber("LF Drive Motor ",
        // Hardware.leftFrontCANMotor.get());
        // Hardware.telemetry.printToConsole("LF Drive Motor "
        // + Hardware.leftFrontCANMotor.get());

        // System.out.println("RR Drive Motor " +
        // Hardware.rightRearCANMotor.get());
        // SmartDashboard.putNumber("RR Drive Motor ",
        // Hardware.rightRearCANMotor.get());
        // Hardware.telemetry.printToConsole("RR Drive Motor " +
        // Hardware.rightRearCANMotor.get());

        // System.out.println("LR Drive Motor "
        // + Hardware.leftRearCANMotor.get());
        // SmartDashboard.putNumber("LR Drive Motor ",
        // Hardware.leftRearCANMotor.get());
        // Hardware.telemetry.printToConsole("LR Drive Motor "
        // + Hardware.leftRearCANMotor.get());

        // System.out.println("Arm Roller "
        // + Hardware.armRoller.get());
        // SmartDashboard.putNumber("Arm Roller ",
        // Hardware.armRoller.get());
        // Hardware.telemetry.printToConsole("Arm Roller "
        // + Hardware.armRoller.get());

        // =================================
        // Relay
        // =================================
        // System.out.println(
        // "Ring light relay: " + Hardware.ringLightRelay.get());
        // SmartDashboard.putString(
        // "Ring light relay: ",
        // "" + Hardware.ringLightRelay.get());
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
        // SmartDashboard.putBoolean(
        // "Left auto switch: ", Hardware.leftAutoSwitch.isOn());
        // Hardware.telemetry.printToConsole(
        // "Left auto switch: " + Hardware.leftAutoSwitch.isOn());

        // System.out.println(
        // "Right auto switch: "
        // + Hardware.rightAutoSwitch.isOn());
        // SmartDashboard.putString(
        // "Right auto switch: ",
        // "" + Hardware.rightAutoSwitch.isOn());
        // Hardware.telemetry.printToConsole(
        // "Right auto switch: "
        // + Hardware.rightAutoSwitch.isOn());

        // System.out.println("Center auto switch: "
        // + Hardware.autoCenterSwitch.isOn());
        // SmartDashboard.putString("Center auto switch: ",
        // "" + Hardware.autoCenterSwitch.isOn());
        // Hardware.telemetry.printToConsole("Center auto switch: "
        // + Hardware.autoCenterSwitch.isOn());

        // System.out.println(
        // "Level one switch: " + Hardware.levelOneSwitch.isOn());
        // SmartDashboard.putString(
        // "Level one switch: ",
        // "" + Hardware.levelOneSwitch.isOn());
        // Hardware.telemetry.printToConsole(
        // "Level one switch: " + Hardware.levelOneSwitch.isOn());

        // System.out.println(
        // "Level two switch: " + Hardware.levelTwoSwitch.isOn());
        // // SmartDashboard.putString(
        // "Level two switch: ",
        // "" + Hardware.levelTwoSwitch.isOn());
        // Hardware.telemetry.printToConsole(
        // "Level two switch: " + Hardware.levelTwoSwitch.isOn());

        // System.out.println("Auto disable switch: "
        // + Hardware.autoDisableSwitch.isOn());
        // SmartDashboard.putString("Auto disable switch: ",
        // "" + Hardware.autoDisableSwitch.isOn());
        // Hardware.telemetry.printToConsole("Auto disable switch: "
        // + Hardware.autoDisableSwitch.isOn());

        // System.out.println("Auto 6 position switch: "
        // + Hardware.autoSixPosSwitch.getPosition());
        // SmartDashboard.putNumber("Auto 6 position switch: ",
        // Hardware.autoSixPosSwitch.getPosition());
        // Hardware.telemetry.printToConsole("Auto 6 position switch: "
        // + Hardware.autoSixPosSwitch.getPosition());

        // ---------------------------------
        // System.out.println("Disable SW " +
        // Hardware.autoDisableSwitch.isOn());
        // SmartDashboard.putBoolean("Disable SW ",
        // Hardware.autoDisableSwitch.isOn());
        // Hardware.telemetry.printToConsole("Disable SW " +
        // Hardware.autoDisableSwitch.isOn());

        // ---------------------------------
        // Encoders
        // ---------------------------------
        // System.out.println("LF encoder Inches = "
        // + Hardware.leftFrontDriveEncoder.getDistance());
        // SmartDashboard.putNumber("Left Front Encoder Inches = ",
        // Hardware.leftFrontDriveEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Left Front Encoder Inches = "
        // + Hardware.leftFrontDriveEncoder.getDistance());

        // System.out.println("LF encoder ticks: "
        // + Hardware.leftFrontDriveEncoder.get());
        // SmartDashboard.putNumber("Left front encoder ticks: ",
        // Hardware.leftFrontDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Left front encoder ticks: "
        // + Hardware.leftFrontDriveEncoder.get());

        // System.out.println("Right Front Inches = "
        // + Hardware.rightFrontDriveEncoder.getDistance());
        // SmartDashboard.putNumber("Right Front Inches = ",
        // Hardware.rightFrontDriveEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Right Front Inches = "
        // + Hardware.rightFrontDriveEncoder.getDistance());

        // System.out.println("Right Front Ticks "
        // + Hardware.rightFrontDriveEncoder.get());
        // SmartDashboard.putNumber("Right Front Ticks ",
        // Hardware.rightFrontDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Right Front Ticks "
        // + Hardware.rightFrontDriveEncoder.get());

        // System.out.println("Left rear encoder inches: "
        // + Hardware.leftRearDriveEncoder.getDistance());
        // SmartDashboard.putNumber("Left rear encoder inches: ",
        // Hardware.leftRearDriveEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Left rear encoder inches: "
        // + Hardware.leftRearDriveEncoder.getDistance());

        // System.out.println("Left rear encoder ticks: "
        // + Hardware.leftRearDriveEncoder.get());
        // SmartDashboard.putNumber("Left rear encoder ticks: ",
        // Hardware.leftRearDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Left rear encoder ticks: "
        // + Hardware.leftRearDriveEncoder.get());

        // System.out.println("Right rear encoder distance: "
        // + Hardware.rightRearDriveEncoder.getDistance());
        // SmartDashboard.putNumber("Right rear encoder distance: ",
        // Hardware.rightRearDriveEncoder.getDistance());
        // Hardware.telemetry
        // .printToConsole("Right rear encoder distance: "
        // + Hardware.rightRearDriveEncoder.getDistance());

        // System.out.println("Right rear encoder ticks: "
        // + Hardware.rightRearDriveEncoder.get());
        // SmartDashboard.putNumber("Right rear encoder ticks: ",
        // Hardware.rightRearDriveEncoder.get());
        // Hardware.telemetry.printToConsole("Right rear encoder ticks: "
        // + Hardware.rightRearDriveEncoder.get());

        // System.out.println("Lift encoder inches: "
        // + Hardware.liftingEncoder.getDistance());
        // SmartDashboard.putNumber("Lift encoder inches: ",
        // Hardware.liftingEncoder.getDistance());
        // Hardware.telemetry.printToConsole("Lift encoder inches: "
        // + Hardware.liftingEncoder.getDistance());

        // System.out.println(
        // "Lift encoder ticks: " + Hardware.liftingEncoder.get());
        // SmartDashboard.putNumber(
        // "Lift encoder ticks: ", Hardware.liftingEncoder.get());
        // Hardware.telemetry.printToConsole(
        // "Lift encoder ticks: " + Hardware.liftingEncoder.get());

        // ---------------------------------
        // Red Light/IR Sensors
        // prints the state of the sensor
        // ---------------------------------
        // System.out.println("Arm IR: " + Hardware.armIR.get());
        // SmartDashboard.putBoolean("Arm IR: ", Hardware.armIR.get());
        // Hardware.telemetry
        // .printToConsole("Arm IR: " + Hardware.armIR.get());

        // System.out
        // .println("Left back IR: " + Hardware.leftBackIR.get());
        // SmartDashboard.putBoolean(
        // "Left back IR: ", Hardware.leftBackIR.get());
        // Hardware.telemetry.printToConsole("Left back IR: " +
        // Hardware.leftBackIR.get());

        // System.out.println(
        // "Right back IR: " + Hardware.rightBackIR.get());
        // SmartDashboard.putBoolean(
        // "Right back IR: ", Hardware.rightBackIR.get());
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
        // SmartDashboard.putBoolean("Arm intake solenoid forward: ",
        // Hardware.armIntakeSolenoid.getForward());
        // Hardware.telemetry
        // .printToConsole("Arm intake solenoid forward: "
        // + Hardware.armIntakeSolenoid.getForward());

        // System.out.println("Arm intake solenoid reverse: "
        // + Hardware.armIntakeSolenoid.getReverse());
        // SmartDashboard.putBoolean("Arm intake solenoid reverse: ",
        // Hardware.armIntakeSolenoid.getReverse());
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
        // SmartDashboard.putNumber("Delay pot: ",
        // Hardware.delayPot.get());
        // SmartDashboard.putNumber("Deploy pot min max: ",
        // Hardware.delayPot.get(0, 5));
        // Hardware.telemetry.printToConsole("Delay pot: " +
        // Hardware.delayPot.get());

        // System.out.println("delay pot: " + Hardware.delayPot.get(0, 5));
        // SmartDashboard.putNumber("delay pot: ",
        // Hardware.delayPot.get(0, 5));
        // Hardware.telemetry.printToConsole("delay pot: " +
        // Hardware.delayPot.get(0, 5));

        // System.out.println("Intake deploy sensor: "
        // + Hardware.intakeDeploySensor.get());
        // SmartDashboard.putNumber("Arm Pot sensor: ",
        // Hardware.armPot.get());
        // Hardware.telemetry.printToConsole("Intake deploy sensor: "
        // + Hardware.intakeDeploySensor.get());

        // ---------------------------------
        // Sonar/UltraSonic
        // ---------------------------------

        // System.out.println("ultrasonic " + Hardware.frontUltraSonic
        // .getDistanceFromNearestBumper());
        // SmartDashboard.putNumber("F ultrasonic: ",
        // Hardware.frontUltraSonic
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
        // SmartDashboard.putNumber("Gyro: ", Hardware.gyro.getAngle());
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

public static void takePicture ()
{
    // Takes a picture if buttons 8 and 9 are pressed on the right operator at
    // the same time
    if ((Hardware.pictureButtonOne.get() == true
            && Hardware.pictureButtonTwo.get() == true)
            || (pictureButton1 == true && pictureButton2 == true))
        {
        // Checks is this is the first time pressing the button or the button is
        // held down
        // Turns on ring light relay and resets and starts timer
        if (firstPress == true)
            {
            pictureButton1 = true;
            pictureButton2 = true;
            Hardware.takePictureTimer.reset();
            Hardware.ringLightRelay.set(Value.kOn);
            firstPress = false;
            Hardware.takePictureTimer.start();
            }
        // Takes a picture after 1 second of the ring light relay being on
        if (Hardware.takePictureTimer.get() >= 1.0
                && imageTaken == false)
            {

            Hardware.axisCamera.saveImage(ImageType.RAW);

            imageTaken = true;
            }
        // If three seconds have passed resets all variables used and turns off
        // ring light relay
        if (Hardware.takePictureTimer.get() >= 3.0)
            {
            // Hardware.ringLightRelay.set(Value.kOff);//TODO
            firstPress = true;
            pictureButton1 = false;
            pictureButton2 = false;
            }


        }

}

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
        } // end if
} // end teleopDrive()



// ================================
// Constants
// ================================

private static final int GEAR_UP_SHIFT_BUTTON = 3;

private static final int GEAR_DOWN_SHIFT_BUTTON = 3;

// The number of gears we want to not go over. There is no reason to make this
// more than 3 unless the code is fixed. Thanks McGee.
private static final int MAX_GEAR_NUMBERS = 2;

public static final int FIRST_GEAR_NUMBER = 0;

public static final int SECOND_GEAR_NUMBER = 1;

private static final double FIRST_GEAR_RATIO_KILROY_XIX = .4;

private static final double SECOND_GEAR_RATIO_KILROY_XIX = .7;

public static final double FIRST_GEAR_RATIO_KILROY_XX = .4;

public static final double SECOND_GEAR_RATIO_KILROY_XX = .5;


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

public static final double FORKLIFT_DIVISOR = 4;

private static double DRIVE_SPEED = .4;

private static double TURN_SPEED = .4;

// lower rocket by default
private static int visionHeight = 0;



// ================================
// Variables
// ================================
private static boolean firstPress = true;

private static boolean imageTaken = false;

private static boolean pictureButton1;

private static boolean pictureButton2;

public static boolean hasFinishedDeposit = false;

public static boolean solenoidInit = false;
} // end class
