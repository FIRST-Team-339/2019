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
// written. Some of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// -----------------------------------------------------
// Periodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.robot;

import frc.Hardware.Hardware;
import frc.Utils.*;
import javax.lang.model.util.ElementScanner6;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import frc.Utils.drive.Drive;

/**
 * An Autonomous class. This class <b>beautifully</b> uses state machines in
 * order to periodically execute instructions during the Autonomous period.
 *
 * This class contains all of the user code for the Autonomous part of the
 * match, namely, the Init and Periodic code
 *
 *
 * @author Michael Andrzej Klaczynski
 * @written at the eleventh stroke of midnight, the 28th of January, Year of our
 *          LORD 2016. Rewritten ever thereafter.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Autonomous
{

/**
 * User Initialization code for autonomous mode should go here. Will run once
 * when the autonomous first starts, and will be followed immediately by
 * periodic().
 */
public static void init ()
{
    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------
    // Hardware.leftDriveMotor.setSafetyEnabled(false);
    // Hardware.rightDriveMotor.setSafetyEnabled(false);

    // Hardware.leftFrontDriveEncoder.reset();
    // Hardware.rightFrontDriveEncoder.reset();

    // TODO @ANE uncomment
    // if (Hardware.autoLevelSwitch.isOn() == true) {
    // autoLevel = Level.DISABLE;
    // autoState = State.FINISH;
    // }

} // end Init

/**
 * State of autonomous as a whole; mainly for init, delay, finish, and choosing
 * which autonomous path is being used
 */
public static enum State
    {
INIT, DELAY, CHOOSE_PATH, CROSS_AUTOLINE, DEPOSIT_CARGO_HATCH, DEPOSIT_ROCKET_HATCH, DEPOSIT_SIDE_CARGO_HATCH, FINISH
    }

/**
 * Starting position and which side of the field the robot is going to
 */

public static enum Position
    {
LEFT, RIGHT, CENTER, NULL
    }

public static enum Level
    {
LEVEL_ONE, LEVEL_TWO, DISABLE, NULL
    }

// variable that controls the state of autonomous as a whole (init, delay
// which path is being used, etc.)
public static State autoState = State.INIT;

// variable that controls the starting position/side (Left, Right, or Center) of
// the robot

public static Position autoPosition = Position.NULL;

// variable that controls the level the robot is starting on (Level 1 or level 2
// or disabled)

public static Level autoLevel = Level.NULL;

/**
 * User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void periodic ()
{
    switch (autoState)
        {
        case INIT:
            setPositionAndLevel();
            autoState = State.DELAY;
            break;
        case DELAY:

            // Delay using the potentiometer, from 0 to 5 seconds
            // once finished, stop the timer and go to the next state

            // if (Hardware.autoTimer.get() >= Hardware.delayPot.get(0.0, 5.0))
            // {
            System.out.println("CATS ARE AWESOME");
            autoState = State.CHOOSE_PATH;
            Hardware.autoTimer.stop();
            // break;
            // }
            break;

        case CHOOSE_PATH:
            choosePath();
            break;

        case CROSS_AUTOLINE:
            if (crossAutoline() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case DEPOSIT_CARGO_HATCH:
            if (depositCargoHatch() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case DEPOSIT_ROCKET_HATCH:
            if (depositRocketHatch() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case DEPOSIT_SIDE_CARGO_HATCH:
            if (depositSideCargoHatch() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case FINISH:
            driverControl();
            break;

        default:
            autoState = State.FINISH;
            break;
        }

}

// ---------------------------------
// Methods
// ---------------------------------

/**
 *
 */
private static void choosePath ()
{
    switch (2/** Hardware.autoSixPosSwitch.getPosition() */
    )
        {
        case 0:
            autoState = State.CROSS_AUTOLINE;
            break;

        case 1:
            autoState = State.DEPOSIT_CARGO_HATCH;
            break;

        case 2:
            rocketHatchState = RocketHatchState.DESCEND;
            autoState = State.DEPOSIT_ROCKET_HATCH;
            break;

        case 3:
            autoState = State.DEPOSIT_SIDE_CARGO_HATCH;
            break;

        case 4:

        case 5:

        default:
            autoState = State.FINISH;
            break;
        }

}

/**
 * sets the enums of both the autoPosition and autoLevel based on the
 * coresponding switches
 *
 */
private static void setPositionAndLevel ()
{

    // sets the autoPosition enum to the correct side based on the
    // state of the autoPositionSwitch
    if (Hardware.autoPositionSwitch.getPosition() == LEFT)
        {
        autoPosition = Position.LEFT;
        } else if (Hardware.autoPositionSwitch.getPosition() == RIGHT)
        {
        autoPosition = Position.RIGHT;
        } else if (Hardware.autoPositionSwitch.isOn() == true)
        {
        autoPosition = Position.CENTER;
        }

    // sets the autoLevel enum to the correct level, or disabled, based on the
    // state
    // of the autoLevelSwitch
    if (Hardware.autoLevelSwitch.getPosition() == LEVEL_ONE)
        {
        autoLevel = Level.LEVEL_ONE;
        } else if (Hardware.autoLevelSwitch.getPosition() == LEVEL_TWO)
        {
        autoLevel = Level.LEVEL_TWO;
        }

}

// =====================================================================
// Path Methods
// =====================================================================

private static boolean crossAutoline ()
{
    if (autoLevel == Level.LEVEL_TWO)
        {
        descendFromLevelTwo();
        }
    if (Hardware.drive.driveStraightInches(DISTANCE_TO_CROSS_AUTOLINE,
            DRIVE_SPEED, ACCELERATION_TIME,
            false) == true)
        {
        return true;
        }
    return false;
}

private static boolean depositCargoHatch ()
{
    if (autoLevel == Level.LEVEL_TWO)
        {
        descendFromLevelTwo();
        }

    return false;
}


private static enum RocketHatchState
    {
STANDBY, DESCEND, STRAIGHTEN_OUT_ON_WALL, DRIVE_FORWARD_TO_TURN, TURN_TOWARDS_FIELD_WALL, DRIVE_TOWARDS_FIELD_WALL, TURN_ALONG_FIELD_WALL, DRIVE_TO_TAPE, ALIGN_TO_ROCKET, DEPOSIT_HATCH, FINISH, DRIVE_BY_CAMERA
    }

private static RocketHatchState rocketHatchState = RocketHatchState.STANDBY;

private static boolean depositRocketHatch ()
{

    switch (rocketHatchState)
        {
        case STANDBY:

            break;

        case DESCEND:
            if (autoLevel == Level.LEVEL_TWO)
                {
                descendFromLevelTwo();
                }
            if (usingVision == true && Hardware.axisCamera.hasBlobs())
                {
                rocketHatchState = RocketHatchState.DRIVE_BY_CAMERA;
                } else
                {
                rocketHatchState = RocketHatchState.STRAIGHTEN_OUT_ON_WALL;
                }
            break;



        // =================================================================
        // DRIVE BY NONVISION
        // =================================================================
        case STRAIGHTEN_OUT_ON_WALL:

            if (usingVision == true && Hardware.axisCamera.hasBlobs())
                {
                // if (dri)
                    {

                    }
                }
            break;

        case DRIVE_FORWARD_TO_TURN:

            break;

        case TURN_TOWARDS_FIELD_WALL:

            break;
        case DRIVE_TOWARDS_FIELD_WALL:

            break;
        case TURN_ALONG_FIELD_WALL:

            break;
        case DRIVE_TO_TAPE:

            break;

        // =================================================================
        // DRIVE BY VISION CODE this is where the cool kidz code
        // =================================================================

        case DRIVE_BY_CAMERA:
            if (Hardware.axisCamera.hasBlobs() && !Hardware.armIR.get())// TODO^^^^
                                                                        // IR
                {
                Hardware.driveWithCamera
                        .driveToTarget(DRIVE_WITH_CAMERA_SPEED);
                } else if (Hardware.armIR.get())
                {
                rocketHatchState = RocketHatchState.ALIGN_TO_ROCKET;
                }
            break;
        // =================================================================
        // END OF SEPCIALIZED DRIVING CODE
        // =================================================================
        case ALIGN_TO_ROCKET:

            break;
        case DEPOSIT_HATCH:

            break;
        case FINISH:
            return true;
        default:
            break;
        }

    return false;
}

private static boolean depositSideCargoHatch ()
{
    if (autoLevel == Level.LEVEL_TWO)
        {
        descendFromLevelTwo();
        }
    return false;
}

private static void driverControl ()
{
    // if (Hardware.leftDriver.getRawButton(5) == true) {
    // Hardware.leftFrontCANMotor.set(.5);
    // } else {
    // Hardware.leftFrontCANMotor.set(0);
    // }
    Teleop.periodic();
}

private static boolean descendFromLevelTwo ()
{
    if (descendInit == false)
        {
        descentTimer.start();
        descendInit = true;
        }

    if (descentTimer.get() <= TIME_TO_DRIVE_OFF_PLATFORM)
        {
        Hardware.drive.driveStraight(DRIVE_SPEED, ACCELERATION_TIME,
                false);
        } else
        {
        Hardware.drive.stop();
        descentTimer.stop();
        descentTimer.reset();
        descendInit = false;
        return true;
        }

    return false;
}

// =========================================================================
// TUNEABLES
// =========================================================================
private static boolean usingVision = false;

private static boolean descendInit = false;

public static Timer descentTimer = new Timer();

/*
 * =============================================================
 * Constants
 * =============================================================
 */



public static final Relay.Value LEFT = Relay.Value.kForward;

public static final Relay.Value RIGHT = Relay.Value.kReverse;

public static final Relay.Value LEVEL_ONE = Relay.Value.kForward;

public static final Relay.Value LEVEL_TWO = Relay.Value.kReverse;

public static final int DISTANCE_TO_CROSS_AUTOLINE = 25;

public static final double DRIVE_SPEED = .4;

public static final double TIME_TO_DRIVE_OFF_PLATFORM = 5.0;

public static final double ACCELERATION_TIME = .6;// not random number, pulled
// from 2018

public static final double DRIVE_WITH_CAMERA_SPEED = .4;// TODO

} // end class
