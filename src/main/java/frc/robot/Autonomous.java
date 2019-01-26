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
import frc.HardwareInterfaces.LightSensor;
import frc.Utils.*;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import frc.Utils.drive.Drive;
import frc.HardwareInterfaces.KilroySPIGyro;

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
INIT, DELAY, CHOOSE_PATH, CROSS_AUTOLINE, DEPOSIT_STRAIGHT_CARGO_HATCH, DEPOSIT_ROCKET_HATCH, DEPOSIT_SIDE_CARGO_HATCH, FINISH
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
    if (Hardware.rightDriver.getRawButton(11)
            && Hardware.leftDriver.getRawButton(11))
        {
        autoState = State.FINISH;
        }

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
            System.out.println("CATS ARE AWESOME, BUT DOGS ARE BETTER");
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

        case DEPOSIT_STRAIGHT_CARGO_HATCH:
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
            autoState = State.DEPOSIT_STRAIGHT_CARGO_HATCH;
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


private static enum DepositCargoHatchState
    {
INIT, DESCEND, ALIGN_TO_CARGO, DEPOSIT_CARGO
    }

private static DepositCargoHatchState depositCargoHatchState = DepositCargoHatchState.INIT;

private static boolean depositCargoHatch ()
{
    switch (depositCargoHatchState)
        {
        case INIT:
            if (autoLevel == Level.LEVEL_TWO)
                {
                depositCargoHatchState = DepositCargoHatchState.DESCEND;
                } else
                {

                }
            break;
        case DESCEND:
            if (descendFromLevelTwo())
                {
                if (usingVision)
                    {

                    } else
                    {

                    }
                }
            break;

        case ALIGN_TO_CARGO:
            break;
        case DEPOSIT_CARGO:
            break;

        }




    if (autoLevel == Level.LEVEL_TWO)
        {
        descendFromLevelTwo();
        }

    return false;
}



private static enum RocketHatchState
    {
STANDBY, DESCEND, STRAIGHTEN_OUT_ON_WALL, DRIVE_FORWARD_TO_TURN, TURN_TOWARDS_FIELD_WALL, DRIVE_TOWARDS_FIELD_WALL, TURN_ALONG_FIELD_WALL, ALIGN_PERPENDICULAR_TO_TAPE, DRIVE_TO_ROCKET_TAPE, ALIGN_TO_ROCKET, DEPOSIT_HATCH, FINISH, DRIVE_BY_CAMERA
    }

private static RocketHatchState rocketHatchState = RocketHatchState.STANDBY;

private static boolean depositRocketHatch ()
{

    if (rocketHatchState == RocketHatchState.STANDBY)
        {
        rocketHatchState = RocketHatchState.DESCEND;
        }
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
                autoTimer.reset();
                autoTimer.start();
                Hardware.drive.drive(DRIVE_AGAINST_WALL_SPEED,
                        DRIVE_AGAINST_WALL_SPEED);
                rocketHatchState = RocketHatchState.STRAIGHTEN_OUT_ON_WALL;
                }
            break;



        // =================================================================
        // DRIVE BY NONVISION this is where the smart kids code
        // =================================================================
        case STRAIGHTEN_OUT_ON_WALL:

            if (autoTimer.get() >= TIME_TO_STRAIGHTEN_OUT_ON_WALL)
                {
                Hardware.drive.stop();
                autoTimer.stop();
                rocketHatchState = RocketHatchState.DRIVE_FORWARD_TO_TURN;
                }
            break;

        case DRIVE_FORWARD_TO_TURN:
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_DRIVE_TO_FIRST_TURN, DRIVE_SPEED,
                    ACCELERATION_TIME, true) == true)
                {
                rocketHatchState = RocketHatchState.TURN_TOWARDS_FIELD_WALL;
                }
            break;

        case TURN_TOWARDS_FIELD_WALL:
            // turn for if we are on the right side of the field
            if (autoPosition == Position.RIGHT && Hardware.drive
                    .turnDegrees(TURN_RIGHT90, TURN_SPEED,
                            ACCELERATION_TIME, true))
                {
                rocketHatchState = RocketHatchState.DRIVE_TOWARDS_FIELD_WALL;
                }
            // turn for if we are on the left side of th field
            else if (autoPosition == Position.LEFT
                    && Hardware.drive
                            .turnDegrees(TURN_LEFT90, TURN_SPEED,
                                    ACCELERATION_TIME, true))
                {
                rocketHatchState = RocketHatchState.DRIVE_TOWARDS_FIELD_WALL;
                }
            break;
        case DRIVE_TOWARDS_FIELD_WALL:
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() >= DISTANCE_NEEDED_TO_TURN)
                {
                Hardware.drive.driveStraight(DRIVE_SPEED,
                        ACCELERATION_TIME, false);
                } else
                {
                Hardware.drive.stop();
                rocketHatchState = RocketHatchState.TURN_ALONG_FIELD_WALL;
                }

            break;
        case TURN_ALONG_FIELD_WALL:
            // turn for if we are on the right side of the field
            if (autoPosition == Position.RIGHT && Hardware.drive
                    .turnDegrees(TURN_LEFT90, TURN_SPEED,
                            ACCELERATION_TIME, false))
                {
                rocketHatchState = RocketHatchState.ALIGN_PERPENDICULAR_TO_TAPE;
                }
            // turn for if we are on the left side of th field
            else if (autoPosition == Position.LEFT
                    && Hardware.drive
                            .turnDegrees(TURN_RIGHT90, TURN_SPEED,
                                    ACCELERATION_TIME, false))
                {
                rocketHatchState = RocketHatchState.ALIGN_PERPENDICULAR_TO_TAPE;
                }
            break;

        case ALIGN_PERPENDICULAR_TO_TAPE:
            // if (alignPerpemdicularToTape() == true)
            // {
            // rocketHatchState = RocketHatchState.DRIVE_TO_TAPE;
            // }

            break;

        case DRIVE_TO_ROCKET_TAPE:
        // if (redlight1 == true || redlight2 == true || redlight3 == true ||
        // redlight4 == true ||redlight5 == true)
            {
            rocketHatchState = RocketHatchState.ALIGN_TO_ROCKET;
            }
        // else
            {
            Hardware.drive.driveStraight(DRIVE_SPEED, ACCELERATION_TIME,
                    false);
            }

            break;

        // =================================================================
        // DRIVE BY VISION CODE this is where the cool kidz code
        // =================================================================

        case DRIVE_BY_CAMERA:
            if (Hardware.axisCamera.hasBlobs())
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
        // if (alignParallelToTape() == true)
            {
            rocketHatchState = RocketHatchState.DEPOSIT_HATCH;
            }
            break;
        case DEPOSIT_HATCH:
        // if (GamePieceManipulator.depositRocketHatch() == true)
            {
            rocketHatchState = RocketHatchState.FINISH;
            }
            break;
        case FINISH:
            return true;
        default:
            break;
        }

    return false;
}

/** Enum for representing the states used in the depositSideCargoHatch path */
private static enum SideCargoHatchState
    {
INIT, LEAVE_LEVEL_2, TURN_AFTER_LEVEL_2_DROP, LEAVE_LEVEL_1_ONLY, DRIVE_1, TURN_1, DRIVE_2, TURN_2, DRIVE_TO_TAPE, DRIVE_AFTER_TAPE, TURN_AFTER_TAPE, DRIVE_TO_CARGO_SHIP, SCORE, FINISHED
    } // and we need to deploy the manipulator somewhere in here

/**
 * Variable for keeping track of the state used in the depositSideCargoHatch
 * path
 */
private static SideCargoHatchState sideCargoHatchState = SideCargoHatchState.LEAVE_LEVEL_2;

private static boolean depositSideCargoHatch ()
{
    switch (sideCargoHatchState)
        {
        case INIT:
            if (autoLevel == Level.LEVEL_TWO)
                sideCargoHatchState = SideCargoHatchState.LEAVE_LEVEL_2;
            else
                sideCargoHatchState = SideCargoHatchState.LEAVE_LEVEL_1_ONLY;
            break;
        case LEAVE_LEVEL_2:
            if (descendFromLevelTwo() == true)
                {
                sideCargoHatchState = SideCargoHatchState.TURN_AFTER_LEVEL_2_DROP;
                }
            break;
        case TURN_AFTER_LEVEL_2_DROP:

            break;
        case LEAVE_LEVEL_1_ONLY:
            if (driveOffStraightLevel1() == true)
                {
                sideCargoHatchState = SideCargoHatchState.DRIVE_1;
                }
            break;
        case DRIVE_1:
            break;
        case TURN_1:
            break;
        case DRIVE_2:
            break;
        case TURN_2:
            break;
        case DRIVE_TO_TAPE:
            break;
        case DRIVE_AFTER_TAPE:
            break;
        case TURN_AFTER_TAPE:
            break;
        case DRIVE_TO_CARGO_SHIP:
            break;
        case SCORE:
            break;
        default:
        case FINISHED:

            break;
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

private static enum DescentState
    {
STANDBY, INIT, DRIVE_FAST, LANDING_SETUP, FINISH
    }

private static DescentState descentState = DescentState.STANDBY;

// TODO placeholder
public static boolean reorientAfterLevel2Drop()
{

    return false;
}
public static boolean driveOffStraightLevel1 ()
{
    return driveOffStraightLevel1(Hardware.leftBackIR,
            Hardware.rightBackIR, Hardware.drive);
}

// TODO this needs to be tested
public static boolean driveOffStraightLevel1 (LightSensor backIR1,
        LightSensor backIR2, Drive drive)
{
    double driveSpeed = .7; // arbitrary number to be tested
    // for now, we are not using the gyro
    boolean usingGyro = USING_GYRO_FOR_DRIVE_STARIGHT;

    if (backIR1.isOn() == true || backIR2.isOn() == true)
        {
        drive.driveStraight(driveSpeed, ACCELERATION_TIME,
                usingGyro);
        return false;
        } else
        {
        drive.stop();
        return true;
        }
}




public static boolean descendFromLevelTwo ()
{

    if (descentState == DescentState.STANDBY)
        {
        descentState = DescentState.INIT;
        }
    // if (descendInit == false)
    // {
    // descentTimer.start();
    // descendInit = true;
    // }

    // if (descentTimer.get() <= TIME_TO_DRIVE_OFF_PLATFORM)
    // {
    // Hardware.drive.driveStraight(DRIVE_SPEED, ACCELERATION_TIME,
    // false);
    // } else
    // {
    // Hardware.drive.stop();
    // descentTimer.stop();
    // descentTimer.reset();
    // descendInit = false;
    // return true;
    // }

    // return false;
    System.out.println(descentState);

    switch (descentState)
        {

        case STANDBY:

            break;
        case INIT:
            descentTimer.reset();
            descentTimer.start();
            descentState = DescentState.DRIVE_FAST;
            break;

        case DRIVE_FAST:
            if (descentTimer.get() >= TIME_TO_DRIVE_OFF_PLATFORM)
                {
                Hardware.drive.stop();
                descentTimer.stop();
                descentState = DescentState.LANDING_SETUP;
                } else
                {
                Hardware.drive.driveStraight(1.0, ACCELERATION_TIME,
                        false);
                }
            break;

        case LANDING_SETUP:

            if (Hardware.testRedLight.isOn() == true)
                {
                descentState = DescentState.FINISH;
                }

            break;

        case FINISH:
            System.out.println(
                    "YEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEETTTTTTTTTTTTTTTTTTTT");
            return true;

        default:
            break;

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

public static final double DRIVE_AGAINST_WALL_SPEED = -.6;

public static final double TIME_TO_DRIVE_OFF_PLATFORM = 2.0;

public static final double TIME_TO_STRAIGHTEN_OUT_ON_WALL = .6;

// whether or not, by default, we are using the gyro for driveStraight
// in our autonomous code
public static final boolean USING_GYRO_FOR_DRIVE_STARIGHT = false;

/**
 * Acceleration time that we generally pass into the drive class's driveStraight
 * function; .6 is the value we used for 2018's robot
 */
public static final double ACCELERATION_TIME = .6;

public static final double DRIVE_WITH_CAMERA_SPEED = .38;// TODO

public static final double DISTANCE_TO_DRIVE_TO_FIRST_TURN = 23;

public static final int DISTANCE_NEEDED_TO_TURN = 6;

public static final int TURN_RIGHT90 = 90;

public static final int TURN_LEFT90 = -90;

public static final double TURN_SPEED = .4;

public static Timer autoTimer = new Timer();

} // end class
