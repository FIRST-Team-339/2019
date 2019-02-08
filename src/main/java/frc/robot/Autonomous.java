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
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.DriveWithCamera.Side;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.Utils.drive.Drive;
import frc.Utils.drive.Drive.BrakeType;
import frc.vision.VisionProcessor.ImageType;


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
    Hardware.drive.setGearPercentage(1, 1.0);
    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------
    // Hardware.leftDriveMotor.setSafetyEnabled(false);
    // Hardware.rightDriveMotor.setSafetyEnabled(false);

    // Hardware.leftFrontDriveEncoder.reset();
    // Hardware.rightFrontDriveEncoder.reset();
    Hardware.gyro.reset();

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
INIT, DELAY, CHOOSE_PATH, CROSS_AUTOLINE, DEPOSIT_STRAIGHT_CARGO_HATCH, DEPOSIT_ROCKET_HATCH, DEPOSIT_SIDE_CARGO_HATCH, BLIND_ROCKET_HATCH, FINISH
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
 *
 *          FYI: drive.stop cuts power to the motors, causing the robot to
 *          coast. drive.brake results in a complete (? IDK, McGee rewrote much
 *          of the drive class.) stop
 *
 */
public static void periodic ()
{
    if (Hardware.rightDriver.getRawButton(11)
            && Hardware.leftDriver.getRawButton(11))
        {
        endAutoPath();
        autoState = State.FINISH;
        }

    switch (autoState)
        {
        case INIT:
            setPositionAndLevel();
            Hardware.autoTimer.start();
            autoState = State.DELAY;
            break;
        case DELAY:

            // Delay using the potentiometer, from 0 to 5 seconds
            // once finished, stop the timer and go to the next state
            if (Hardware.autoTimer.get() >= Hardware.delayPot.get(0.0,
                    5.0))
                {
                autoState = State.CHOOSE_PATH;
                Hardware.autoTimer.stop();
                }
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
            System.out.println("rocket hatch");
            usingVision = true;
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

        case BLIND_ROCKET_HATCH:
            usingVision = false;
            if (depositRocketHatch() == true)
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
    switch (Hardware.autoSixPosSwitch.getPosition())
        {
        case 0:
            autoState = State.CROSS_AUTOLINE;
            break;

        case 1:
            autoState = State.DEPOSIT_STRAIGHT_CARGO_HATCH;
            break;

        case 2:
            autoState = State.DEPOSIT_ROCKET_HATCH;
            break;

        case 3:
            autoState = State.DEPOSIT_SIDE_CARGO_HATCH;
            break;

        case 4:
            autoState = State.BLIND_ROCKET_HATCH;
            break;

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
    // autoLevel = Level.LEVEL_ONE;
    // (only needed if not testing the physical switch)
    // sets the autoPosition enum to the correct side based on the
    // state of the autoPositionSwitch

    if (Hardware.autoCenterSwitch.getPosition() == LEFT)
        {
        autoPosition = Position.LEFT;
        System.out.println("position and level set");
        } else if (Hardware.autoCenterSwitch.getPosition() == RIGHT)
        {
        autoPosition = Position.RIGHT;
        } else if (Hardware.autoCenterSwitch.isOn() == true)
        {
        autoPosition = Position.CENTER;
        }

    // sets the autoLevel enum to the correct level, or disabled, based on the
    // state of the autoDisableSwitch
    // if (Hardware.autoDisableSwitch.getPosition() == LEVEL_ONE)
    // {
    // System.out.println("Turning on GPS!");
    // crossAutoline();
    // } else if (Hardware.autoDisableSwitch
    // .getPosition() == LEVEL_TWO)
    // {
    // crossAutoline();
    // }

    // TEMP CODE FOR TEST PURPOSES ONLY

    // autoLevel = Level.LEVEL_ONE;
    // autoPosition = Position.RIGHT;
}

// =====================================================================
// Path Methods
// =====================================================================
public static enum Cross
    {
AWAKEN, L2_CLEAR_DATUM, NYOOM, SLAM_BRAKES, FINITE_INCANTATEM
    }

private static Cross cross = Cross.AWAKEN;

private static boolean crossAutoline ()
{
    switch (cross)
        {
        case AWAKEN:
            // initial state for crossing the autoline
            // setPositionAndLevel();
            switch (autoPosition)
                {
                case LEFT:
                    distanceToCrossAutoline = 60;
                    break;
                case CENTER:
                    distanceToCrossAutoline = 90;
                    break;
                case RIGHT:
                    distanceToCrossAutoline = 120;
                    break;
                case NULL:
                    break;
                }
            Hardware.leftFrontDriveEncoder.reset();
            Hardware.rightFrontDriveEncoder.reset();
            System.out.println("GOOD MORNING VIETNAM!");
            cross = Cross.L2_CLEAR_DATUM;
            break;

        case L2_CLEAR_DATUM:
            // only run if going off of level ii
            if (autoLevel == Level.LEVEL_TWO)
                {
                descendFromLevelTwo(usingAlignByWall);
                Hardware.leftFrontDriveEncoder.reset();
                Hardware.rightFrontDriveEncoder.reset();
                } else
                {
                System.out.println("Manoevering, clear the datum!");
                cross = Cross.NYOOM;
                }
            break;

        case NYOOM:
            if (Hardware.drive.driveStraightInches(
                    distanceToCrossAutoline,
                    DRIVE_SPEED, ACCELERATION_TIME, false) == true)
                {
                System.out.println("*distant screaming*");
                cross = Cross.SLAM_BRAKES;
                }
            break;

        case SLAM_BRAKES:
            Hardware.drive.brake(BrakeType.AFTER_DRIVE);
            System.out.println("SLAM THE BRAKES! SLAM THE BRAKES!");
            cross = Cross.FINITE_INCANTATEM;
            break;

        case FINITE_INCANTATEM:
            System.out.println(
                    "You have arrived at your final destination...the foreboding Vaaach homeworld.");
            break;
        }
    return false;
}

// non-state machine version
// private static boolean crossAutoline ()
// {
// switch (autoPosition)
// {
// case LEFT:
// distanceToCrossAutoline = 60;
// break;
// case CENTER:
// distanceToCrossAutoline = 90;
// break;
// case RIGHT:
// distanceToCrossAutoline = 120;
// break;
// }
// if (autoLevel == Level.LEVEL_ONE)
// {
// // this actually works! 2 February 2019
// if (Hardware.drive.driveStraightInches(
// distanceToCrossAutoline,
// DRIVE_SPEED, ACCELERATION_TIME,
// false) == true)
// {
// Hardware.drive.brake(BrakeType.AFTER_DRIVE);
// return true;
// }
// System.out.println(
// "IT WORKED TOO WELL! SLAM THE BRAKES!");
// }
// if (autoLevel == Level.LEVEL_TWO)
// {
// System.out.println("Oof!");
// descendFromLevelTwo(usingAlignByWall);
// }
// if (Hardware.drive.driveStraightInches(
// distanceToCrossAutoline,
// DRIVE_SPEED, ACCELERATION_TIME,
// false) == true)
// {
// Hardware.drive.brake(BrakeType.AFTER_DRIVE);
// return true;
// }
// return false;
// }


private static enum DepositCargoHatchState
    {
INIT, DESCEND, STRAIGHT_DEPOSIT_DRIVE_1, STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE, STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE, STRAIGHT_DEPOSIT_DRIVE_2, STRAIGHT_DEPOSIT_TURN_2_RIGHT_SIDE, STRAIGHT_DEPOSIT_TURN_2_LEFT_SIDE, STRAIGHT_DEPOSIT_DRIVE_3, STRAIGHT_DEPOSIT_ALIGN_TO_CARGO, STRAIGHT_DEPOSIT_DEPOSIT_CARGO, FINISHED
    }

private static DepositCargoHatchState depositCargoHatchState = DepositCargoHatchState.INIT;

private static boolean depositCargoHatch ()
{
    switch (depositCargoHatchState)
        {
        case INIT:
            // if on level two decend, turn base on start position
            if (autoLevel == Level.LEVEL_TWO)
                {
                depositCargoHatchState = DepositCargoHatchState.DESCEND;
                } else
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_1;
                }
            break;
        case DESCEND:
        // if (descendFromLevelTwo(usingAlignByWall))
            {
            // turn based on start position
            depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_1;
            }
            break;

        case STRAIGHT_DEPOSIT_DRIVE_1:
            if (Hardware.drive.driveStraightInches(
                    60, DRIVE_SPEED,
                    ACCELERATION_TIME,
                    USING_GYRO))
                {
                if (autoPosition == Position.RIGHT)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE;
                    } else
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE;
                    }
                }
            break;
        case STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE:
            if (Hardware.drive.turnDegrees(TURN_LEFT90,
                    TURN_BY_GYRO_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_2;
                }
            break;
        case STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE:
            if (Hardware.drive.turnDegrees(TURN_RIGHT90,
                    TURN_BY_GYRO_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_2;
                }
            break;
        case STRAIGHT_DEPOSIT_DRIVE_2:
            if (Hardware.drive.driveStraightInches(
                    DRIVE_STRAIGHT_DEPOSIT_1, DRIVE_SPEED,
                    ACCELERATION_TIME,
                    USING_GYRO))
                {
                if (autoPosition == Position.RIGHT)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_2_RIGHT_SIDE;
                    } else
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_2_LEFT_SIDE;
                    }
                }
            break;
        case STRAIGHT_DEPOSIT_TURN_2_RIGHT_SIDE:
            if (Hardware.drive.turnDegrees(TURN_RIGHT90,
                    TURN_BY_GYRO_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                if (usingVisionOnStraight == true)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;
                    } else
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_3;
                }
            break;
        case STRAIGHT_DEPOSIT_TURN_2_LEFT_SIDE:
            if (Hardware.drive.turnDegrees(-TURN_LEFT90,
                    TURN_BY_GYRO_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                if (usingVisionOnStraight == true)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;
                    } else
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_3;
                }
            break;
        case STRAIGHT_DEPOSIT_DRIVE_3:
            if (Hardware.drive.driveStraightInches(
                    DRIVE_STRAIGHT_DEPOSIT_2, DRIVE_SPEED,
                    ACCELERATION_TIME,
                    USING_GYRO)
                    || Hardware.frontUltraSonic
                            .getDistanceFromNearestBumper() < 20)
                {

                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DEPOSIT_CARGO;
                }

            break;
        case STRAIGHT_DEPOSIT_ALIGN_TO_CARGO:
            // maybe align with vision
            if (Hardware.driveWithCamera
                    .driveToTarget(DRIVE_WITH_CAMERA_SPEED))
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DEPOSIT_CARGO;
                }
            break;
        case STRAIGHT_DEPOSIT_DEPOSIT_CARGO:
            if (Hardware.manipulator.depositHatch())
                {
                return true;
                }
            break;
        case FINISHED:
            break;
        }
    return false;
}

private static enum RocketHatchState
    {
STANDBY, DESCEND, DRIVE_FORWARD_TO_TURN, TURN_TOWARDS_FIELD_WALL, DRIVE_TOWARDS_FIELD_WALL, DELAY_BEFORE_TURN_ALONG_FIELD_WALL, TURN_ALONG_FIELD_WALL, ALIGN_PERPENDICULAR_TO_TAPE, DRIVE_TO_ROCKET_TAPE, ALIGN_TO_ROCKET, PREP_TO_DEPOSIT_HATCH, DEPOSIT_HATCH, FINISH, DRIVE_BY_CAMERA
    }

private static RocketHatchState rocketHatchState = RocketHatchState.STANDBY;

// states for the camera nested switch statement
private static enum DriveWithCameraStates
    {
INIT, DRIVE, FIND_SIDE, TURN_RIGHT, TURN_LEFT, ALIGN
    }

private static DriveWithCameraStates driveWithCameraStates = DriveWithCameraStates.INIT;

private static boolean depositRocketHatch ()
{
    System.out.println(rocketHatchState);
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
                descendFromLevelTwo(usingAlignByWall);
                }
            if (usingVision == true)
                {
                rocketHatchState = RocketHatchState.DRIVE_BY_CAMERA;
                } else
                {
                // Hardware.axisCamera.setRelayValue(Value.kOff);
                autoTimer.reset();
                autoTimer.start();
                // Hardware.drive.drive(DRIVE_AGAINST_WALL_SPEED,
                // DRIVE_AGAINST_WALL_SPEED);
                rocketHatchState = RocketHatchState.DRIVE_FORWARD_TO_TURN;
                }
            break;
        // TODO @ANE
        // =================================================================
        // DRIVE BY NONVISION this is where the smart kids code
        // =================================================================
        case DRIVE_FORWARD_TO_TURN:
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_DRIVE_TO_FIRST_TURN_ROCKET, DRIVE_SPEED,
                    ACCELERATION_TIME,
                    true) == true)
                {
                rocketHatchState = RocketHatchState.TURN_TOWARDS_FIELD_WALL;
                }
            break;

        case TURN_TOWARDS_FIELD_WALL:
            // turn for if we are on the right side of the field
            if (autoPosition == Position.RIGHT
                    && Hardware.drive.turnDegrees(TURN_RIGHT90,
                            TURN_SPEED, ACCELERATION_TIME, true))
                {
                rocketHatchState = RocketHatchState.DRIVE_TOWARDS_FIELD_WALL;
                }
            // turn for if we are on the left side of th field
            else if (autoPosition == Position.LEFT
                    && Hardware.drive.turnDegrees(TURN_LEFT90,
                            TURN_SPEED, ACCELERATION_TIME, true))
                {
                rocketHatchState = RocketHatchState.DRIVE_TOWARDS_FIELD_WALL;
                }
            break;
        case DRIVE_TOWARDS_FIELD_WALL:
            System.out
                    .println("ultrasonic - " + Hardware.frontUltraSonic
                            .getDistanceFromNearestBumper());
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() > DISTANCE_NEEDED_TO_TURN)
                {
                Hardware.drive.driveStraight(DRIVE_SPEED,
                        ACCELERATION_TIME, false);
                } else
                {

                Hardware.drive.stop();
                Hardware.gyro.reset();
                autoTimer.reset();
                autoTimer.reset();
                rocketHatchState = RocketHatchState.TURN_ALONG_FIELD_WALL;
                }

            break;

        case DELAY_BEFORE_TURN_ALONG_FIELD_WALL:
            if (autoTimer.get() >= TIME_TO_DELAY_B4_TURN)
                {
                rocketHatchState = RocketHatchState.TURN_ALONG_FIELD_WALL;
                }
            break;


        case TURN_ALONG_FIELD_WALL:
            // turn for if we are on the right side of the field
            if (autoPosition == Position.RIGHT
                    && Hardware.drive.turnDegrees(TURN_LEFT90,
                            TURN_SPEED, ACCELERATION_TIME, true))
                {
                // currently bypasses the align state
                rocketHatchState = RocketHatchState.DRIVE_TO_ROCKET_TAPE;
                }
            // turn for if we are on the left side of th field
            else if (autoPosition == Position.LEFT
                    && Hardware.drive.turnDegrees(TURN_RIGHT90,
                            TURN_SPEED, ACCELERATION_TIME, true))
                {
                // currently bypasses the align state
                rocketHatchState = RocketHatchState.DRIVE_TO_ROCKET_TAPE;
                }
            break;

        case ALIGN_PERPENDICULAR_TO_TAPE:
            // if (alignPerpendicularToTape() == true)
            // {
            rocketHatchState = RocketHatchState.DRIVE_TO_ROCKET_TAPE;
            // }

            break;

        case DRIVE_TO_ROCKET_TAPE:
            // if (redlight1 == true || redlight2 == true || redlight3 == true
            // ||
            // redlight4 == true ||redlight5 == true)
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() < DISTANCE_NEEDED_TO_TURN)
                {
                Hardware.drive.stop();
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

            System.out.println("camera state" + driveWithCameraStates);
            System.out.println(
                    "ultrasonic distance: " + Hardware.frontUltraSonic
                            .getDistanceFromNearestBumper());
            System.out.println("encoder: "
                    + Hardware.rightFrontDriveEncoder.getDistance());
            switch (driveWithCameraStates)
                {
                case INIT:
                    System.out.println("the cool kidz code");

                    driveWithCameraStates = DriveWithCameraStates.DRIVE;
                    break;
                case DRIVE:
                    if (Hardware.drive.driveStraightInches(
                            distanceToCrossAutoline,
                            .6,
                            ACCELERATION_TIME, USING_GYRO))
                        {


                        if (autoPosition == Position.RIGHT)
                            {
                            driveWithCameraStates = DriveWithCameraStates.TURN_RIGHT;
                            } else if (autoPosition == Position.LEFT)
                            {
                            driveWithCameraStates = DriveWithCameraStates.TURN_LEFT;
                            } else
                            {
                            driveWithCameraStates = DriveWithCameraStates.FIND_SIDE;
                            }


                        }
                    break;
                case FIND_SIDE:
                    // System.out.println("find side: "
                    // + Hardware.driveWithCamera.getTargetSide());
                    if (Hardware.driveWithCamera
                            .getTargetSide() == Side.RIGHT)
                        {
                        driveWithCameraStates = DriveWithCameraStates.TURN_LEFT;
                        } else if (Hardware.driveWithCamera
                                .getTargetSide() == Side.LEFT)
                        {
                        driveWithCameraStates = DriveWithCameraStates.TURN_RIGHT;
                        } else
                        {
                        if (Hardware.drive.driveStraightInches(
                                DISTANCE_TO_CROSS_AUTOLINE_CAMERA, .4,
                                ACCELERATION_TIME, USING_GYRO))
                            rocketHatchState = RocketHatchState.FINISH;
                        }
                    break;
                case TURN_RIGHT:
                    System.out.println("right");

                    if (Hardware.drive.turnDegrees(
                            TURN_FOR_CAMERA_DEGREES, CAMERA_TURN_SPEED,
                            CAMERA_ACCELERATION, USING_GYRO))
                        {
                        driveWithCameraStates = DriveWithCameraStates.ALIGN;
                        }
                    break;
                case TURN_LEFT:
                    System.out.println(
                            "gyro degrees" + Hardware.gyro.getAngle());
                    System.out.println("left");
                    if (Hardware.drive.turnDegrees(
                            -TURN_FOR_CAMERA_DEGREES, CAMERA_TURN_SPEED,
                            CAMERA_ACCELERATION, USING_GYRO))
                        {
                        driveWithCameraStates = DriveWithCameraStates.ALIGN;
                        }
                    break;
                case ALIGN:
                    // Hardware.axisCamera.saveImage(ImageType.PROCESSED);
                    // Hardware.axisCamera.saveImage(ImageType.RAW);
                    // align with the camera
                    if (Hardware.driveWithCamera
                            .driveToTarget(DRIVE_WITH_CAMERA_SPEED))
                        {


                        rocketHatchState = RocketHatchState.DEPOSIT_HATCH;
                        }

                    break;
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

        case PREP_TO_DEPOSIT_HATCH:
            if (prepToDeposit() == true)
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
            System.out.println(
                    "YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO IT WORKED");
            return true;
        default:
            break;
        }

    return false;
}

/**
 * Enum for representing the states used in the depositSideCargoHatch path
 */
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
// welcome to the Gates of Hell
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
            if (descendFromLevelTwo(usingAlignByWall) == true)
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
    // if (Hardware.leftDriver.getRawButton(5) == true)
    // {
    // Hardware.leftFrontCANMotor.set(.5);
    // } else
    // {
    // Hardware.leftFrontCANMotor.set(0);
    // }
    Teleop.periodic();
}

public static enum DescentState
    {
STANDBY, INIT, DRIVE_FAST, LANDING_SETUP, BACKWARDS_TIMER_INIT, DRIVE_BACKWARDS_TO_ALIGN, FINISH
    }


// Cole wrote this method, January 2019



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
        drive.driveStraight(driveSpeed, ACCELERATION_TIME, usingGyro);
        return false;
        } else
        {
        drive.stop();
        return true;
        }
}


public static DescentState descentState = DescentState.STANDBY;

// TODO placeholder
public static boolean reorientAfterLevel2Drop ()
{

    return false;
}

public static boolean descendFromLevelTwo (boolean usingAlignByWall)
{

    if (descentState == DescentState.STANDBY)
        {
        descentState = DescentState.INIT;
        }


    System.out.println(descentState);
    System.out.println(descentTimer.get());
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
                // Hardware.drive.driveStraight(1.0, ACCELERATION_TIME,
                // false);
                Hardware.transmission.driveRaw(
                        SPEED_TO_DRIVE_OFF_PLATFORM,
                        SPEED_TO_DRIVE_OFF_PLATFORM);
                }
            break;

        case LANDING_SETUP:

            // if (Hardware.testRedLight.isOn() == true
            // && usingAlignByWall == true)
            // {
            // descentState = DescentState.BACKWARDS_TIMER_INIT;
            // } else if (Hardware.testRedLight.isOn()
            // && usingAlignByWall == false)
            // {
            // descentState = DescentState.FINISH;
            // }

            break;

        case BACKWARDS_TIMER_INIT:
            descentTimer.reset();
            descentTimer.start();
            descentState = DescentState.DRIVE_BACKWARDS_TO_ALIGN;
            break;

        case DRIVE_BACKWARDS_TO_ALIGN:
            if (descentTimer.get() >= TIME_TO_DRIVE_BACKWARDS_TO_ALIGN)
                {
                descentTimer.stop();
                Hardware.drive.stop();
                descentState = DescentState.FINISH;
                } else
                {
                Hardware.transmission.driveRaw(DRIVE_BACKWARDS_SPEED,
                        DRIVE_BACKWARDS_SPEED);
                }
            break;

        case FINISH:
            System.out.println(
                    "YEEt! XD");
            return true;

        default:
            break;

        }
    return false;
}


/**
 * function to back up and raise arm to deposit
 */
public static boolean prepToDeposit ()
{
    if (Hardware.drive.driveStraightInches(-6, .5, .6,
            USING_GYRO) == true)
        {
        // Hardware.manipulator.
        System.out.println(
                "NOW WE WOULD MOVE THE ARM IF THAT CODE WAS ACTUALLY WRITTEN");
        return true;
        }
    return false;
}


public static void endAutoPath ()
{
    sideCargoHatchState = SideCargoHatchState.FINISHED;
    depositCargoHatchState = DepositCargoHatchState.FINISHED;
    rocketHatchState = RocketHatchState.FINISH;
    descentState = DescentState.FINISH;
    cross = Cross.FINITE_INCANTATEM;

}


// =========================================================================
// TUNEABLES
// =========================================================================
// use vision for rocket autopath
private static boolean usingVision = true;

private static boolean usingAlignByWall = false;

// use vision for the put hatch straght auto path
private static boolean usingVisionOnStraight = true;

private static boolean descendInit = false;

public static Timer descentTimer = new Timer();





public static int distanceToCrossAutoline;
/*
 * ==============================================================
 * Constants
 * ==============================================================
 */


// General constants

// turn stuff

public static final double TURN_BY_GYRO_SPEED = .5;

public static final int TURN_RIGHT90 = 90;

public static final int TURN_LEFT90 = -90;

public static final double TURN_SPEED = .4;

// whether or not, by default, we are using the gyro for driveStraight
// in our autonomous code
public static final boolean USING_GYRO_FOR_DRIVE_STARIGHT = false;

// TODO test cross autoline at 50% speed in the near future;
// 2 February 2019
// public static final double DRIVE_SPEED = .4;

public static final boolean USING_GYRO = true;

public static Timer autoTimer = new Timer();

public static final double DRIVE_AGAINST_WALL_SPEED = -.6;

public static final double DRIVE_BACKWARDS_SPEED = -.5;

public static final double SPEED_TO_DRIVE_OFF_PLATFORM = .75;

public static final double DRIVE_SPEED = .4;



/**
 * Acceleration time that we generally pass into the drive class's driveStraight
 * function; .6 is the value we used for 2018's robot
 */

public static final double ACCELERATION_TIME = .6;

public static final Relay.Value LEFT = Relay.Value.kForward;

public static final Relay.Value RIGHT = Relay.Value.kReverse;

public static final Relay.Value LEVEL_ONE = Relay.Value.kForward;

public static final Relay.Value LEVEL_TWO = Relay.Value.kReverse;

public static final double TIME_TO_DRIVE_OFF_PLATFORM = 1.0;

public static final double TIME_TO_STRAIGHTEN_OUT_ON_WALL = .6;

public static final double TIME_TO_DRIVE_BACKWARDS_TO_ALIGN = .5;

// cross autoline constants


// straight cargo hatch constants


// rocket hatch contstants- no vision

public static final double DISTANCE_TO_DRIVE_TO_FIRST_TURN_ROCKET = 20;// 60;

public static final int DISTANCE_NEEDED_TO_TURN = 20;// change @ANE

public static final double TIME_TO_DELAY_B4_TURN = 2.0;



// rocket hatch vision constants
public static final double CAMERA_TURN_SPEED = .5;

public static final double CAMERA_ACCELERATION = .2;


public static final double DRIVE_WITH_CAMERA_SPEED = .3;// TODO

public static final int TURN_FOR_CAMERA_DEGREES = 80;

// changed to correct-ish number 2 February 2019
public static final int DISTANCE_TO_CROSS_AUTOLINE_CAMERA = 60;


// public static final int LEFT_DISTANCE_CROSS_AUTOLINE = 60;
// public static final int CENTER_DISTANCE_CROSS_AUTOLINE = 90;
// public static final int RIGHT_DISTANCE_CROSS_AUTOLINE = 120;


// side cargo hatch constants

public static final double DRIVE_STRAIGHT_DEPOSIT_1 = 37;





public static final double DRIVE_STRAIGHT_DEPOSIT_2 = 170;
}
