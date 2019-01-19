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
public class Autonomous {

    /**
     * User Initialization code for autonomous mode should go here. Will run once
     * when the autonomous first starts, and will be followed immediately by
     * periodic().
     */
    public static void init() {
        // --------------------------------------
        // reset the MotorSafetyHelpers for each
        // of the drive motors
        // --------------------------------------
        // Hardware.leftDriveMotor.setSafetyEnabled(false);
        // Hardware.rightDriveMotor.setSafetyEnabled(false);

        // Hardware.leftFrontDriveEncoder.reset();
        // Hardware.rightFrontDriveEncoder.reset();

        if (Hardware.levelStartSwitch.isOn() == true)
            autoState = State.FINISH;

    } // end Init

    /**
     * State of autonomous as a whole; mainly for init, delay, finish, and choosing
     * which autonomous path is being used
     */
    public static enum State {
        INIT, DELAY, CHOOSE_PATH, CROSS_AUTOLINE, DEPOSIT_CARGO_HATCH, DEPOSIT_ROCKET_HATCH, DEPOSIT_SIDE_CARGO_HATCH,
        FINISH
    }

    /**
     * Starting position and which side of the field the robot is going to
     */

    public static enum Position {
        LEFT, RIGHT, NULL
    }

    public static enum Level {
        LEVEL1, LEVEL2, Disable
    }

    // variable that controls the state of autonomous as a whole (init, delay
    // which path is being used, etc.)
    public static State autoState = State.INIT;

    /**
     * User Periodic code for autonomous mode should go here. Will be called
     * periodically at a regular rate while the robot is in autonomous mode.
     *
     * @author Nathanial Lydick
     * @written Jan 13, 2015
     */
    public static void periodic() {
        switch (autoState) {
        case INIT:

            autoState = State.DELAY;
            break;
        case DELAY:

            // Delay using the potentiometer, from 0 to 5 seconds
            // once finished, stop the timer and go to the next state

            if (Hardware.autoTimer.get() >= Hardware.delayPot.get(0.0, 5.0)) {
                autoState = State.CHOOSE_PATH;
                Hardware.autoTimer.stop();
                break;
            }
            break;

        case CROSS_AUTOLINE:

            break;

        case DEPOSIT_CARGO_HATCH:

            break;

        case DEPOSIT_ROCKET_HATCH:

            break;

        case DEPOSIT_SIDE_CARGO_HATCH:

            break;

        case FINISH:

            break;

        default:
            autoState = State.FINISH;
            break;
        }

    }

    // ---------------------------------
    // Methods
    // ---------------------------------

    public void setMatchSettings() {

    }

    /*
     * ================================ Constants ================================
     */

} // end class
