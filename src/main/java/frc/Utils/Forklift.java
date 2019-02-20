package frc.Utils;

import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.QuickSwitch;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for controlling the Forklift on the robot. Gets information from, but
 * should be seperate from, the GamePieceManipulator object , in order for the
 * code to be kept modular
 *
 * Adapted for 2019 primarily by Cole
 *
 * Year created: 2019
 *
 */
public class Forklift
{


// ========================
// ----- Constructors -----
// ========================

/**
 * Constructor for Forklift
 *
 * @param liftMotor
 *                                 whichever motor is moving the forklift up and
 *                                 down
 * @param liftEncoder
 *                                 the encoder attached to the liftMotor
 * @param gamePieceManipulator
 *
 */
public Forklift (SpeedController liftMotor, KilroyEncoder liftEncoder,
        GamePieceManipulator gamePieceManipulator)
{
    this.forkliftMotor = liftMotor;
    this.forkliftEncoder = liftEncoder;
    this.manipulator = gamePieceManipulator;
}

// ===================
// ----- Methods -----
// ===================

public void initiliazeConstantsFor2018 ()
{
    SET_LIFT_UPWARD_LIFT_MOVEMENT_SCALER = SET_LIFT_UPWARD_LIFT_MOVEMENT_SCALER_2018;

    SET_LIFT_DOWNWARD_LIFT_MOVEMENT_SCALER = SET_LIFT_DOWNWARD_LIFT_MOVEMENT_SCALER_2018;

    DOWN_JOYSTICK_SCALAR = DOWN_JOYSTICK_SCALAR_2018;

    UP_JOYSTICK_SCALAR = UP_JOYSTICK_SCALAR_2018;

    DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED = DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED_2018;

    STAY_UP_NO_PIECE = STAY_UP_NO_PIECE_2018;

    STAY_UP_WITH_CARGO = STAY_UP_WITH_CARGO_2018;

    DEFAULT_SPEED_UP = UP_JOYSTICK_SCALAR;

    DEFAULT_SPEED_DOWN = DOWN_JOYSTICK_SCALAR;

}




/**
 * Gets the forklift encoder's value, scaled to inches based on what the
 * distance per pulse was set (should have been set in robot init). This
 * corresponds to high the forklift is compared to where it started when the
 * robot was initialized (the encodershould be reset to 0 during
 * initialization).
 *
 * @return the height of the forklift, in inches
 */
public double getForkliftHeight ()
{
    return this.forkliftEncoder.getDistance();
}




/**
 * Moves the forklift up and down based on a Joystick's y axis
 *
 * Implements deadbands inside of this function, so no need to put any if
 * statements around the call for this in Teleop
 *
 * @author Cole
 *
 * @param operator
 *                           - the Joystick that will control the forklift
 * @param overrideButton
 *                           - override button for the forklift
 */
public void moveForkliftWithController (Joystick operator,
        boolean overrideButton)
{
    SmartDashboard.putNumber("Arm Joystick", operator.getY());


    // if (operator.getY() < Forklift.JOYSTICK_DEADBAND)
    // wasAtMax = false;
    // if (operator.getY() > -Forklift.JOYSTICK_DEADBAND)
    // wasAtMin = false;

    // TODO scale the joystick as soon as the deaband is
    // exceeded?
    if (Math.abs(operator.getY()) > Forklift.JOYSTICK_DEADBAND)
        this.moveForkliftAtSpeed(operator.getY(), overrideButton);

}

/**
 * Moves the forklift up and down based on a speed, and scales the speed as
 * appropriate depending on if the forklift is moving up or down (motors are
 * given less power when the forklift is moving down because it is being helped
 * by gravity)
 *
 * Does not use deadbands. For that, use moveForkLiftWithController()
 *
 * @param overrideButton
 *                           the button that, if helf, activates forklift
 *                           override
 * @param speed
 *                           How fast the forklift should be moving, as a
 *                           proportion. Positive values correspond to moving
 *                           the forklift up, negative to moving the forklift
 *                           down.
 *
 */
private void moveForkliftAtSpeed (double speed, boolean overrideButton)
{
    // if the speed is up, the lift cannot be moving
    // towards a min, so resets wasAtMin


    // if override button is pressed, ignore encoder and skip this if
    // statement
    if (overrideButton == false)
        {
        // If we are trying to move up and past the max height, or trying to
        // move down and below the min height, tell the forklift to stay where
        // it is

        if (/* wasAtMin == true || wasAtMax == true || */ ((speed > 0
                && this.getForkliftHeight() > currentLiftMaxHeight)
                || (speed < 0 && this
                        .getForkliftHeight() < currentLiftMinHeight)))
            {
            this.liftState = ForkliftState.STAY_AT_POSITION;

            // if (speed > 0
            // && this.getForkliftHeight() > currentLiftMaxHeight)
            // wasAtMax = true;

            // if (speed < 0 && this
            // .getForkliftHeight() < currentLiftMinHeight)
            // wasAtMin = true;

            // return so we exit the method and do not accidentally set
            // liftState to MOVE_JOY
            return;
            }
        }

    // Move the forklift the desired speed; the DOWN_JOYSTICK_SCALAR should
    // usually be less than the UP_JOYSTICK_SCALAR because
    if (speed > 0)
        forkliftTargetSpeed = speed * UP_JOYSTICK_SCALAR;
    else
        forkliftTargetSpeed = speed * DOWN_JOYSTICK_SCALAR;

    this.liftState = ForkliftState.MOVE_JOY;
}

// // used to keep track if the forklift already stopped itself b/c
// // it was at the max height
// private boolean wasAtMin = false;

// // // used to keep track if the forklift already stopped itself b/c
// // // it was at the min height
// private boolean wasAtMax = false;

/**
 * Sets the maximum height for the lift. Use only for demo mode.
 *
 * @param inches
 *                   Maximum height, in inches.
 */
public void setMaxLiftHeight (int inches)
{
    this.currentLiftMaxHeight = inches;
}

/**
 * Method for setting a forklift to a preset height using a button.
 * For use in teleop. The button just needs to be preseed once (not held)
 * and the forklift state machine will start moving to the necessary height.
 * This can be interruted at any time by moving the joysticks past their
 * deadzones (cuasing joystick control to take over).
 *
 * This should be called directly as is in teleop and does not need to
 * be surrounded by any if statements
 *
 *
 * @param position
 *                          - the height we are moving the forklift to
 * @param forkliftSpeed
 *                          - the desired speed we will move to position at
 * @param button
 *                          - the QuickSwitch we are using to say when we want
 *                          to move to the specified position
 *
 */
public void setLiftPositionByButton (double position,
        double forkliftSpeed, QuickSwitch button)
{
    // if the button is being held down and was not being held down before
    if (button.getCurrentValue() == true)
        {
        // tell the forklift state machine we want to move to a particular
        // position
        setLiftPositionInit = true;
        this.setLiftPosition(position, forkliftSpeed);
        }
}

/**
 * Moves the arm to the the position input, FORKLIFT_MAX_HEIGHT being the top
 * soft stop, and FORKLIFT_MIN_HEIGHT being the FORKLIFT_MIN_HEIGHT.
 *
 * Overloads the setLiftPosition, using the FORKLIFT_SPEED constants
 *
 * Should be used in autonomous only, not in teleop. For use in teleop,
 * use setLiftPositionByButton.
 *
 * @param position
 *                     The position the forklift will move to, in inches.
 * @return true if the forklift is at or above the specified height, false if
 *         still moving
 */
public boolean setLiftPosition (double position)
{
    double defaultSpeed = 0.0;
    // If the requested position is greater than the current position, set
    // the
    // state machine to go up.
    if (this.getForkliftHeight() < position)
        {
        defaultSpeed = DEFAULT_SPEED_UP;
        }
    // Else, we are going down.
    else
        {
        defaultSpeed = DEFAULT_SPEED_DOWN;
        }

    return setLiftPosition(position, defaultSpeed);
}

/**
 * Moves the arm to the desired position, FORKLIFT_MIN_HEIGHT is the bottom,
 * FORKLIFT_MAX_HEIGHT is the top
 *
 * Should be used in autonomous only, not in teleop. For use in teleop,
 * use setLiftPositionByButton.
 *
 * @param position
 *                          The position that the forklift will be set to move
 *                          to
 * @param forkliftSpeed
 *                          how fast the robot should move it's forklift (0.0 to
 *                          1.0)
 * @return true if the forklift is at or above the specified height, false if we
 *         are still moving
 */
public boolean setLiftPosition (double position, double forkliftSpeed)
{
    // Sets the target position and speed, enables "moving-to-position"
    // state.
    if (setLiftPositionInit == true)
        {
        forkliftTargetHeight = position;


        forkliftTargetSpeed = Math.abs(forkliftSpeed);
        forkliftDirection = ForkliftDirectionState.NEUTRAL;
        // if the forklift will move up
        if (forkliftTargetHeight < this.getForkliftHeight())
            {
            forkliftTargetSpeed *= SET_LIFT_DOWNWARD_LIFT_MOVEMENT_SCALER;
            }
        else // if the forklift will move down
            {
            forkliftTargetSpeed *= SET_LIFT_UPWARD_LIFT_MOVEMENT_SCALER;
            }


        liftState = ForkliftState.MOVING_TO_POSITION;
        setLiftPositionInit = false;
        }

    // return true is we are done moving, false is we are still going
    if (liftState == ForkliftState.STAY_AT_POSITION)
        {
        setLiftPositionInit = true;
        return true;
        }
    return false;
}



/**
 * Sets the forklift to the next higher Cargo or Hatch height on the
 * rocket ship when the button associated with goToHeightButton is
 * pressed
 *
 * @param forkliftSpeed
 *                                    - the speed the forklift will move at; if
 *                                    unsure
 *                                    about which to use, use
 *                                    Forklift.DEFAULT_TELEOP_BUTTON_SPEED
 * @param goToHeightButton
 *                                    - the QuickSwitch associated with the
 *                                    button that
 *                                    needs to be pressed to go to the actually
 *                                    move the forklift to the preset heights
 *
 * @param goingToCargoButtonValue
 *                                    - value of the button, that, if true,
 *                                    tells
 *                                    the forklift to go the preset Cargo
 *                                    heights for the rocket ships (when
 *                                    goToHeightButton is pressed); if false, go
 *                                    to the preset hatch heights for the rocket
 *                                    ship (when goToHeightButton is pressed)
 *
 */
public void setToNextHigherPreset (double forkliftSpeed,
        QuickSwitch goToHeightButton, boolean goingToCargoButtonValue)
{
    if (goToHeightButton.getCurrentValue() == true)
        {
        double position = -1;
        double angle = -1;
        double forkliftHeight = this.getForkliftHeight()
                + NEXT_HIGHER_POSITION_DEADBAND;

        // if the button to indicate the operator wishes to be going
        // to a cargo height is being pressed
        if (goingToCargoButtonValue == true)
            {
            // set position to the next preset cargo height on the rocket
            // above the forklift's current height
            if (forkliftHeight < LOWER_ROCKET_CARGO)
                {
                position = LOWER_ROCKET_CARGO;
                angle = LOWER_ROCKET_CARGO_ANGLE;
                }
            else
                if (forkliftHeight < MIDDLE_ROCKET_CARGO)
                    {
                    position = MIDDLE_ROCKET_CARGO;
                    angle = MIDDLE_ROCKET_CARGO_ANGLE;
                    }
                else
                    if (forkliftHeight < TOP_ROCKET_CARGO)
                        {
                        position = TOP_ROCKET_CARGO;
                        angle = TOP_ROCKET_CARGO_ANGLE;
                        }
            }
        else
            {
            // set position to the next preset hatch height on the rocket
            // above the forklift's current height
            if (forkliftHeight < LOWER_ROCKET_HATCH)
                {
                position = LOWER_ROCKET_HATCH;
                angle = LOWER_ROCKET_HATCH_ANGLE;
                }
            else
                if (forkliftHeight < MIDDLE_ROCKET_HATCH)
                    {
                    position = MIDDLE_ROCKET_HATCH;
                    angle = MIDDLE_ROCKET_HATCH_ANGLE;
                    }
                else
                    if (forkliftHeight < TOP_ROCKET_HATCH)
                        {
                        position = TOP_ROCKET_HATCH;
                        angle = TOP_ROCKET_HATCH_ANGLE;
                        }
            }


        // if position was set to one of the prest heights
        // (if it was not it would still be -1)
        if (position >= 0.0)
            {
            // tell the forklift state machine we want to move to said
            // position
            setLiftPositionInit = true;
            this.setLiftPosition(position, forkliftSpeed);
            SmartDashboard.putNumber("Set Higher Angle", angle);
            if (angle >= 0.0)
                this.manipulator.setAngleForForkliftNextPostion(angle);
            }
        }
}

private final double SET_ANGLE_DEADBAND = 3.0;

// # of feet that the forklift can be off of the next highest position to
// count as already being there
private final double NEXT_HIGHER_POSITION_DEADBAND = 1;

/**
 * Sets the forklift to the next lower Cargo or Hatch height on the
 * rocket ship when the button associated with goToHeightButton is
 * pressed
 *
 * @param forkliftSpeed
 *                                    - the speed the forklift will move at; if
 *                                    unsure
 *                                    about which to use, use
 *                                    Forklift.DEFAULT_TELEOP_BUTTON_SPEED
 * @param goToHeightButton
 *                                    - the QuickSwitch associated with the
 *                                    button that
 *                                    needs to be pressed to go to the actually
 *                                    move the forklift to the preset heights
 *
 * @param goingToCargoButtonValue
 *                                    - value of the button, that, if true,
 *                                    tells
 *                                    the forklift to go the preset Cargo
 *                                    heights for the rocket ships (when
 *                                    goToHeightButton is pressed); if false, go
 *                                    to the preset hatch heights for the rocket
 *                                    ship (when goToHeightButton is pressed)
 *
 */
public void setToNextLowerPreset (double forkliftSpeed,
        QuickSwitch goToHeightButton, boolean goingToCargoButtonValue)
{


    if (goToHeightButton.getCurrentValue() == true)
        {
        double position = -1;
        double angle = -1;
        double forkliftHeight = this.getForkliftHeight()
                - NEXT_LOWER_POSITION_DEADBAND;

        // if the button to indicate the operator wishes to be going
        // to a cargo height is being pressed
        if (goingToCargoButtonValue == true)
            {
            // set position to the next preset cargo height on the rocket
            // below the forklift's current height
            if (forkliftHeight > TOP_ROCKET_CARGO)
                {
                position = TOP_ROCKET_CARGO;
                angle = TOP_ROCKET_CARGO_ANGLE;
                }
            else
                if (forkliftHeight > MIDDLE_ROCKET_CARGO)
                    {
                    position = MIDDLE_ROCKET_CARGO;
                    angle = MIDDLE_ROCKET_CARGO_ANGLE;
                    }
                else
                    if (forkliftHeight > LOWER_ROCKET_CARGO)
                        {
                        position = LOWER_ROCKET_CARGO;
                        angle = LOWER_ROCKET_CARGO_ANGLE;
                        }
            }
        else
            {
            // set position to the next preset hatch height on the rocket
            // below the forklift's current height
            if (forkliftHeight > TOP_ROCKET_HATCH)
                {
                position = TOP_ROCKET_HATCH;
                angle = TOP_ROCKET_HATCH_ANGLE;
                }
            else
                if (forkliftHeight > MIDDLE_ROCKET_HATCH)
                    {
                    position = MIDDLE_ROCKET_HATCH;
                    angle = MIDDLE_ROCKET_HATCH_ANGLE;
                    }
                else
                    if (forkliftHeight > LOWER_ROCKET_HATCH)
                        {
                        position = LOWER_ROCKET_HATCH;
                        angle = LOWER_ROCKET_HATCH_ANGLE;
                        }
            }

        // if position was set to one of the prest heights
        // (if it was not it would still be -1)
        if (position >= 0.0)
            {
            // tell the forklift state machine we want to move to said
            // position
            setLiftPositionInit = true;
            this.setLiftPosition(position, forkliftSpeed);
            SmartDashboard.putNumber("Set Lower Angle", angle);
            if (angle >= 0.0)
                this.manipulator.setAngleForForkliftNextPostion(angle);
            }
        }
}

// # of feet that the forklift can be off of the next lowest position to
// count as already being there
private final double NEXT_LOWER_POSITION_DEADBAND = 1;

private final double NEXT_LOWER_ANGLE_ADJUSTMENT = 10;

/**
 * For use in teleop and autonomous periodic.
 *
 * Any functions that move the lift will NOT WORK UNLESS this function is called
 * as well.
 *
 * Runs the forklift movement code in the background, which allows multiple
 * movements in autonomous state machines.
 */
public void update ()
{
    // Make sure the lift stays up to prevent bad things when folding the
    // deploy
    if (manipulator.isArmClearOfFrame() == false)
        this.currentLiftMaxHeight = IS_NOT_CLEAR_FRAME_MAX_HEIGHT;
    else
        this.currentLiftMaxHeight = MAX_HEIGHT;

    if (this.getForkliftHeight() < LIMIT_ARM_ANGLE_HEIGHT)
        this.manipulator
                .setMaxArmAngle(manipulator.MAX_ARM_POSITION_ADJUSTED);
    else
        this.manipulator
                .setMaxArmAngle(manipulator.MAX_FORKLIFT_UP_ANGLE);

    // main switch statement for the forklift state machine
    switch (liftState)
        {
        case MOVING_TO_POSITION:
            // Make sure we are not trying to move past the MAX or MIN position
            if ((this.forkliftTargetHeight > currentLiftMaxHeight)
                    || (this.forkliftTargetHeight < currentLiftMinHeight))
                {
                liftState = ForkliftState.STAY_AT_POSITION;
                break;
                }

            // Begins by stating whether we are increasing or decreasing
            if (forkliftDirection == ForkliftDirectionState.NEUTRAL)
                {
                if (forkliftTargetHeight < this.getForkliftHeight())
                    forkliftDirection = ForkliftDirectionState.MOVING_DOWN;
                else
                    forkliftDirection = ForkliftDirectionState.MOVING_UP;
                }

            // Differentiate moving up from down
            if (forkliftDirection == ForkliftDirectionState.MOVING_UP)
                {
                // If we have passed the value we wanted...
                if (this.getForkliftHeight() > forkliftTargetHeight)
                    {
                    liftState = ForkliftState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    forkliftDirection = ForkliftDirectionState.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value , keep going up.
                this.forkliftMotor.set(forkliftTargetSpeed);

                }
            else
                {
                // If we have passed the value we wanted...
                if (this.getForkliftHeight() < forkliftTargetHeight)
                    {
                    liftState = ForkliftState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    forkliftDirection = ForkliftDirectionState.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value , keep going down.
                this.forkliftMotor.set(-forkliftTargetSpeed);
                }

            break;
        case MOVE_JOY:
            setLiftPositionInit = true;
            this.forkliftMotor.set(forkliftTargetSpeed);
            // If we are no longer holding the joystick, then it will
            // automatically stay at position. If we are holding the joysticks,
            // then other functions will set liftState back to MOVE_JOY before
            // we get back here
            liftState = ForkliftState.STAY_AT_POSITION;
            break;
        default:
            // print out we reached the default case (which we shouldn't
            // have),
            // then fall through to STAY_AT_POSITION
            // System.out.println(
            // "Reached default in the liftState switch in "
            // + "forkliftUpdate in Forklift");
        case STAY_AT_POSITION:
            // If the manipulator has a cargo piece, send the appropriate
            // value to the motor so the forklift does not slide down due to
            // gravity

            if (manipulator.hasCargo() == true)
                this.forkliftMotor.set(STAY_UP_WITH_CARGO);
            else
                this.forkliftMotor.set(STAY_UP_NO_PIECE);

            // Reset the direction for next move-to-position.
            forkliftDirection = ForkliftDirectionState.NEUTRAL;
            setLiftPositionInit = true;
            break;

        case STOP:
            this.forkliftMotor.set(0.0);
            break;
        }
}

/**
 * Resets the forklift encoder to 0, and therefore
 * resets what the forklift thinks the forklift
 * height is to 0. Does not actually move the
 * forklift
 */
public void resetEncoder ()
{
    this.forkliftEncoder.reset();
}

/**
 * Resets the state machine so the forklift does not keep trying to run
 * code from a previous enable after a disable. Should be called in teleop
 * init ONLY.
 */
public void resetStateMachine ()
{
    this.liftState = ForkliftState.STAY_AT_POSITION;
}

// Useful forklift infromation that can be sent to smart dashboard when we
// are testing
public void printDebugInfo ()
{
    SmartDashboard.putNumber("FL Height: ", this.getForkliftHeight());
    SmartDashboard.putNumber("FL Encoder Ticks: ",
            this.forkliftEncoder.get());
    SmartDashboard.putNumber("FL current max height",
            currentLiftMaxHeight);
    SmartDashboard.putNumber("FL current min height",
            currentLiftMinHeight);
    SmartDashboard.putString("FL Overall State: ", "" + this.liftState);
    SmartDashboard.putString("FL Direction State: ",
            "" + this.forkliftDirection);
    SmartDashboard.putBoolean("FL setLiftPositionInit: ",
            setLiftPositionInit);
    SmartDashboard.putNumber("Forklift Motor", forkliftMotor.get());
}

// ==================
// ----- Fields -----
// ==================

// ===== Hardware =====

private SpeedController forkliftMotor;

private KilroyEncoder forkliftEncoder;

private GamePieceManipulator manipulator;

// ===== Enum Declarations =====

// enum for defining the overall states of the forklift
public static enum ForkliftState
    {
    MOVING_TO_POSITION, MOVE_JOY, STAY_AT_POSITION, STOP
    }

// enum for holding which way the forklift needs to move.
public static enum ForkliftDirectionState
    {
    NEUTRAL, MOVING_DOWN, MOVING_UP
    }

// ===== Variables =====

// Variable for defining the overall state of the forklift
public ForkliftState liftState = ForkliftState.STAY_AT_POSITION;

// Variable for holding which way the forklift needs to move. Used by the
// MOVING_TO_POSITION state of liftState
private ForkliftDirectionState forkliftDirection = ForkliftDirectionState.NEUTRAL;

private boolean setLiftPositionInit = true;

private double currentLiftMaxHeight = MAX_HEIGHT;

// used by the MOVING_TO_POSITION state in the state machine to determine what
// position to move to
private double forkliftTargetHeight = 0.0;

// used by the MOVING_TO_POSITION state in the state machine to determine
// what speed to move at
private double forkliftTargetSpeed = 0.0;

private double currentLiftMinHeight = 0.0;

// ===== Constants =====

// ----- Speed Constants -----

private static final double JOYSTICK_DEADBAND = .2;

private double SET_LIFT_UPWARD_LIFT_MOVEMENT_SCALER = 0.6;

// leave this positive even though it is the downward scalar;
// the speed is multipled by a negative value
private double SET_LIFT_DOWNWARD_LIFT_MOVEMENT_SCALER = 0.2;

private double UP_JOYSTICK_SCALAR = 0.6;

private double DOWN_JOYSTICK_SCALAR = 0.2;

private double DEFAULT_SPEED_UP = UP_JOYSTICK_SCALAR;

private double DEFAULT_SPEED_DOWN = DOWN_JOYSTICK_SCALAR;

// for use in teleop when we are calling setLiftPosition(position, speed)
public static double DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED = 1.0;

// speed sent to the forklift motor to hold position when we do not
// have any game piece
private double STAY_UP_NO_PIECE = 0.1;

// speed sent to the forklift motor to hold position when we have a
// cargo
private double STAY_UP_WITH_CARGO = 0.2;// TODO

// ----- Preset Heights -----

// heights for the top, middle, and bottom openings for the cargo on the
// rocket ship
public final static double TOP_ROCKET_CARGO = 57; // placeholder value

public final static double MIDDLE_ROCKET_CARGO = 36.5;

public final static double LOWER_ROCKET_CARGO = 8.5;

public final static double TOP_ROCKET_CARGO_ANGLE = 70; // placeholder value

public final static double MIDDLE_ROCKET_CARGO_ANGLE = 60;

public final static double LOWER_ROCKET_CARGO_ANGLE = 60;

// heights for the top, middle, and bottom openings for the hatch
// rocket ship
public final static double TOP_ROCKET_HATCH = 57;// placeholder value

public final static double MIDDLE_ROCKET_HATCH = 35;

public final static double LOWER_ROCKET_HATCH = 10;

public final static double TOP_ROCKET_HATCH_ANGLE = 40;// placeholder value

public final static double MIDDLE_ROCKET_HATCH_ANGLE = 20;

public final static double LOWER_ROCKET_HATCH_ANGLE = 20;

// heights for the cargo and hatch openings on the cargo ship
public final static double CARGO_SHIP_CARGO = 45;

public final static double CARGO_SHIP_HATCH = 40;

private final static double CARGO_SHIP_CARGO_ANGLE = 45;

private final static double CARGO_SHIP_HATCH_ANGLE = 40;


private static final double MAX_HEIGHT = 57; // placeholder value from last year

private double IS_NOT_CLEAR_FRAME_MAX_HEIGHT = 0;

private double LIMIT_ARM_ANGLE_HEIGHT = .75;

private final double NO_PIECE_MIN_HEIGHT = 0;

private final double DEPLOY_FOLDED_MIN_HEIGHT = 15;

private double PAST_CONSIDER_OUT_OF_FRAME_HEIGHT = 35;
// ----- Forklift Speed Constants 2018 ------

private static final double SET_LIFT_UPWARD_LIFT_MOVEMENT_SCALER_2018 = 1.0;

// leave this positive even though it is the downward scalar;
// the speed is multipled by a negative value
private static final double SET_LIFT_DOWNWARD_LIFT_MOVEMENT_SCALER_2018 = .55;

private final double UP_JOYSTICK_SCALAR_2018 = 1.0;

private final double DOWN_JOYSTICK_SCALAR_2018 = .55;

private final double DEFAULT_SPEED_UP_2018 = UP_JOYSTICK_SCALAR_2018;

private final double DEFAULT_SPEED_DOWN_2018 = DOWN_JOYSTICK_SCALAR_2018;

public static final double DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED_2018 = 1.0;

private final double STAY_UP_NO_PIECE_2018 = 0.05;

private final double STAY_UP_WITH_CARGO_2018 = .1;

private final double FORKLIFT_STARTING_HEIGHT_FROM_FLOOR = 0;

/**
 * ROCKET information from the game manual.
 * __________________________________________________________________________
 * CARGO PORTS
 * The center of the bottommost port is 2'3.5" (two feet and 3 and a half
 * inches) from the carpet.
 * The center of the middle port is 4'7.5" (four feet and 7 and a half
 * inches)
 * from the carpet.
 * THe center of the topmost port is 6'11.5" (six feet and 11 and a half
 * inches)
 * from the carpet.
 * __________________________________________________________________________
 * HATCH OPENINGS
 * The center of the lowest hatch opening is 1'7" (one foot and seven
 * inches) from
 * the carpet.
 * The center of the middle hatch opening is 3'11" (three feet and eleven
 * inches)
 * from the carpet.
 * The center of the topmost hatch opening is 6'3" (six feet and three
 * inches)
 * from the carpet.
 * _________________________________________________________________________
 * CARGO SHUTTLE information from the game manual.
 * The center of the shuttle opening(above the hatch) for the cargo is
 * 3'3.75" (three feet and three and three-fourths inches) from the carpet.
 * The center of the hatch opening is 1'7" (one foot and seven inches) from
 * the carpet.
 * _________________________________________________________________________
 * SOURCES:
 * ROCKET information at page 17 of the game manual (21 of the pdf)
 * CARGO SHUTTLE information at page 21 of the game manual (25 of the pdf)
 */

}
