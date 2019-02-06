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

    // if override button is pressed, ignore encoder.
    if (overrideButton == true)
        {
        this.forkliftTargetSpeed = speed;
        // force the lift state to be move by joysticks
        this.liftState = ForkliftState.MOVE_JOY;
        } else
        {
        // If we are trying to move up and past the max height, or trying to
        // move down and below the min height, tell the forklift to stay where
        // it is
        if ((speed > 0
                && this.getForkliftHeight() > currentForkliftMaxHeight)
                || (speed < 0 && this
                        .getForkliftHeight() < currentMinLiftPosition))
            {
            this.liftState = ForkliftState.STAY_AT_POSITION;
            // return so we exit the method and do not accidentally set
            // liftState to MOVE_JOY
            return;
            }
        // Move the forklift the desired speed; the DOWN_JOYSTICK_SCALAR should
        // usually be less than the UP_JOYSTICK_SCALAR because
        if (speed > 0)
            forkliftTargetSpeed = speed * UP_JOYSTICK_SCALAR;
        else
            forkliftTargetSpeed = speed * DOWN_JOYSTICK_SCALAR;

        this.liftState = ForkliftState.MOVE_JOY;
        }
}

/**
 * Sets the maximum height for the lift. Use only for demo mode.
 *
 * @param inches
 *                   Maximum height, in inches.
 */
public void setMaxLiftHeight (int inches)
{
    this.currentForkliftMaxHeight = inches;
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
        forkliftTargetHeight = position;
        forkliftTargetSpeed = Math.abs(forkliftSpeed);
        liftState = ForkliftState.MOVING_TO_POSITION;
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




// TODO add deadband?
public void setToNextHigherPreset (double forkliftSpeed,
        QuickSwitch goToHeightButton, boolean goingToCargoButtonValue)
{
    if (goToHeightButton.getCurrentValue() == true)
        {
        double position = -1;
        double forkliftHeight = this.getForkliftHeight();

        // if the button to indicate the operator wishes to be going
        // to a cargo height is being pressed
        if (goingToCargoButtonValue == true)
            {
            // set position to the next preset cargo height on the rocket
            // above the forklift's current height
            if (forkliftHeight < LOWER_ROCKET_CARGO)
                {
                position = LOWER_ROCKET_CARGO;
                } else if (forkliftHeight < MIDDLE_ROCKET_CARGO)
                {
                position = MIDDLE_ROCKET_CARGO;
                } else if (forkliftHeight < TOP_ROCKET_CARGO)
                {
                position = TOP_ROCKET_CARGO;
                }
            } else
            {
            // set position to the next preset hatch height on the rocket
            // above the forklift's current height
            if (forkliftHeight < LOWER_ROCKET_HATCH)
                {
                position = LOWER_ROCKET_HATCH;
                } else if (forkliftHeight < MIDDLE_ROCKET_HATCH)
                {
                position = MIDDLE_ROCKET_HATCH;
                } else if (forkliftHeight < TOP_ROCKET_HATCH)
                {
                position = TOP_ROCKET_HATCH;
                }
            }

        SmartDashboard.putNumber("Next Highest Position:", position);
        // if position was set to one of the prest heights
        // (if it was not it would still be -1)
        if (position >= 0.0)
            {
            // tell the forklift state machine we want to move to said
            // position
            this.forkliftTargetHeight = position;
            this.forkliftTargetSpeed = Math.abs(forkliftSpeed);
            this.liftState = ForkliftState.MOVING_TO_POSITION;
            }
        }
}

public void setToNextLowerPreset (double forkliftSpeed,
        QuickSwitch goToHeightButton, boolean goingToCargoButtonValue)
{
    if (goToHeightButton.getCurrentValue() == true)
        {
        double position = -1;
        double forkliftHeight = this.getForkliftHeight();

        // if the button to indicate the operator wishes to be going
        // to a cargo height is being pressed
        if (goingToCargoButtonValue == true)
            {
            // set position to the next preset cargo height on the rocket
            // below the forklift's current height
            if (forkliftHeight > TOP_ROCKET_CARGO)
                {
                position = TOP_ROCKET_CARGO;
                } else if (forkliftHeight > MIDDLE_ROCKET_CARGO)
                {
                position = MIDDLE_ROCKET_CARGO;
                } else if (forkliftHeight > LOWER_ROCKET_CARGO)
                {
                position = LOWER_ROCKET_CARGO;
                }
            } else
            {
            // set position to the next preset hatch height on the rocket
            // below the forklift's current height
            if (forkliftHeight > TOP_ROCKET_HATCH)
                {
                position = TOP_ROCKET_HATCH;
                } else if (forkliftHeight > MIDDLE_ROCKET_HATCH)
                {
                position = MIDDLE_ROCKET_HATCH;
                } else if (forkliftHeight > LOWER_ROCKET_HATCH)
                {
                position = LOWER_ROCKET_HATCH;
                }
            }

        SmartDashboard.putNumber("Next Lower Position:", position);
        // if position was set to one of the prest heights
        // (if it was not it would still be -1)
        if (position >= 0.0)
            {
            // tell the forklift state machine we want to move to said
            // position
            this.forkliftTargetHeight = position;
            this.forkliftTargetSpeed = Math.abs(forkliftSpeed);
            this.liftState = ForkliftState.MOVING_TO_POSITION;
            }
        }
}




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
    // this.printDebugInfo();
    // Make sure the lift stays up to prevent bad things when folding the
    // deploy
    if (manipulator.isDeployed() == false)
        this.currentMinLiftPosition = DEPLOY_FOLDED_MIN_HEIGHT;
    else
        this.currentMinLiftPosition = NO_PIECE_MIN_HEIGHT;

    // main switch statement for the forklift state machine
    switch (liftState)
        {
        case MOVING_TO_POSITION:
            // Make sure we are not trying to move past the MAX or MIN position
            if ((this.forkliftTargetHeight > currentForkliftMaxHeight)
                    || (this.forkliftTargetHeight < currentMinLiftPosition))
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

                } else
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
            System.out.println(
                    "Reached default in the liftState switch in "
                            + "forkliftUpdate in Forklift");
        case STAY_AT_POSITION:
            // Depending on what piece the manipulator has, send the appropriate
            // value to the motor so the forklift does not slide down due to
            // gravity
            switch (manipulator.hasWhichGamePiece())
                {
                case HATCH_PANEL:
                    this.forkliftMotor.set(STAY_UP_WITH_HATCH);
                    break;

                case CARGO:
                    this.forkliftMotor.set(STAY_UP_WITH_CARGO);
                    break;

                default:
                    System.out.println(
                            "Reached default in the GamePiece switch in STAY_AY_POSITION in liftState switch in forkliftUpdate in Forklift");
                case NONE:
                    this.forkliftMotor.set(STAY_UP_NO_PIECE);
                    break;

                }
            // Reset the direction for next move-to-position.
            forkliftDirection = ForkliftDirectionState.NEUTRAL;
            setLiftPositionInit = true;
            break;

        case STOP:
            this.forkliftMotor.set(0.0);
            break;
        }
}

// Useful forklift infromation that can be sent to smart dashboard when we are
// testing
public void printDebugInfo ()
{
    SmartDashboard.putNumber("FL Height: ", this.getForkliftHeight());
    SmartDashboard.putNumber("FL Encoder Ticks: ",
            this.forkliftEncoder.get());
    SmartDashboard.putString("FL Overall State: ", "" + this.liftState);
    SmartDashboard.putString("FL Direction State: ",
            "" + this.forkliftDirection);
    SmartDashboard.putBoolean("FL setLiftPositionInit: ",
            setLiftPositionInit);
}

// ==================
// ----- Fields -----
// ==================
// Enums

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

// Variable for defining the overall state of the forklift
public ForkliftState liftState = ForkliftState.STAY_AT_POSITION;

// Variable for holding which way the forklift needs to move. Used by the
// MOVING_TO_POSITION state of liftState
private ForkliftDirectionState forkliftDirection = ForkliftDirectionState.NEUTRAL;

// Hardware
private SpeedController forkliftMotor;

private KilroyEncoder forkliftEncoder;

private GamePieceManipulator manipulator;

// Variables

private boolean setLiftPositionInit = true;

private double currentForkliftMaxHeight = MAX_HEIGHT;

// used by the MOVING_TO_POSITION state in the state machine to determine what
// position to move to
private double forkliftTargetHeight = 0.0;

// used by the MOVING_TO_POSITION state in the state machine to determine what
// speed to move at
private double forkliftTargetSpeed = 0.0;

private double currentMinLiftPosition = 0;

// Constants

// heights for the top, middle, and bottom openings for the cargo on the
// rocket ship
public final static double TOP_ROCKET_CARGO = 69; // placeholder value

public final static double MIDDLE_ROCKET_CARGO = 26;// placeholder value

public final static double LOWER_ROCKET_CARGO = 0;// placeholder value


// heights for the top, middle, and bottom openings for the hatch
// rocket ship
public final static double TOP_ROCKET_HATCH = 50;// placeholder value

public final static double MIDDLE_ROCKET_HATCH = 30;// placeholder value

public final static double LOWER_ROCKET_HATCH = 10;// placeholder value

// heights for the cargo and hatch openings on the cargo ship
public final static double CARGO_SHIP_CARGO = 0;// placeholder value

public final static double CARGO_SHIP_HATCH = 0;// placeholder value

private static final double MAX_HEIGHT = 69; // placeholder value from last year

private final double DOWN_JOYSTICK_SCALAR = .55;

private final double UP_JOYSTICK_SCALAR = 1.0;

private final double NO_PIECE_MIN_HEIGHT = 0;

private final double DEPLOY_FOLDED_MIN_HEIGHT = 15;

private final double DEFAULT_SPEED_UP = UP_JOYSTICK_SCALAR;

private final double DEFAULT_SPEED_DOWN = DOWN_JOYSTICK_SCALAR;

// for use in teleop when we are calling setLiftPosition(position, speed)
public static final double DEFAULT_TELEOP_BUTTON_SPEED = .6;

// speed sent to the forklift motor to hold position when we do not
// have any game piece
private final double STAY_UP_NO_PIECE = 0.05;

// speed sent to the forklift motor to hold position when we have a
// hatch
private final double STAY_UP_WITH_HATCH = .1;

// speed sent to the forklift motor to hold position when we have a
// cargo
private final double STAY_UP_WITH_CARGO = .1;

private static final double JOYSTICK_DEADBAND = .2;


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
