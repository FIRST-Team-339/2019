package frc.Utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.Utils.GamePieceManipulator.GamePiece;
import edu.wpi.first.wpilibj.Joystick;

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
public class Forklift {

    // ==================
    // ----- Fields -----
    // ==================
    // Enums

    public static enum ForkliftState {
        MOVING_TO_POSITION, MOVE_JOY, STAY_AT_POSITION, STOP
    }

    public static enum ForkliftDirectionState {
        NEUTRAL, MOVING_DOWN, MOVING_UP
    }

    public ForkliftState liftState = ForkliftState.STAY_AT_POSITION;

    // Hardware
    private WPI_TalonSRX forkliftMotor;
    private KilroyEncoder forkliftEncoder;

    private GamePieceManipulator manipulator;

    // Variables

    private boolean setLiftPositionInit = true;

    private double currentForkliftDownSpeed = 0;

    private double currentForkliftMaxHeight = MAX_HEIGHT;

    private double forkliftTargetHeight = 0.0;

    private double forkliftTargetSpeed = 0.0;

    private double currentMinLiftPosition = 0;

    private ForkliftDirectionState forkliftDirection = ForkliftDirectionState.NEUTRAL;

    // Constants

    public final static double TOP_ROCKET_CARGO = 69;

    public final static double MIDDLE_ROCKET_CARGO = 26;

    public final static double LOWER_ROCKET_CARGO = 0;

    public final static double TOP_ROCKET_HATCH = 50;

    public final static double MIDDLE_ROCKET_HATCH = 30;

    public final static double LOWER_ROCKET_HATCH = 10;

    public final static double CARGO_SHIP_CARGO = 0;

    public final static double CARGO_SHIP_HATCH = 0;

    private static final double MAX_HEIGHT = 40; // placeholder value from last year

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

    // ========================
    // ----- Constructors -----
    // ========================

    /**
     * Constructor for Forklift
     *
     * @param liftMotor
     * @param liftEncoder
     */
    public Forklift(WPI_TalonSRX liftMotor, KilroyEncoder liftEncoder, GamePieceManipulator gamePieceManipulator) {
        this.forkliftMotor = liftMotor;
        this.forkliftEncoder = liftEncoder;
        this.manipulator = gamePieceManipulator;
    }

    // ===================
    // ----- Methods -----
    // ===================

    /**
     * Gets the value from the encoder, which is the absolute position above the
     * ground.
     *
     * @return the height of the forklift, in inches
     */
    public double getForkliftHeight() {
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
     * @param operator       - the Joystick that will control the forklift
     * @param overrideButton - override button for the forklift
     */
    public void moveForkliftWithController(Joystick operator, boolean overrideButton) {
        if (Math.abs(operator.getY()) > Forklift.JOYSTICK_DEADBAND)
            this.moveForkliftAtSpeed(operator.getY(), overrideButton);

    }

    /**
     * Moves the forklift up and down based on a speed
     *
     * Does not use deadbands. For that, use moveForkLiftWithController()
     *
     * @param overrideButton the button that, if helf, activates forklift override
     * @param speed          How fast the forklift should be moving, as a proportion
     *
     */
    private void moveForkliftAtSpeed(double speed, boolean overrideButton) {

        // if override button is pressed, ignore encoder.
        if (overrideButton == true) {
            this.forkliftTargetSpeed = speed;
            this.liftState = ForkliftState.MOVE_JOY;
        } else {
            // If we are past the max height or below the min
            if ((speed > 0 && this.getForkliftHeight() > currentForkliftMaxHeight)
                    || (speed < 0 && this.getForkliftHeight() < currentMinLiftPosition)) {
                this.liftState = ForkliftState.STAY_AT_POSITION;
                return;
            }
            // Move the forklift the desired speed
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
     * @param inches Maximum height, in inches.
     */
    public void setMaxLiftHeight(int inches) {
        this.currentForkliftMaxHeight = inches;
    }

    /**
     * Button used
     *
     */
    public void setLiftPositionByButton(double position, double forkliftSpeed, boolean button) {
        if (button == true) {
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
     * @param position The position the forklift will move to, in inches.
     * @return true if the forklift is at or above the specified height, false if
     *         still moving
     */
    public boolean setLiftPosition(double position) {
        double defaultSpeed = 0.0;
        // If the requested position is greater than the current position, set
        // the
        // state machine to go up.
        if (this.getForkliftHeight() < position) {
            defaultSpeed = DEFAULT_SPEED_UP;
        }
        // Else, we are going down.
        else {
            defaultSpeed = DEFAULT_SPEED_DOWN;
        }

        return setLiftPosition(position, defaultSpeed);
    }

    /**
     * Moves the arm to the desired position, FORKLIFT_MIN_HEIGHT is the bottom,
     * FORKLIFT_MAX_HEIGHT is the top
     *
     * @param position      The position that the forklift will be set to move to
     * @param forkliftSpeed how fast the robot should move it's forklift (0.0 to
     *                      1.0)
     * @return true if the forklift is at or above the specified height, false if we
     *         are still moving
     */
    public boolean setLiftPosition(double position, double forkliftSpeed) {
        // Sets the target position and speed, enables "moving-to-position"
        // state.
        if (setLiftPositionInit == true) {
            forkliftTargetHeight = position;
            forkliftTargetSpeed = Math.abs(forkliftSpeed);

            liftState = ForkliftState.MOVING_TO_POSITION;

            setLiftPositionInit = false;
        }

        // return true is we are done moving, false is we are still going
        if (liftState == ForkliftState.STAY_AT_POSITION) {
            setLiftPositionInit = true;
            return true;
        }
        return false;
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
    public void update() {

        // this.printDebugInfo();
        // Make sure the lift stays up to prevent bad things when folding the
        // deploy
        if (manipulator.isDeployed() == false)
            this.currentMinLiftPosition = DEPLOY_FOLDED_MIN_HEIGHT;
        else
            this.currentMinLiftPosition = NO_PIECE_MIN_HEIGHT;

        // main switch statement for the forklift state machine
        switch (liftState) {
        case MOVING_TO_POSITION:
            // Make sure we don't move past the MAX or MIN position
            if ((this.forkliftTargetHeight > currentForkliftMaxHeight)
                    || (this.forkliftTargetHeight < currentMinLiftPosition)) {
                liftState = ForkliftState.STAY_AT_POSITION;
                break;
            }

            // Begins by stating whether we are increasing or decreasing
            if (forkliftDirection == ForkliftDirectionState.NEUTRAL) {
                if (forkliftTargetHeight < this.getForkliftHeight())
                    forkliftDirection = ForkliftDirectionState.MOVING_DOWN;
                else
                    forkliftDirection = ForkliftDirectionState.MOVING_UP;
            }

            // Differentiate moving up from down
            if (forkliftDirection == ForkliftDirectionState.MOVING_UP) {
                // If we have passed the value we wanted...
                if (this.getForkliftHeight() > forkliftTargetHeight) {
                    liftState = ForkliftState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    forkliftDirection = ForkliftDirectionState.NEUTRAL;
                    break;
                }
                // we have NOT passed the value , keep going up.

                this.forkliftMotor.set(forkliftTargetSpeed);

            } else {
                // If we have passed the value we wanted...
                if (this.getForkliftHeight() < forkliftTargetHeight) {
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
            // IF we are no longer holding the joystick, then it will
            // automatically stay at position.
            liftState = ForkliftState.STAY_AT_POSITION;
            break;
        default:
            // print out we reached the default case (which we shouldn't
            // have),
            // then fall through to STAY_AT_POSITION
            System.out.println("Reached default in the liftState switch in " + "forkliftUpdate in Forklift");
        case STAY_AT_POSITION:
            // IF we have a cube, then send a constant voltage.
            switch (manipulator.hasWhichGamePiece()) {
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

    public void printDebugInfo() {
        System.out.println("FL Height: " + this.getForkliftHeight());
        System.out.println("FL Encoder Ticks: " + this.forkliftEncoder.get());
        System.out.println("FL Overall State: " + this.liftState);
        System.out.println("FL Direction State: " + this.forkliftDirection);
        System.out.println("FL setLiftPositionInit: " + setLiftPositionInit);
    }

}
