package frc.Utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.Utils.GamePieceManipulator.GamePiece;

/**
 * Class for controlling the Forklift on the robot. Gets information from, but
 * should be seperate from, the GamePieceManipulator object , in order for the
 * code to be kept modular
 */
public class Forklift {

    // ==================
    // ----- Fields -----
    // ==================
    // Enums

    public enum ForkliftState {
        MOVING_TO_POSITION, MOVE_JOY, STAY_AT_POSITION, STOP
    }

    public enum ForkliftDirectionState {
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

    private double currentForkliftMaxHeight = 0;

    private double forkliftTargetHeight = 0.0;

    private double forkliftTargetSpeed = 0.0;

    private double currentMinLiftPosition = 0;

    private ForkliftDirectionState forkliftDirection = ForkliftDirectionState.NEUTRAL;

    // Constants

    private final double MAX_HEIGHT = 69; // placeholder value from last year

    private final double DOWN_JOYSTICK_SCALAR = .55;

    private final double UP_JOYSTICK_SCALAR = 1.0;

    private final double NO_PIECE_MIN_HEIGHT = 0;

    private final double DEPLOY_FOLDED_MIN_HEIGHT = 15;

    private final double DEFAULT_SPEED_UP = UP_JOYSTICK_SCALAR;

    private final double DEFAULT_SPEED_DOWN = DOWN_JOYSTICK_SCALAR;

    private final double STAY_UP_NO_PIECE_SPEED = 0.05; // -.15;

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
    public Forklift(WPI_TalonSRX liftMotor, KilroyEncoder liftEncoder) {
        this.forkliftMotor = liftMotor;
        this.forkliftEncoder = liftEncoder;
    }

    // ===================
    // ----- Methods -----
    // ===================

    public void moveForkliftWithController(double speed, boolean overrideButton) {

        this.currentForkliftDownSpeed = DOWN_JOYSTICK_SCALAR;

        // Override button, ignore encoder.
        if (overrideButton == true) {
            this.forkliftTargetSpeed = speed;
            this.liftState = ForkliftState.MOVE_JOY;
        } else {
            // If we are past the max height or below the min
            if ((speed > 0 && forkliftEncoder.getDistance() > currentForkliftMaxHeight)
                    || (speed < 0 && forkliftEncoder.getDistance() < currentMinLiftPosition)
                    || Math.abs(speed) < JOYSTICK_DEADBAND) {
                this.liftState = ForkliftState.STAY_AT_POSITION;
                return;
            }
            // Move the forklift the desired speed
            if (speed > 0)
                forkliftTargetSpeed = speed * UP_JOYSTICK_SCALAR;
            else
                forkliftTargetSpeed = speed * currentForkliftDownSpeed;

            this.liftState = ForkliftState.MOVE_JOY;
        }
    }

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

    public void update() {
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
                if (forkliftTargetHeight < forkliftEncoder.getDistance())
                    forkliftDirection = ForkliftDirectionState.MOVING_DOWN;
                else
                    forkliftDirection = ForkliftDirectionState.MOVING_UP;
            }

            // Differentiate moving up from down
            // TODO test scaleIR code
            if (forkliftDirection == ForkliftDirectionState.MOVING_UP) {
                // If we have passed the value we wanted...
                if (this.forkliftEncoder.getDistance() > forkliftTargetHeight) {
                    liftState = ForkliftState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    forkliftDirection = ForkliftDirectionState.NEUTRAL;
                    break;
                }
                // we have NOT passed the value , keep going up.

                this.forkliftMotor.set(forkliftTargetSpeed);

            } else {
                // If we have passed the value we wanted...
                if (this.forkliftEncoder.getDistance() < forkliftTargetHeight) {
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
            System.out.println("Reached default in the liftState switch in " + "forkliftUpdate in CubeManipulator");
        case STAY_AT_POSITION:
            // IF we have a cube, then send a constant voltage.
            switch (manipulator.hasWhichGamePiece()) {

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

}
