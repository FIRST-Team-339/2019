package frc.Utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Timer;

public class GamePieceManipulator
{

private SpeedController armMotor = null;

private RobotPotentiometer armPot = null;

private KilroyEncoder armEncoder = null;

private RollerIntakeMechanism intake = null;




/**
 * constructor to use in hardware
 */
public GamePieceManipulator (SpeedController armMotor,
        RobotPotentiometer armPot, SpeedController armRollers,
        LightSensor photoSwitch)
{
    this.armMotor = armMotor;
    this.armPot = armPot;
    this.intake = new RollerIntakeMechanism(armRollers, photoSwitch);
}

/**
 * constructor to use in hardware
 */
public GamePieceManipulator (SpeedController armMotor,
        KilroyEncoder armEncoder, SpeedController armRollers,
        LightSensor photoSwitch)
{
    this.armMotor = armMotor;
    this.armEncoder = armEncoder;
    this.intake = new RollerIntakeMechanism(armRollers, photoSwitch);
}

public static enum GamePiece
    {
HATCH_PANEL, CARGO, NONE, BOTH
    }

// placeholder, will need to do something
// used to tell us things also
public static enum DeployState
    {

    }

public static enum DeployMovementState
    {
MOVING_TO_POSITION, MOVING_BY_JOY, STAY_AT_POSITION, STOP
    }

// =========================================================================
// General Methods
// =========================================================================

// placeholder function since Forklift will need to understand which piece
// the manipulator has
public GamePiece hasWhichGamePiece ()
{
    if (this.intake.hasCargo() /* and does not have a Hatch */)
        {
        return GamePiece.CARGO;
        }

    return GamePiece.NONE;
}

// placeholder, will need to be changed
public boolean isDeployed ()
{
    return true;
}

/** Update all the states machines for this class */
public void masterUpdate ()
{
    this.intake.update();
}

// =========================================================================
// armMotor methods
// =========================================================================

/**
 * call during teleop to move the arm up and down based on the joystick controls
 */

public void moveArmByJoystick (Joystick armJoystick)
{
    if (Math.abs(armJoystick
            .getY()) < DEPLOY_JOYSTICK_DEADBAND)
    // if ((getCurrentArmPosition() < MAX_ARM_POSITION)
    // && (getCurrentArmPosition() > MIN_ARM_POSITION))
        {
        // TODO change to use the state machine
        this.armMotor.set(armJoystick.getY());
        }
}

public int getCurrentArmPosition ()
{
    return armEncoder.get();// armPot.get();
}

// ready to test
public void moveArmToPosition (int targetPosition)
{
    if (getCurrentArmPosition() > targetPosition + ACCEPTABLE_ERROR)
        {
        armMotor.set(LOWER_ARM_SPEED);
        } else if (getCurrentArmPosition() < targetPosition
                - ACCEPTABLE_ERROR)
        {
        armMotor.set(RAISE_ARM_SPEED);
        } else
        {
        armMotor.set(HOLD_ARM_SPEED);
        }
}


public void deployUpdate ()
{
    switch (deployMovementState)
        {
        case MOVING_BY_JOY:
            // if ((this.forkliftTargetHeight > currentForkliftMaxHeight)
            // || (this.forkliftTargetHeight < currentMinLiftPosition))
            // {
            // liftState = ForkliftState.STAY_AT_POSITION;
            // break;
            // }

            // // Begins by stating whether we are increasing or decreasing
            // if (forkliftDirection == ForkliftDirectionState.NEUTRAL)
            // {
            // if (forkliftTargetHeight < this.getForkliftHeight())
            // forkliftDirection = ForkliftDirectionState.MOVING_DOWN;
            // else
            // forkliftDirection = ForkliftDirectionState.MOVING_UP;
            // }

            // // Differentiate moving up from down
            // if (forkliftDirection == ForkliftDirectionState.MOVING_UP)
            // {
            // // If we have passed the value we wanted...
            // if (this.getForkliftHeight() > forkliftTargetHeight)
            // {
            // liftState = ForkliftState.STAY_AT_POSITION;
            // // Reset the direction for next time.
            // forkliftDirection = ForkliftDirectionState.NEUTRAL;
            // break;
            // }
            // // we have NOT passed the value , keep going up.

            // this.forkliftMotor.set(forkliftTargetSpeed);

            // } else
            // {
            // // If we have passed the value we wanted...
            // if (this.getForkliftHeight() < forkliftTargetHeight)
            // {
            // liftState = ForkliftState.STAY_AT_POSITION;
            // // Reset the direction for next time.
            // forkliftDirection = ForkliftDirectionState.NEUTRAL;
            // break;
            // }
            // // we have NOT passed the value , keep going down.
            // this.forkliftMotor.set(-forkliftTargetSpeed);
            // }
            break;
        case MOVING_TO_POSITION:
            break;

        default:
        case STAY_AT_POSITION:
            break;

        case STOP:

            break;

        }
}
// =========================================================================
// Hatch Panel Methods
// =========================================================================

public boolean depositHatch ()
{
    return false;
}

// =========================================================================
// roller methods
// =========================================================================

// TODO do we just want it so if you hit the override, even without pulling
// trigger, it intakes?
/**
 * Method for calling intake and outtake when one button is for moving the
 * rollers, and the other determines which direction they are being moved in
 *
 * This is private because this control scheme is not the one that the
 * operators want. intakeOuttakeByButtonsSeperated should be used instead.
 *
 * @param intakeButtonValue
 *                                      value of the button used for intake/
 *                                      outtake
 * @param reverseIntakeButtonValue
 *                                      value of the button that, if held, will
 *                                      reverse the direction on the intake
 *                                      motors when the intakeButton is held
 *                                      (causing the manipulator to outake
 *                                      instead of intale when both buttons are
 *                                      held)
 * @param intakeOverrideButtonValue
 *                                      value of the override button for intake,
 *                                      used if the photoSwitch is failing
 */
private void intakeOuttakeByButtons (boolean intakeButtonValue,
        boolean reverseIntakeButtonValue,
        boolean intakeOverrideButtonValue)
{
    this.intake.intakeOuttakeByButtons(intakeButtonValue,
            reverseIntakeButtonValue, intakeOverrideButtonValue);
}


/**
 * Method for calling intake and outtake when they are both mapped to two
 * different buttons. This is in contrast to the intakeOuttakeByButtons
 * method, which has one button for intake, and another that
 * reverses the intake
 *
 * @param intakeButtonValue
 *                                      value of the button used for intake
 * @param reverseIntakeButtonValue
 *                                      value of the button used for outtake
 * @param intakeOverrideButtonValue
 *                                      value of the override button for intake,
 *                                      used if the photoSwitch is failing
 */
public void intakeOuttakeByButtonsSeperated (boolean intakeButtonValue,
        boolean outtakeButtonValue, boolean intakeOverrideButtonValue)
{
    this.intakeOuttakeByButtonsSeperated(intakeButtonValue,
            outtakeButtonValue, intakeOverrideButtonValue);
}




// =========================================================================
// Constants
// =========================================================================

private static final double DEPLOY_JOYSTICK_DEADBAND = 0.2;

private static final int MAX_ARM_POSITION = 170;

private static final int MIN_ARM_POSITION = 0;

private static final int RETRACTED_ARM_POSITION = 150;

private static final int GROUND_ARM_POSITION = 10;

private static final int ACCEPTABLE_ERROR = 6;

private static final double LOWER_ARM_SPEED = -.3;

private static final double RAISE_ARM_SPEED = .5;

private static final double HOLD_ARM_SPEED = .2;



// =========================================================================
// Variables
// =========================================================================

public DeployMovementState deployMovementState = DeployMovementState.STAY_AT_POSITION;

// =========================================================================
// Tuneables
// =========================================================================

private Timer rollerTimer = new Timer();

}
