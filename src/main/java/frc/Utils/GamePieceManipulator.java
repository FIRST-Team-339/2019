package frc.Utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.HardwareInterfaces.QuickSwitch;

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

private static enum DeployMovementDirection
    {
MOVING_UP, MOVING_DOWN, NEUTRAL
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
    this.deployUpdate();
}

// =========================================================================
// armMotor methods
// =========================================================================


/**
 * call during teleop to move the arm up and down based on the joystick controls
 */
public void moveArmByJoystick (Joystick armJoystick,
        boolean overrideButton)
{
    if (Math.abs(armJoystick
            .getY()) > DEPLOY_JOYSTICK_DEADBAND)
        {
        // assuming up is a positive value
        double speed = (armJoystick.getY() - DEPLOY_JOYSTICK_DEADBAND)
                * DEPLOY_JOYSTICK_DEADBAND_SCALER;
        // if override button is pressed, ignore potentiometer/ encoder.
        if (overrideButton == true)
            {
            this.deployTargetSpeed = speed;
            // force the lift state to be move by joysticks
            this.deployMovementState = DeployMovementState.MOVING_BY_JOY;
            } else
            {
            // If we are trying to move up and past the max angle, or
            // trying to move down and below the min height, tell the
            // arm to stay where it is
            if ((speed > 0
                    && this.getCurrentArmPosition() > currentDeployMaxAngle)
                    || (speed < 0 && this
                            .getCurrentArmPosition() < currentDeployMinAngle))
                {
                this.deployMovementState = DeployMovementState.STAY_AT_POSITION;
                // return so we exit the method and do not accidentally set
                // deployMovementState to MOVING_BY_JOY;
                return;
                }

            // scales the speed based on whether it is going up or down
            if (speed > 0)
                deployTargetSpeed = speed * UP_JOYSTICK_SCALER;
            else
                deployTargetSpeed = speed * DOWN_JOYSTICK_SCALER;

            this.deployMovementState = DeployMovementState.MOVING_BY_JOY;
            }
        }

}

/**
 * Adjusts the arm potentiometer value so that it is zeroed
 * at the undeployed position and being below the undeployed
 * position has a negative value
 */
private double armPotAdjusted ()
{
    // TODO no idea if this is the right value b/c I
    // do not know if the potentiometer can be reset at
    // robot init
    return armPot.get();
}

public double getCurrentArmPosition ()
{
    if (armPot != null)
        {
        double valueFromHorizontal = (this.armPotAdjusted()
                - ARM_POT_RAW_HORIZONTAL_VALUE)
                * ARM_POT_SCALE_TO_DEGREES;

        return valueFromHorizontal;
        } else // if we are not using an armPot, we should be using an encoder
        {
        // assumes that the value from the encoder is reset to 0
        // when the robot is started and negative when the manipulator
        // is below the starting position
        // TODO should getDistance be used instead of get?
        double valueFromHorizontal = (armEncoder.get()
                - ARM_ENCODER_RAW_HORIZONTAL_VALUE)
                * ARM_ENCODER_SCALE_TO_DEGREES;

        return valueFromHorizontal;
        }
}



/**
 * Method for setting the deploy arm to a preset angle using a button.
 * For use in teleop. The button just needs to be pressed once (not held)
 * and the dpeloy state machine will start moving to the necessary angle.
 * This can be interruted at any time by moving the joysticks past their
 * deadzones (causing joystick control to take over).
 *
 * This should be called directly as is in teleop and does not need to
 * be surrounded by any if statements
 *
 *
 * @param angle
 *                     the angle the arm will be moved to
 * @param armSpeed
 *                     the desired speed the arm will be moved at
 * @param button
 *                     the QuickSwitch we are using to say when we want
 *                     to move to the specified angle
 *
 */
public void moveArmByButton (double angle,
        double armSpeed, QuickSwitch button)
{
    // if the button is being held down and was not being held down before
    if (button.getCurrentValue() == true)
        {
        // tell the forklift state machine we want to move to a particular
        // position
        this.deployTargetAngle = angle;
        this.deployTargetSpeed = Math.abs(armSpeed);
        this.deployMovementState = DeployMovementState.MOVING_TO_POSITION;
        }
}

/**
 * Function to move the deploy arm to a specified angle. For use
 * in autonomous only (not in teleop). For teleop, please use
 * moveArmByButton.
 *
 * @param angle
 *                  the target angle the arm will move towards
 *
 * @param speed
 *                  the speed the arm will move at
 *
 *
 * @return true when the arm has finished moving to the proper
 *         position, false otherwise
 */
public boolean moveArmToPosition (double angle, double speed)
{
    // Sets the target position and speed, enables "moving-to-position"
    // state.
    if (isSetDeployPositionInitReady == true)
        {
        this.deployTargetAngle = angle;
        this.deployTargetSpeed = Math.abs(speed);

        this.deployMovementState = DeployMovementState.MOVING_TO_POSITION;

        isSetDeployPositionInitReady = false;
        }

    // return true is we are done moving, false is we are still going
    if (this.deployMovementState == DeployMovementState.STAY_AT_POSITION)
        {
        isSetDeployPositionInitReady = true;
        return true;
        }
    return false;
}



public void deployUpdate ()
{
    SmartDashboard.putString("Arm Potentiometer", "" + armPot.get());
    switch (deployMovementState)
        {
        case MOVING_TO_POSITION:
            if ((this.deployTargetAngle > currentDeployMaxAngle)
                    || (this.deployTargetAngle < currentDeployMinAngle))
                {
                deployMovementState = DeployMovementState.STAY_AT_POSITION;
                break;
                }

            // Begins by stating whether we are increasing or decreasing
            if (deployDirection == DeployMovementDirection.NEUTRAL)
                {
                if (deployTargetAngle < this.getCurrentArmPosition())
                    deployDirection = DeployMovementDirection.MOVING_DOWN;
                else
                    deployDirection = DeployMovementDirection.MOVING_UP;
                }

            // Differentiate moving up from down
            if (deployDirection == DeployMovementDirection.MOVING_UP)
                {
                // If we have passed the value we wanted...
                if (this.getCurrentArmPosition() > deployTargetAngle)
                    {
                    deployMovementState = DeployMovementState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    deployDirection = DeployMovementDirection.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value , keep going up.

                this.armMotor.set(deployTargetSpeed);

                } else
                {
                // If we have passed the value we wanted...
                if (this.getCurrentArmPosition() < deployTargetAngle)
                    {
                    deployMovementState = DeployMovementState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    deployDirection = DeployMovementDirection.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value , keep going down.
                this.armMotor.set(-deployTargetSpeed);
                }
            break;
        case MOVING_BY_JOY:
            isSetDeployPositionInitReady = true;
            this.armMotor.set(deployTargetSpeed);
            // If we are no longer holding the joystick, then it will
            // automatically stay at position. If we are holding the
            // joysticks, then other functions will set
            // deployMovementState back to MOVINg_BY_JOY before we get
            // back here
            deployMovementState = DeployMovementState.STAY_AT_POSITION;
            break;

        default:
        case STAY_AT_POSITION:
            // TODO the new armMotor might not even need a voltage to
            // stay in place, so we might be able to just give the arm motor
            // 0.0 no matter what game piece we have

            // Depending on what piece the manipulator has, send the appropriate
            // value to the motor so the forklift does not slide down due to
            // gravity
            switch (this.hasWhichGamePiece())
                {
                case HATCH_PANEL:
                    this.armMotor.set(STAY_UP_WITH_HATCH);
                    break;

                case CARGO:
                    this.armMotor.set(STAY_UP_WITH_CARGO);
                    break;

                default:
                case NONE:
                    this.armMotor.set(STAY_UP_NO_PIECE);
                    break;

                }
            // Reset the direction for next move-to-position.
            deployDirection = DeployMovementDirection.NEUTRAL;
            isSetDeployPositionInitReady = true;
            break;

        case STOP:
            this.armMotor.set(0.0);
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
    this.intake.intakeOuttakeByButtonsSeperated(intakeButtonValue,
            outtakeButtonValue, intakeOverrideButtonValue);
}




// =========================================================================
// Constants
// =========================================================================

// ----- Joystick Constants -----
private static final double DEPLOY_JOYSTICK_DEADBAND = 0.2;

// should be equal to 1/(1 - DEPLOY_JOYSTICK_DEADBAND)
private static final double DEPLOY_JOYSTICK_DEADBAND_SCALER = 1.25;

private static final double UP_JOYSTICK_SCALER = .5;

private static final double DOWN_JOYSTICK_SCALER = .5;


// ----- Deploy Constants -----

private static final int MAX_ARM_POSITION = 170;

private static final int MIN_ARM_POSITION = 0;

private static final int RETRACTED_ARM_POSITION = 150;

private static final int GROUND_ARM_POSITION = 10;

private static final int ACCEPTABLE_ERROR = 6;

private static final double LOWER_ARM_SPEED = -.3;

private static final double RAISE_ARM_SPEED = .5;

private static final double HOLD_ARM_SPEED = .2;

// value that the arm pot returns when the manipulator is
// parallel to the floor
private static final double ARM_POT_RAW_HORIZONTAL_VALUE = -90; // placeholder

// value that the arm encoder returns when the manipulator is
// parallel to the floor
private static final double ARM_ENCODER_RAW_HORIZONTAL_VALUE = 0.0; // placeholder

// value that is multipled to the value from the arm pot to convert
// it to degrees
private static final double ARM_POT_SCALE_TO_DEGREES = 1.0; // placeholder

// value that is multiplied by the number of ticks to convert it to degreesf
private static final double ARM_ENCODER_SCALE_TO_DEGREES = 0.0; // placeholder

private static double STAY_UP_WITH_HATCH = 0.0;

private static double STAY_UP_WITH_CARGO = 0.0;

private static double STAY_UP_NO_PIECE = 0.0;


// =========================================================================
// Variables
// =========================================================================

private DeployMovementState deployMovementState = DeployMovementState.STAY_AT_POSITION;

private DeployMovementDirection deployDirection = DeployMovementDirection.NEUTRAL;

// The angle the manipulator is trying to move to; 0 is the start angle,
// positive angles are above the start, negative angles are below the starts
private double deployTargetAngle = 0.0;

private double deployTargetSpeed = 0.0;

private double currentDeployMaxAngle = 0.0;

private double currentDeployMinAngle = 0.0;

private boolean isSetDeployPositionInitReady = true;


// =========================================================================
// Tuneables
// =========================================================================

private Timer rollerTimer = new Timer();

/**
 * Deploy goals:
 *
 * have a set max position and variable min positions (the
 * deploy will be able to go low enough to lift the bottom of
 * the robot up, and we probably don't want to do this
 * until we climb)
 *
 *
 *
 */


}
