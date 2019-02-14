package frc.Utils;

import edu.wpi.first.wpilibj.SpeedController;
import frc.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class for all the methods for taking in and spitting out the cargo for
 * an intake mechanism that uses rollers, like the Nessie head.
 * Was turned into a seperate class from GamePieceManipuator to improve
 * overall code modularity in case in any changes were made to the
 * manipulator later in the season (i.e. if we switch to a different
 * intake mechanism, a seperate intake class from could be written with
 * identical methods and the intake object used by GamePieceManipulator
 * could just be replaced by an object
 *
 * I did not make a seperate Intake class or interface for this class to
 * extend just to avoid making it more confusing for individuals who are
 * not as used to Java or this project.
 *
 * @author Cole; 2019 build season
 */
public class RollerIntakeMechanism
{

private SpeedController armRollers = null;

private LightSensor photoSwitch = null;


private Timer outakeTimer = new Timer();

public RollerIntakeMechanism (SpeedController armRollers,
        LightSensor photoSwitch)
{
    this.armRollers = armRollers;
    this.photoSwitch = photoSwitch;
}




// TODO do we just want it so if you hit the override, even without pulling
// trigger, it intakes?
/**
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
public void intakeOuttakeByButtons (boolean intakeButtonValue,
        boolean reverseIntakeButtonValue,
        boolean intakeOverrideButtonValue)
{
    if (intakeButtonValue == true)
        {
        if (reverseIntakeButtonValue == true)
            {
            intakeState = IntakeState.OUTTAKE;
            }
        else
            if (intakeOverrideButtonValue == true
                    || this.hasCargo() == false)
                {
                intakeState = IntakeState.INTAKE;
                }
        }
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
    if (intakeButtonValue == true && (this.hasCargo() == false
            || intakeOverrideButtonValue == true))
        {
        intakeState = IntakeState.INTAKE;
        }
    else
        if (outtakeButtonValue == true)
            {
            intakeState = IntakeState.OUTTAKE;
            }
}

/**
 * Method used to autonomously spit out the cargo.
 * Should be called continously until it returns true
 * in order to ensure its initialization variable gets
 * reset properly.
 *
 * @return true if it has finished and spit out the cargo
 *         (enough time has passed), false otherwise
 */
public boolean spinOutCargoByTimer ()
{
    System.out.println(intakeState);
    System.out.println(depositInit);
    if (depositInit == false)
        {

        intakeState = IntakeState.OUTTAKE_BY_TIMER;
        return false;
        }
    if (depositInit == true
            && intakeState != IntakeState.OUTTAKE_BY_TIMER)
        {

        // depositInit = false; //TODO this no worky
        return true;
        }
    return false;
}



/** Returns whether or not the manipulator has cargo */
public boolean hasCargo ()
{
    return !this.photoSwitch.isOn();
}


/**
 * Resets the state machine so the intake does not keep trying to run
 * code from a previous enable after a disable. Should be called in teleop
 * init ONLY.
 */
public void resetStateMachine ()
{
    intakeState = IntakeState.HOLD;
}

/**
 * Update method for the intake/ outtake device and state machine on the
 * manipulator used for picking up cargo
 *
 * @author Cole; 2019 build season
 */
public void update ()
{

    switch (intakeState) // main state machine of intake
        {
        // sets the motors to bring in a cargo
        case INTAKE:
            armRollers.set(INTAKE_ROLLER_SPEED);
            // sets intakeState to HOLD so that if the intake
            // button is not being held, the intake will stop.
            // However, if the intake button is being held, the
            // state will go back to INTAKE before intakeUpdate is
            // called again
            intakeState = IntakeState.HOLD;
            break;

        // sets the motors to bring in a cargo
        case OUTTAKE:
            armRollers.set(OUTTAKE_ROLLER_SPEED);
            intakeState = IntakeState.HOLD;
            break;

        // used for autonamously pushing out the cube based on a timer
        case OUTTAKE_BY_TIMER:

            if (depositInit == false)
                {
                outakeTimer.reset();
                outakeTimer.start();
                // Sets depositInit to true so the timer is not
                // continually reset. depositInit is reset by the
                // the spinOutCargo method, which should be continually
                // called anyway if the OUTTAKE_BY_TIMER state is being,
                // since it should only be used if the carog is being
                // expelled autonomously
                depositInit = true;
                }

            if (outakeTimer.get() >= DEPOSIT_CARGO_TIME)
                {

                outakeTimer.stop();
                armRollers.set(HOLD_INTAKE_SPEED_NO_CARGO);
                intakeState = IntakeState.HOLD;
                }
            else
                {
                armRollers.set(OUTTAKE_ROLLER_SPEED);
                }
            break;
        default:
            // state to stop intake, or apply a small speed if necessary
            // to keep in cargo (it may not be, in which case the constant
            // passed to the motors is still 0.0)
        case HOLD:
            if (this.hasCargo() == true)
                {
                this.armRollers.set(HOLD_INTAKE_SPEED_WITH_CARGO);
                }
            else
                {
                this.armRollers.set(HOLD_INTAKE_SPEED_NO_CARGO);
                }
            break;
        }
}

// =========================================================================
// Constants
// =========================================================================

// speed sent to the intake to keep in the cargo
// if via testing we determine this is 0.0 on te new robot,
// we can probably change the HOLD sate in intakeUpdate to not use
// this anymore
private static final double HOLD_INTAKE_SPEED_WITH_CARGO = 0.0;

// speed given to the armRollers when not in use. Should be 0
private static final double HOLD_INTAKE_SPEED_NO_CARGO = 0.0;

private static final double DEPOSIT_CARGO_TIME = 2.0;

private static final double INTAKE_ROLLER_SPEED = -.6;

private static final double OUTTAKE_ROLLER_SPEED = .6;

// =========================================================================
// Enums
// =========================================================================

public static enum IntakeState
    {
    INTAKE, OUTTAKE, HOLD, OUTTAKE_BY_TIMER
    }

private IntakeState intakeState = IntakeState.HOLD;

// =========================================================================
// Variables
// =========================================================================

private boolean depositInit = false;
}
