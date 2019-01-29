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

private SpeedController armRollers = null;

private RobotPotentiometer armPot = null;

private KilroyEncoder armEncoder = null;

private LightSensor photoSwitch = null;

private boolean depositInit = false;


/**
 * constructor to use in hardware
 */
public GamePieceManipulator (SpeedController armMotor,
        RobotPotentiometer armPot, SpeedController armRollers,
        LightSensor photoSwitch)
{
    this.armMotor = armMotor;
    this.armPot = armPot;
    this.armRollers = armRollers;
    this.photoSwitch = photoSwitch;
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
    this.armRollers = armRollers;
    this.photoSwitch = photoSwitch;
}

public static enum GamePiece
    {
HATCH_PANEL, CARGO, NONE
    }

// placeholder, will need to do something
public static enum DeployState
    {

    }

// placeholder function since Forklift will need to understand which piece
// the manipulator has
public GamePiece hasWhichGamePiece ()
{
    return GamePiece.NONE;
}

// placeholder, will need to be changed
public boolean isDeployed ()
{
    return true;
}


// =========================================================================
// armMotor methods
// =========================================================================

/**
 * call during teleop to move the arm up and down based on the joystick controls
 */

public void moveArmByJoystick (Joystick armJoystick)
{
    // if ((getCurrentArmPosition() < MAX_ARM_POSITION)
    // && (getCurrentArmPosition() > MIN_ARM_POSITION))
        {
        armMotor.set(armJoystick.getY());
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



// =========================================================================
// roller methods
// =========================================================================


public void intakeOuttakeByButtons (boolean intakeButtonValue,
        boolean reverseIntakeButtonValue,
        boolean intakeOverrideButtonValue)
{
    if (intakeButtonValue == true)
        {
        if (reverseIntakeButtonValue == true)
            {
            intakeState = IntakeState.OUTTAKE;
            } else if (intakeOverrideButtonValue == true
                    || this.hasCube() == false)
            {
            intakeState = IntakeState.INTAKE;
            }
        }
}




/**
 * Method for calling intake and outtake when they are both mapped to two
 * different buttons. This is in contrast to the intakeOuttake method, which has
 * one button for
 *
 */
public void intakeOuttakeByButtonsSeperated (boolean intakeButtonValue,
        boolean outtakeButtonValue)
{
    if (intakeButtonValue == true)
    // && photoSwitch.get() == false)
        {
        armRollers.set(INTAKE_ROLLER_SPEED);
        } else if (outtakeButtonValue == true)
        {
        armRollers.set(OUTTAKE_ROLLER_SPEED);
        } else
        {
        armRollers.set(0.0);
        }
}

public boolean spinOutCargo ()
{
    if (depositInit == false)
        {
        depositTimer.reset();
        depositTimer.start();
        armRollers.set(OUTTAKE_ROLLER_SPEED);
        depositInit = true;
        }
    if (depositTimer.get() >= DEPOSIT_CARGO_TIME)
        {
        depositTimer.stop();
        armRollers.set(0.0);
        depositInit = false;
        return true;
        }
    return false;
}

public boolean depositHatch ()
{
    return false;
}

public boolean hasCube ()
{
    return photoSwitch.isOn();
}

public void intakeUpdate ()
{
    switch (intakeState)
        {
        case INTAKE:
            armRollers.set(INTAKE_ROLLER_SPEED);
            intakeState = IntakeState.HOLD;
            break;

        case OUTTAKE:
            armRollers.set(OUTTAKE_ROLLER_SPEED);
            intakeState = IntakeState.HOLD;
            break;

        default:
        case HOLD:
            if (this.hasCube() == true)
                {
                this.armRollers.set(HOLD_INTAKE_SPEED_WITH_CARGO);
                } else
                {
                this.armRollers.set(HOLD_INTAKE_SPEED_NO_CARGO);
                }
            break;
        }
}

// =========================================================================
// Constants
// =========================================================================

private static final double HOLD_INTAKE_SPEED_WITH_CARGO = 0.0;

private static final double HOLD_INTAKE_SPEED_NO_CARGO = 0.0;

private int MAX_ARM_POSITION = 170;

private int MIN_ARM_POSITION = 0;

private int RETRACTED_ARM_POSITION = 150;

private int GROUND_ARM_POSITION = 10;

private int ACCEPTABLE_ERROR = 6;

private double INTAKE_ROLLER_SPEED = .6;

private double OUTTAKE_ROLLER_SPEED = -.6;

private double LOWER_ARM_SPEED = -.3;

private double RAISE_ARM_SPEED = .5;

private double HOLD_ARM_SPEED = .2;

private double DEPOSIT_CARGO_TIME = 3;

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


// =========================================================================
// Tuneables
// =========================================================================

private Timer rollerTimer = new Timer();

private Timer depositTimer = new Timer();



}
