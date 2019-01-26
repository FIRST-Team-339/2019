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

private Joystick leftOperator = null;

private Joystick rightOperator = null;

private boolean depositInit = false;


/**
 * constructor to use in hardware
 */
public GamePieceManipulator (SpeedController armMotor,
        RobotPotentiometer armPot, SpeedController armRollers,
        LightSensor photoSwitch, Joystick leftOperator,
        Joystick rightOperator)
{
    this.armMotor = armMotor;
    this.armPot = armPot;
    this.armRollers = armRollers;
    this.photoSwitch = photoSwitch;
    this.leftOperator = leftOperator;
    this.rightOperator = rightOperator;
}

/**
 * constructor to use in hardware
 */
public GamePieceManipulator (SpeedController armMotor,
        KilroyEncoder armEncoder, SpeedController armRollers,
        LightSensor photoSwitch, Joystick leftOperator,
        Joystick rightOperator)
{
    this.armMotor = armMotor;
    this.armEncoder = armEncoder;
    this.armRollers = armRollers;
    this.photoSwitch = photoSwitch;
    this.leftOperator = leftOperator;
    this.rightOperator = rightOperator;
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

public void moveArmByJoystick ()
{
    // if ((getCurrentArmPosition() < MAX_ARM_POSITION)
    // && (getCurrentArmPosition() > MIN_ARM_POSITION))
        {
        armMotor.set(rightOperator.getY());
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
public void spinRollers ()
{
    if (rightOperator.getTrigger() == true)
    // && photoSwitch.get() == false)
        {
        armRollers.set(INPUT_ROLLER_SPEED);
        } else if (rightOperator.getRawButton(2) == true)
        {
        armRollers.set(OUTPUT_ROLLER_SPEED);
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
        armRollers.set(OUTPUT_ROLLER_SPEED);
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


// =========================================================================
// Constants
// =========================================================================
private int MAX_ARM_POSITION = 170;

private int MIN_ARM_POSITION = 0;

private int RETRACTED_ARM_POSITION = 150;

private int GROUND_ARM_POSITION = 10;

private int ACCEPTABLE_ERROR = 6;

private double INPUT_ROLLER_SPEED = .6;

private double OUTPUT_ROLLER_SPEED = -.6;

private double LOWER_ARM_SPEED = -.3;

private double RAISE_ARM_SPEED = .5;

private double HOLD_ARM_SPEED = .2;

private double DEPOSIT_CARGO_TIME = 3;




// =========================================================================
// Tuneables
// =========================================================================

private Timer rollerTimer = new Timer();

private Timer depositTimer = new Timer();





}
