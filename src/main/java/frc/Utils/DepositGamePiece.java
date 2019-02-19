package frc.Utils;

import frc.Utils.drive.*;
import frc.Hardware.Hardware;
import frc.Utils.GamePieceManipulator;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * @author Conner McKevitt
 *
 *         use to deposit a game piece for the 2019 season
 */
public class DepositGamePiece
{

public Drive drive = null;

public Forklift forklift = null;

public GamePieceManipulator gamePieceManipulator = null;

public DepositGamePiece (Drive drive, Forklift forklift,
        GamePieceManipulator gamePieceManipulator)
{
    this.drive = drive;
    this.forklift = forklift;
    this.gamePieceManipulator = gamePieceManipulator;


}



public enum DepositHatchState
    {
    INIT, DEPOSIT_HATCH, BACKUP_HATCH, BACKUP_HATCH_AFTER_FORK, STOP
    }

public static DepositHatchState depositHatchState = DepositHatchState.INIT;

/**
 * a statemachine that deposits and hatch. The forklift must already be at the
 * correct height and the manipulator at the correct angle. For auto use
 * prepToDepost to set up for a rocket hatch
 *
 */
public boolean depositHatch (boolean inAuto)
{


    switch (depositHatchState)
        {


        case INIT:

            depositHatchState = DepositHatchState.DEPOSIT_HATCH;

            break;

        case DEPOSIT_HATCH:
            if (this.drive.driveStraightInches(FORWARD_TO_DEPOSIT,
                    .2, BACKUP_ACCELERATION, usingGyro))
                {

                depositHatchState = DepositHatchState.BACKUP_HATCH;

                }
            break;

        case BACKUP_HATCH:
            System.out.println("in auto: " + inAuto);
            if (inAuto)
                {
                Hardware.manipulator.moveArmToPosition(
                        DEPOSIT_ARM_ANGLE_AUTO);
                if (this.drive.driveStraightInches(BACKUP_INCHES,
                        -BACKUP_SPEED, BACKUP_ACCELERATION, usingGyro))
                    {
                    depositHatchState = DepositHatchState.STOP;
                    }
                }
            else
                if (depositHeighthatch == 2)
                    {
                    System.out.println("hadsjoafno");
                    if (Hardware.manipulator.moveArmToPosition(
                            Hardware.manipulator.getCurrentArmPosition()
                                    - 10))
                        {
                        if (this.drive.driveStraightInches(
                                BACKUP_INCHES,
                                -BACKUP_SPEED, BACKUP_ACCELERATION,
                                usingGyro))
                            {
                            depositHatchState = DepositHatchState.STOP;
                            }
                        }
                    }
                else
                    {
                    if (Hardware.lift.setLiftPosition(
                            Hardware.lift.getForkliftHeight() - 3,
                            FORK_SPEED))
                        {
                        depositHatchState = DepositHatchState.BACKUP_HATCH_AFTER_FORK;
                        }
                    }
            break;

        case BACKUP_HATCH_AFTER_FORK:
            if (this.drive.driveStraightInches(
                    BACKUP_INCHES,
                    -BACKUP_SPEED, BACKUP_ACCELERATION,
                    usingGyro))
                {
                depositHatchState = DepositHatchState.STOP;
                }
            break;
        case STOP:
            this.drive.drive(0, 0);

            depositHatchState = DepositHatchState.INIT;
            return true;
        }
    return false;
}

public enum DepositCargoState
    {
    INIT, RAISE_MANIPULATOR, DEPOSIT_CARGO, BACKUP_CARGO, STOP
    }

public static DepositCargoState depositCargoState = DepositCargoState.INIT;

/**
 * use this to deposit a cargo gamepiece. The forklift and manipulator must
 * already be set to the proper height and angle.
 *
 * @return
 */
public boolean depositCargo ()
{
    switch (depositCargoState)
        {
        case INIT:
            depositCargoState = DepositCargoState.RAISE_MANIPULATOR;
            break;
        case RAISE_MANIPULATOR:
            // if (this.gamePieceManipulator
            // .moveArmToPosition(CARGO_ARM_POSITION))
            // {
            depositCargoState = DepositCargoState.DEPOSIT_CARGO;
            // }
            break;
        case DEPOSIT_CARGO:
            if (this.gamePieceManipulator.spinOutCargoByTimer())
                {
                depositCargoState = DepositCargoState.BACKUP_CARGO;
                }
            break;
        case BACKUP_CARGO:
            if (this.drive.driveStraightInches(BACKUP_INCHES,
                    -BACKUP_SPEED, BACKUP_ACCELERATION, usingGyro))
                {
                depositCargoState = DepositCargoState.STOP;
                }
            break;
        case STOP:
            this.drive.drive(0, 0);

            depositCargoState = DepositCargoState.INIT;
            return true;
        }
    return false;
} // end depositCargo()

private enum DepositTeleopState
    {
    INIT, HOLD, PREP_FORKLIFT, PREP_MANIPULATOR, ALIGN_TO_TARGET, DEPOSIT, FINISH
    }

public DepositTeleopState depositTeleopState = DepositTeleopState.INIT;

public enum DepositHeightHatch
    {
    CARGO_SHIP_HATCH, ROCKET_HATCH_1, ROCKET_HATCH_2, ROCKET_HATCH_3
    }

public enum DepositHeightCargo
    {
    CARGO_SHIP_CARGO, ROCKET_CARGO_1, ROCKET_CARGO_2, ROCKET_CARGO_3
    }



// default is hatch
public boolean hasCargo = false;

public int depositHeighthatch = 0;

public int depositHeightCargo = 0;


/**
 * This is the main state machine for depositing the gamepieces in Teleop. To
 * deposit you should call startTeleopDeposit while calling this function in the
 * teleop loop
 *
 */
public boolean depositTeleopStateMachine ()
{

    switch (depositTeleopState)
        {

        case HOLD:



            break;


        case INIT:

            if (hasCargo == false)
                {
                switch (depositHeighthatch)
                    {

                    case 0:
                        // System.out.print("0");
                        forkliftHeight = Forklift.LOWER_ROCKET_HATCH;
                        break;

                    case 1:
                        // System.out.print("1");
                        forkliftHeight = Forklift.MIDDLE_ROCKET_HATCH;
                        break;

                    case 2:
                        // System.out.print("2");
                        forkliftHeight = Forklift.TOP_ROCKET_HATCH;
                        break;

                    case 3:
                        // System.out.print("3");
                        forkliftHeight = Forklift.CARGO_SHIP_HATCH;
                        break;
                    }
                }
            else
                {
                switch (depositHeightCargo)
                    {
                    case 0:

                        forkliftHeight = Forklift.LOWER_ROCKET_CARGO;
                        break;
                    case 1:

                        forkliftHeight = Forklift.MIDDLE_ROCKET_CARGO;
                        break;

                    case 2:

                        forkliftHeight = Forklift.TOP_ROCKET_CARGO;
                        break;
                    case 3:

                        forkliftHeight = Forklift.CARGO_SHIP_CARGO;
                        break;
                    }
                }
            Hardware.axisCamera.setRelayValue(Value.kOn);

            depositTeleopState = DepositTeleopState.PREP_FORKLIFT;
            break;

        case PREP_FORKLIFT:

            if (Hardware.lift.setLiftPosition(
                    forkliftHeight, FORK_SPEED))
                {

                depositTeleopState = DepositTeleopState.PREP_MANIPULATOR;
                }
            break;

        case PREP_MANIPULATOR:

            if (hasCargo == false)
                {
                switch (depositHeighthatch)
                    {

                    case 0:


                        if (Hardware.manipulator.moveArmToPosition(
                                Forklift.LOWER_ROCKET_HATCH_ANGLE))
                            {
                            depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                            }

                        break;

                    case 1:

                        if (Hardware.manipulator.moveArmToPosition(
                                Forklift.MIDDLE_ROCKET_HATCH_ANGLE))
                            {
                            depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                            }
                        break;

                    case 2:
                        if (Hardware.manipulator.moveArmToPosition(
                                Forklift.TOP_ROCKET_HATCH_ANGLE))
                            {
                            depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                            }
                        break;

                    case 3:

                        forkliftHeight = Forklift.CARGO_SHIP_HATCH;
                        break;
                    }
                }
            else
                {
                switch (depositHeightCargo)
                    {
                    case 0:
                        if (Hardware.manipulator.moveArmToPosition(
                                Forklift.LOWER_ROCKET_CARGO_ANGLE))
                            {
                            depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                            }
                        break;
                    case 1:

                        if (Hardware.manipulator.moveArmToPosition(
                                Forklift.MIDDLE_ROCKET_CARGO_ANGLE))
                            {
                            depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                            }
                        break;

                    case 2:

                        if (Hardware.manipulator.moveArmToPosition(
                                Forklift.TOP_ROCKET_CARGO_ANGLE))
                            {
                            depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                            }
                        break;
                    case 3:

                        forkliftHeight = Forklift.CARGO_SHIP_CARGO;
                        break;
                    }
                }
            break;
        case ALIGN_TO_TARGET:

            if (Hardware.driveWithCamera.driveToTargetClose(.1)
                    || (Hardware.frontUltraSonic
                            .getDistanceFromNearestBumper() <= 22
                            && Hardware.rightFrontDriveEncoder
                                    .getDistance() > 10))
                {
                depositTeleopState = DepositTeleopState.DEPOSIT;
                }
            break;

        case DEPOSIT:
            if (hasCargo == false)
                {
                if (this.depositHatch(false))
                    {

                    depositTeleopState = DepositTeleopState.FINISH;
                    }
                }
            else
                {
                if (this.depositCargo())
                    {

                    depositTeleopState = DepositTeleopState.FINISH;
                    }
                }
            break;
        default:
        case FINISH:
            this.resetDepositTeleop();
            return true;
        }

    return false;
}

boolean hasStartedDeposit = false;

/**
 * use to start the deposit in teleop statemachine
 *
 * @param heightLevel
 *                        the level on the rocket that you want to deposit to.
 *                        0 = lowest, 1 = middle, 2 = high, 3 = cargo ship
 * @param gamepiece
 *                        the type of gamepiece in the manipulator
 *
 */
public boolean startTeleopDeposit (int heightLevel, boolean hasCargo)
{

    if (hasCargo == false)
        {
        depositHeighthatch = heightLevel;
        }
    else
        {
        depositHeightCargo = heightLevel;
        }
    if (!hasStartedDeposit)
        {
        System.out
                .println("Stop. It's Conner Time");
        hasStartedDeposit = true;
        depositTeleopState = DepositTeleopState.INIT;
        }

    if (depositTeleopState == DepositTeleopState.FINISH)
        {
        hasStartedDeposit = false;
        return true;
        }
    return false;
}


/**
 * Thes set the depositTeleop state machine to it holding state in preparation
 * for depositing
 *
 */
public void resetDepositTeleop ()
{
    Hardware.alignVisionButton.setValue(false);
    hasStartedDeposit = false;
    depositTeleopState = DepositTeleopState.HOLD;
    Hardware.drive.resetEncoders();

}



public static boolean hasDoneThePrep = false;

/**
 * function to back up and raise arm to deposit in autonomous. This will only
 * work with the hatch panel
 */
public void prepToDepositHatch ()
{
    if (hasDoneThePrep == false)
        {
        // System.out.println("*Dabs on haters*");
        if (Hardware.manipulator
                .moveArmToPosition(105))
            {
            // System.out.println("*Hater has been dabbed on*");
            hasDoneThePrep = true;

            }
        }
} // end prepToDeposit()

/**
 * This checks to see if any joysticks are being used. To stop the deposit the
 * drivers still have to toggle the button.
 *
 * @return
 */
public boolean overrideVision ()

{
    if (Hardware.leftOperator.getY() > JOYSTICK_DEADBAND
            || Hardware.leftOperator.getY() < -JOYSTICK_DEADBAND
            || Hardware.rightOperator.getY() > JOYSTICK_DEADBAND
            || Hardware.rightOperator.getY() < -JOYSTICK_DEADBAND
            || Hardware.leftDriver.getY() > JOYSTICK_DEADBAND
            || Hardware.leftDriver.getY() < -JOYSTICK_DEADBAND
            || Hardware.rightDriver.getY() > JOYSTICK_DEADBAND
            || Hardware.rightDriver.getY() < -JOYSTICK_DEADBAND)
        {
        // System.out.println("Mission Failed. We'll get'em next time");
        depositTeleopState = DepositTeleopState.FINISH;
        return true;
        }

    return false;
}

public void printDebugStatements ()
{

    SmartDashboard.putString("deposit teleop",
            this.depositTeleopState.toString());

    SmartDashboard.putBoolean("hasDoneThePrep",
            hasDoneThePrep);
    SmartDashboard.putBoolean("deposit with vision enabled",
            Hardware.alignVisionButton.isOnCheckNow());

    SmartDashboard.putNumber("forklift height",
            forkliftHeight);

    SmartDashboard.putBoolean("has cargo",
            hasCargo);
}

//
public double forkliftHeight = 0;

// constants for prep

public static final double PREP_FOR_HATCH_MAX = 110;

public static final double PREP_FOR_HATCH_MIN = 100;

// Hatch constants======================

private static final int FORWARD_TO_DEPOSIT = 4;

private static final double DEPOSIT_ARM_ANGLE_AUTO = 90;

// otro constants===========================

private static final double JOYSTICK_DEADBAND = .2;

private static boolean usingGyro = true;

private static final double BACKUP_INCHES = 10;

private static final double BACKUP_ACCELERATION = .1;

private static final double BACKUP_SPEED = .3;

private static final double FORK_SPEED = 1;




}
