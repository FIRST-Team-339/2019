package frc.Utils;

import frc.Utils.drive.*;
import frc.Hardware.Hardware;
import frc.Utils.GamePieceManipulator;
import edu.wpi.first.wpilibj.Relay.Value;


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
    INIT, DEPOSIT_HATCH, BACKUP_HATCH, LOWER_FORKLIFT_HATCH, STOP
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
            if (Hardware.manipulator.moveArmToPosition(
                    35/* 105 */,
                    FORK_SPEED)
                    || (Hardware.manipulator
                            .getCurrentArmPosition() > PREP_FOR_HATCH_MIN
                            && Hardware.manipulator
                                    .getCurrentArmPosition() < PREP_FOR_HATCH_MAX))

                depositHatchState = DepositHatchState.DEPOSIT_HATCH;
            break;

        case DEPOSIT_HATCH:
            if (this.drive.driveStraightInches(FORWARD_TO_DEPOSIT,
                    .4, BACKUP_ACCELERATION, usingGyro))
                {

                depositHatchState = DepositHatchState.BACKUP_HATCH;

                }
            break;

        case BACKUP_HATCH:
            if (inAuto)
                {
                Hardware.manipulator.moveArmToPosition(
                        DEPOSIT_ARM_ANGLE,
                        ARM_MOVE_SPEED);
                if (this.drive.driveStraightInches(BACKUP_INCHES,
                        -BACKUP_SPEED, BACKUP_ACCELERATION, usingGyro))
                    {
                    depositHatchState = DepositHatchState.STOP;
                    }
                }
            else
                if (depositHeighthatch == 3)
                    {
                    if (Hardware.manipulator.moveArmToPosition(
                            Hardware.manipulator.getCurrentArmPosition()
                                    - 10,
                            ARM_MOVE_SPEED))
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
                            Hardware.lift.getForkliftHeight() - 3, 1))
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
            if (this.gamePieceManipulator
                    .moveArmToPosition(CARGO_ARM_POSITION,
                            ARM_MOVE_SPEED)
                    || true)
                {
                depositCargoState = DepositCargoState.DEPOSIT_CARGO;
                }
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
    INIT, HOLD, PREP, ALIGN_TO_TARGET, DEPOSIT, FINISH
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

            Hardware.axisCamera.setRelayValue(Value.kOff);

            break;


        case INIT:

            if (hasCargo == false)
                {
                switch (depositHeightCargo)
                    {

                    case 0:

                        forkliftHeight = Forklift.LOWER_ROCKET_HATCH;
                        break;

                    case 1:

                        forkliftHeight = Forklift.MIDDLE_ROCKET_HATCH;
                        break;

                    case 2:

                        forkliftHeight = Forklift.TOP_ROCKET_HATCH;
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

                        forkliftHeight = Forklift.TOP_ROCKET_HATCH;
                        break;

                    case 1:

                        forkliftHeight = Forklift.LOWER_ROCKET_HATCH;
                        break;
                    case 2:

                        forkliftHeight = Forklift.MIDDLE_ROCKET_HATCH;
                        break;
                    case 3:

                        forkliftHeight = Forklift.CARGO_SHIP_CARGO;
                        break;
                    }
                }
            Hardware.axisCamera.setRelayValue(Value.kOn);

            depositTeleopState = DepositTeleopState.PREP;
            break;

        case PREP:
            if (Hardware.lift.setLiftPosition(
                    forkliftHeight, 1))
                {

                if (hasCargo == false)
                    {
                    // has a hatch
                    if (Hardware.manipulator.moveArmToPosition(
                            105,
                            1)
                            || (Hardware.manipulator
                                    .getCurrentArmPosition() > PREP_FOR_HATCH_MIN
                                    && Hardware.manipulator
                                            .getCurrentArmPosition() < PREP_FOR_HATCH_MAX))
                        {
                        depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                        }
                    }
                // has cargo
                else
                    {
                    if (Hardware.manipulator.moveArmToPosition(
                            CARGO_ARM_POSITION,
                            1)
                            || (Hardware.manipulator
                                    .getCurrentArmPosition() > CARGO_ARM_POSITION
                                            + 3
                                    && Hardware.manipulator
                                            .getCurrentArmPosition() < CARGO_ARM_POSITION
                                                    - 3))
                        {
                        depositTeleopState = DepositTeleopState.ALIGN_TO_TARGET;
                        }

                    }
                }
            break;

        case ALIGN_TO_TARGET:

            if (true /* Hardware.driveWithCamera.driveToTargetClose(.25) */)
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

            depositTeleopState = DepositTeleopState.HOLD;
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

    if (depositTeleopState == DepositTeleopState.HOLD)
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
    hasStartedDeposit = false;
    depositTeleopState = DepositTeleopState.HOLD;
}

public static boolean hasDoneThePrep = false;






/**
 * function to back up and raise arm to deposit in autonomous. This will only
 * wok with the hatch panel
 */
public void prepToDepositHatch ()
{
    if (hasDoneThePrep == false)
        {
        System.out.println("*Dabs on haters*");
        if (Hardware.manipulator.moveArmToPosition(105, .8)
                || (Hardware.manipulator
                        .getCurrentArmPosition() > PREP_FOR_HATCH_MIN
                        && Hardware.manipulator
                                .getCurrentArmPosition() < PREP_FOR_HATCH_MAX))
            {
            System.out.println("*Hater has been dabbed on*");
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
        depositTeleopState = DepositTeleopState.FINISH;
        return true;
        }

    return false;
}

public double forkliftHeight = Forklift.LOWER_ROCKET_HATCH;

// constants for prep

public static final double PREP_FOR_HATCH_MAX = 110;

public static final double PREP_FOR_HATCH_MIN = 100;

// Hatch constants======================


private static final int FORWARD_TO_DEPOSIT = 2;// TODO

private static final double DEPOSIT_ARM_ANGLE = 90;

// Cargo constants=========================


private static final double CARGO_ARM_POSITION = 90;


// otro constants===========================

private static final double JOYSTICK_DEADBAND = .1;

private static boolean usingGyro = true;

private static final double ARM_MOVE_SPEED = .6;

private static final double BACKUP_INCHES = 10;// TODO

private static final double BACKUP_ACCELERATION = .1;

private static final double BACKUP_SPEED = .3;

private static final double FORK_SPEED = 1;


}
