package frc.Utils;

import frc.Utils.drive.*;
import frc.Hardware.Hardware;
import frc.HardwareInterfaces.MomentarySwitch;
import frc.Utils.GamePieceManipulator;
import frc.Utils.GamePieceManipulator.GamePiece;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.buttons.Button;

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
public boolean depositHatch ()
{

    System.out.println("depositHatchstate: " + depositHatchState);
    switch (depositHatchState)
        {
        case INIT:

            // if (this.gamePieceManipulator.isDeployed())
            // {
            // depositHatchState = DepositHatchState.DEPOSIT_HATCH;
            // }
            // else
            // {
            // if (this.gamePieceManipulator
            // .deployArm())
            // {
            // depositHatchState = DepositHatchState.DEPOSIT_HATCH;
            // }
            // }
            depositHatchState = DepositHatchState.DEPOSIT_HATCH;
            break;

        case DEPOSIT_HATCH:
            System.out.println("deposit deposit");
            if (this.drive.driveStraightInches(FORWARD_TO_DEPOSIT,
                    .4, BACKUP_ACCELERATION, usingGyro))
                {

                depositHatchState = DepositHatchState.BACKUP_HATCH;

                }
            break;

        case BACKUP_HATCH:
            System.out.println("back deposit");
            Hardware.manipulator.moveArmToPosition(DEPOSIT_ARM_ANGLE,
                    ARM_MOVE_SPEED);
            if (this.drive.driveStraightInches(BACKUP_INCHES,
                    -BACKUP_SPEED, BACKUP_ACCELERATION, usingGyro))
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
    System.out.println("depositCargostate: " + depositCargoState);
    switch (depositCargoState)
        {
        case INIT:
            if (this.gamePieceManipulator.isDeployed())
                {

                depositCargoState = DepositCargoState.RAISE_MANIPULATOR;
                }
            else
                {
                if (this.gamePieceManipulator.moveArmToPosition(45,
                        ARM_MOVE_SPEED))
                    {
                    depositCargoState = DepositCargoState.DEPOSIT_CARGO;
                    }
                }
            break;
        case RAISE_MANIPULATOR:
            // if (this.gamePieceManipulator
            // .moveArmToPosition(CARGO_ARM_POSITION,
            // ARM_MOVE_SPEED))
            // {
            // depositCargoState = DepositCargoState.DEPOSIT_CARGO;
            // }
            depositCargoState = DepositCargoState.DEPOSIT_CARGO;
            break;

        case DEPOSIT_CARGO:
            System.out.println("Depositing the cargo");

            if (this.gamePieceManipulator.spinOutCargoByTimer())
                {
                System.out.println(
                        "deposited");

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

public enum DepositHeight
    {
    CARGO_SHIP, ROCKET_HATCH_1, ROCKET_HATCH_2, ROCKET_HATCH_3, ROCKET_CARGO_1, ROCKET_CARGO_2, ROCKET_CARGO_3
    }

public enum HatchOrCargo
    {
    HATCH, CARGO, NULL
    }

public HatchOrCargo hatchOrCargo = HatchOrCargo.NULL;

public DepositHeight depositHeight = DepositHeight.ROCKET_CARGO_1;

public boolean depositTeleopStateMachine ()
{

    System.out.println("deposit teleop state: " + depositTeleopState);
    switch (depositTeleopState)
        {

        case HOLD:
            Hardware.axisCamera.setRelayValue(Value.kOff);

            break;
        case INIT:
            switch (depositHeight)
                {

                case ROCKET_HATCH_1:

                    forkliftHeight = Forklift.LOWER_ROCKET_HATCH;
                    break;
                }
            Hardware.axisCamera.setRelayValue(Value.kOn);

            depositTeleopState = DepositTeleopState.PREP;
            break;

        case PREP:
            Hardware.lift.setLiftPosition(forkliftHeight);

            if (hatchOrCargo == HatchOrCargo.HATCH)
                {
                // has a hatch
                if (Hardware.manipulator.moveArmToPosition(35/* 105 */,
                        .8)
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


                }

            break;

        case ALIGN_TO_TARGET:

            if (Hardware.driveWithCamera.driveToTargetClose(.3))
                {
                depositTeleopState = DepositTeleopState.DEPOSIT;
                }
            break;

        case DEPOSIT:
            if (hatchOrCargo == HatchOrCargo.HATCH)
                {
                if (this.depositHatch())
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

public boolean startTeleopDeposit (DepositHeight height,
        HatchOrCargo gamepiece)
{

    depositHeight = height;
    hatchOrCargo = gamepiece;
    if (hasStartedDeposit)
        {
        System.out
                .println("started teleopdeposit");
        hasStartedDeposit = true;
        depositTeleopState = DepositTeleopState.INIT;
        }

    if (depositTeleopState == DepositTeleopState.FINISH)
        {
        System.out
                .println("finished teleopdeposit");
        hasStartedDeposit = false;
        return true;
        }
    return false;
}

public void resetDepositTeleop ()
{
    System.out.println("reseting teleop");
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
        if (Hardware.manipulator.moveArmToPosition(35/* 105 */, .8)
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


public double forkliftHeight = Forklift.LOWER_ROCKET_HATCH;

// constants for prep

public static final double PREP_FOR_HATCH_MAX = 40/* 110 */;

public static final double PREP_FOR_HATCH_MIN = 30/* 100 */;

// Hatch constants======================





private static final int FORWARD_TO_DEPOSIT = 2;// TODO

private static final double DEPOSIT_ARM_ANGLE = 90;

// Cargo constants=========================




// otro constants===========================
private static boolean usingGyro = true;

private static final double ARM_MOVE_SPEED = .4;

private static final double BACKUP_INCHES = 10;// TODO

private static final double BACKUP_ACCELERATION = .1;

private static final double BACKUP_SPEED = .2;


}
