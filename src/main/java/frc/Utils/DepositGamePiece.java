package frc.Utils;

import frc.Utils.drive.*;
import frc.Utils.GamePieceManipulator;
import edu.wpi.first.wpilibj.Timer;


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

public Timer outakeTimer = new Timer();

public enum DepositHatchState
    {
    INIT, DEPOSIT_HATCH, BACKUP_HATCH, LOWER_FORKLIFT_HATCH, STOP
    }

public static DepositHatchState depositHatchState = DepositHatchState.INIT;

/**
 *
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

                depositHatchState = DepositHatchState.LOWER_FORKLIFT_HATCH;

                }
            break;
        case LOWER_FORKLIFT_HATCH:
            if (this.forklift.setLiftPosition(
                    this.forklift.getForkliftHeight() - 4))
                {
                depositHatchState = DepositHatchState.BACKUP_HATCH;
                }
            break;
        case BACKUP_HATCH:
            System.out.println("back deposit");
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



public boolean outakeGamepiece ()
{


    return true;
}


// Hatch constants======================



private static final int LOWERED_ARM_AFTER_DEPOSIT_POSITION = 250;// TODO

private static final int FORWARD_TO_DEPOSIT = 2;// TODO


// Cargo constants=========================




// otro constants===========================
private static boolean usingGyro = true;

private static final double ARM_MOVE_SPEED = .4;

private static final double BACKUP_INCHES = 10;// TODO

private static final double BACKUP_ACCELERATION = .1;

private static final double BACKUP_SPEED = .4;


}
