package frc.Utils;

import frc.Utils.drive.*;
import frc.Utils.GamePieceManipulator;


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
 *
 *
 */
public boolean depositHatch ()
{
    switch (depositHatchState)
        {
        case INIT:
            System.out.println("init deposit");
            if (this.gamePieceManipulator.isDeployed())
                {
                depositHatchState = DepositHatchState.DEPOSIT_HATCH;
                }
            else
                {
                if (this.gamePieceManipulator
                        .moveArmToPosition(LOWERED_ARM_POSITION,
                                ARM_MOVE_SPEED))
                    {
                    depositHatchState = DepositHatchState.DEPOSIT_HATCH;
                    }
                }
            break;

        case DEPOSIT_HATCH:
            System.out.println("deposit deposit");
            depositHatchState = DepositHatchState.BACKUP_HATCH;
            break;
        case LOWER_FORKLIFT_HATCH:
            if (this.gamePieceManipulator.moveArmToPosition(
                    LOWERED_ARM_AFTER_DEPOSIT_POSITION, ARM_MOVE_SPEED))
                {
                }
            break;
        case BACKUP_HATCH:
            System.out.println("back deposit");
            if (this.drive.driveStraightInches(BACKUP_INCHES,
                    BACKUP_SPEED, BACKUP_ACCELERATION, usingGyro))
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
    switch (depositCargoState)
        {
        case INIT:
            if (this.gamePieceManipulator.isDeployed())
                {

                depositCargoState = DepositCargoState.RAISE_MANIPULATOR;
                }
            else
                {
                if (this.gamePieceManipulator
                        .moveArmToPosition(CARGO_ARM_POSITION,
                                ARM_MOVE_SPEED))
                    {
                    depositCargoState = DepositCargoState.DEPOSIT_CARGO;
                    }
                }
            break;
        case RAISE_MANIPULATOR:
            if (this.gamePieceManipulator
                    .moveArmToPosition(CARGO_ARM_POSITION,
                            ARM_MOVE_SPEED))
                {
                depositCargoState = DepositCargoState.DEPOSIT_CARGO;
                }
            break;

        case DEPOSIT_CARGO:
            System.out.println("Depositing the cargo");
            depositHatchState = DepositHatchState.BACKUP_HATCH;
            break;
        case BACKUP_CARGO:
            if (this.drive.driveStraightInches(BACKUP_INCHES,
                    BACKUP_SPEED, BACKUP_ACCELERATION, usingGyro))
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
} // end depositCargo()


// Hatch constants======================

private static final int LOWERED_ARM_POSITION = 260;// TODO

private static final int LOWERED_ARM_AFTER_DEPOSIT_POSITION = 250;// TODO

private static final int RAISED_ARM_POSITION = 225;// TODO


// Cargo constants=========================
private static final int CARGO_ARM_POSITION = 245;// TODO magic number



// otro constants===========================
private static boolean usingGyro = true;

private static final double ARM_MOVE_SPEED = .4;

private static final double BACKUP_INCHES = -10;// TODO

private static final double BACKUP_ACCELERATION = .1;

private static final double BACKUP_SPEED = .4;


}
