package frc.Utils;

import frc.Hardware.Hardware;
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
INIT, DEPOSIT_HATCH, BACKUP_HATCH, STOP
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
            if (this.gamePieceManipulator.isDeployed())
                {
                depositHatchState = DepositHatchState.DEPOSIT_HATCH;
                } else
                {
                this.gamePieceManipulator
                        .moveArmToPosition(LOWERED_ARM_POSITION);
                }
            break;

        case DEPOSIT_HATCH:
            depositHatchState = DepositHatchState.BACKUP_HATCH;
            break;
        case BACKUP_HATCH:
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

public boolean depositCargo ()
{


    return false;
}

private static boolean usingGyro = true;

private static final double BACKUP_INCHES = 6;// TODO

private static final double BACKUP_ACCELERATION = .1;

private static final double BACKUP_SPEED = .4;

private static final int LOWERED_ARM_POSITION = 260;

private static final int RAISED_ARM_POSITION = 225;





}
