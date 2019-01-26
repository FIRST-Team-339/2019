/*
 * outline created for potential use in teleop assistance
 *
 * note to self: add override button to constructors
 */

package frc.Utils;

import frc.HardwareInterfaces.DriveWithCamera;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.UltraSonic;
import frc.Utils.drive.Drive;

public class AlignToDeposit
{

public DriveWithCamera driveWithCamera = null;

public Drive drive = null;

public UltraSonic frontUltraSonic = null;

public KilroyEncoder leftFrontDriveEncoder = null;

public KilroyEncoder rightFrontDriveEncoder = null;

public AlignToDeposit ()
{
    this.driveWithCamera = null;
    this.drive = null;
    this.frontUltraSonic = null;
    this.frontUltraSonic = null;
    this.leftFrontDriveEncoder = null;
    this.rightFrontDriveEncoder = null;
}

public AlignToDeposit (DriveWithCamera driveWithCamera, Drive drive,
        UltraSonic frontUltraSonic)
{
    this.driveWithCamera = driveWithCamera;
    this.drive = drive;
    this.frontUltraSonic = frontUltraSonic;

}

public boolean alignToDeposit (int override, boolean usingCamera)
{
    switch (state)
        {
        case INIT:
            if (usingCamera)
                {
                state = AlignToDepositState.ALIGN_BY_CAMERA;
                } else
                {
                state = AlignToDepositState.ALIGN_BY_TAPE;
                }
            break;
        case ALIGN_BY_CAMERA:
            if (this.driveWithCamera
                    .driveToTarget(ALIGN_BY_CAMERA_SPEED))
                {
                state = AlignToDepositState.ALIGN_BY_TAPE;
                }
            break;
        case ALIGN_BY_TAPE:
            // AlignWithTape();
            state = AlignToDepositState.STOP;
            break;

        case STOP:
            // set stuff to stop

            break;
        }

    return false;
}

AlignToDepositState state = AlignToDepositState.INIT;

public enum AlignToDepositState
    {
INIT, ALIGN_BY_CAMERA, ALIGN_BY_TAPE, STOP
    }

private final double ALIGN_BY_CAMERA_SPEED = .3;



private final double DISTANCE_AWAY_FROM_DEPOSIT = 10; // TODO

}
