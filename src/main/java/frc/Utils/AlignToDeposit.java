package frc.Utils;

import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DriveWithCamera;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.UltraSonic;
import frc.Utils.drive.Drive;

public class AlignToDeposit {

    public DriveWithCamera driveWithCamera = null;

    public Drive drive = null;

    public UltraSonic frontUltraSonic = null;

    public KilroyEncoder leftFrontDriveEncoder = null;

    public KilroyEncoder rightFrontDriveEncoder = null;

    public AlignToDeposit() {
        this.driveWithCamera = null;
        this.drive = null;
        this.frontUltraSonic = null;
        this.frontUltraSonic = null;
        this.leftFrontDriveEncoder = null;
        this.rightFrontDriveEncoder = null;
    }

    public AlignToDeposit(DriveWithCamera driveWithCamera, Drive drive, UltraSonic frontUltraSonic) {
        this.driveWithCamera = driveWithCamera;
        this.drive = drive;
        this.frontUltraSonic = frontUltraSonic;

    }

    public AlignToDeposit(DriveWithCamera driveWithCamera, Drive drive, UltraSonic frontUltraSonic,
            KilroyEncoder leftFrontDriveEncoder, KilroyEncoder rightFrontDriveEncoder) {
        this.driveWithCamera = driveWithCamera;
        this.drive = drive;
        this.frontUltraSonic = frontUltraSonic;
        this.leftFrontDriveEncoder = leftFrontDriveEncoder;
        this.rightFrontDriveEncoder = rightFrontDriveEncoder;

    }

    public boolean alignToDeposit(int override) {

        switch (state) {
        case INIT:
            if (USING_CAMERA) {
                state = AlignToDepositState.ALIGN_BY_CAMERA;
            } else {
                state = AlignToDepositState.ALIGN_BY_TAPE;
            }
            break;
        case ALIGN_BY_CAMERA:
            if (this.driveWithCamera.driveToTarget(ALIGN_BY_CAMERA_SPEED)) {
                state = AlignToDepositState.ALIGN_BY_TAPE;
            }
            break;
        case ALIGN_BY_TAPE:
            // alignByTape();
            state = AlignToDepositState.LIFT_MANIPULATOR;
            break;
        case LIFT_MANIPULATOR:
            // LiftArm();
            state = AlignToDepositState.DRIVE;
            break;
        case DRIVE:
            double distance = 0;// TODO make calcs

            this.drive.driveStraightInches(distance, .2, .1, true);
            state = AlignToDepositState.SCORE;
            break;
        case SCORE:
            break;

        case STOP:
            break;
        }

        return false;
    }

    AlignToDepositState state = AlignToDepositState.INIT;

    public enum AlignToDepositState {
        INIT, ALIGN_BY_CAMERA, ALIGN_BY_TAPE, LIFT_MANIPULATOR, DRIVE, SCORE, STOP
    }

    private final double ALIGN_BY_CAMERA_SPEED = .3;

    private final boolean USING_CAMERA = true;

    private final double DISTANCE_AWAY_FROM_DEPOSIT = 10; // TODO

}