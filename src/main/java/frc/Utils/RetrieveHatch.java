package frc.Utils;

import edu.wpi.first.wpilibj.Timer;
import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DriveWithCamera.DriveWithCameraState;

public class RetrieveHatch
{
public static enum RetrievalState
    {
    STANDBY, START_RETRIEVAL, DOUBLE_CHECK_POSITIONS, DELAY_ONE, DRIVE_FORWARD, DELAY_TWO, LIFT_FORKLIFT_TO_GET_HATCH, DELAY_THREE, BACKUP_SLOWLY, STOP, FINISH
    }

public static RetrievalState retrievalState = RetrievalState.STANDBY;

public static Timer delayTimer = new Timer();


public static void retrieveHatch ()
{
    retrievalState = RetrievalState.START_RETRIEVAL;

}


public void retrievalUpdate ()
{

    switch (retrievalState)
        {
        case STANDBY:

            break;

        case START_RETRIEVAL:
            // initiates the retrieval sequence
            retrievalState = RetrievalState.DOUBLE_CHECK_POSITIONS;
            break;

        case DOUBLE_CHECK_POSITIONS:
            // double checks that the forklift and the manipulator are in the
            // right positions and moves them to the right positions if theyre
            // not in the right positions
            if (forkliftHeightReached == false && Hardware.lift
                    .setLiftPosition(
                            LIFT_HEIGHT_TO_RETRIEVE_HATCH) == true)
                {
                forkliftHeightReached = true;
                }
            if (forkliftHeightReached == true && Hardware.manipulator
                    .moveArmToPosition(0) == true)
                {
                delayInit();
                forkliftHeightReached = false;
                retrievalState = RetrievalState.DELAY_ONE;
                break;
                }
            break;

        case DELAY_ONE:
            if (delayTimer.get() >= DELAY_ONE_TIME)
                {
                delayTimer.stop();
                retrievalState = RetrievalState.DRIVE_FORWARD;
                }
            break;

        case DRIVE_FORWARD:
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_DRIVE_FORWARD, SPEED_TO_DRIVE_FORWARD,
                    0.0, true))
                {
                delayInit();
                retrievalState = RetrievalState.DELAY_TWO;
                }
            break;

        case DELAY_TWO:
            if (delayTimer.get() >= DELAY_TWO_TIME)
                {
                delayTimer.stop();
                retrievalState = RetrievalState.LIFT_FORKLIFT_TO_GET_HATCH;
                }
            break;

        case LIFT_FORKLIFT_TO_GET_HATCH:
            if (Hardware.lift.setLiftPosition(
                    LIFT_HEIGHT_TO_PULL_BACK_HATCH) == true)
                {
                delayInit();
                retrievalState = RetrievalState.DELAY_THREE;
                }
            break;

        case DELAY_THREE:
            if (delayTimer.get() >= DELAY_THREE_TIME)
                {
                delayTimer.stop();
                retrievalState = RetrievalState.BACKUP_SLOWLY;
                }
            break;

        case BACKUP_SLOWLY:
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_BACKUP_SLOWLY, SPEED_TO_BACKUP_SLOWLY,
                    0, true))
                {
                retrievalState = RetrievalState.STOP;
                }
            break;

        case STOP:
            stop();
            retrievalState = RetrievalState.FINISH;
            break;

        case FINISH:
            retrievalState = RetrievalState.STANDBY;
            break;


        }

}


/**
 * resets and starts delay timer to use in delay states
 */
public void delayInit ()
{
    // resets and starts delayTimer
    delayTimer.reset();
    delayTimer.start();
}

public void stop ()
{
    Hardware.drive.stop();
    Hardware.lift.resetStateMachine();
    Hardware.manipulator.resetStateMachine();

}

public boolean alignWithVision (double speed)
{
    if (Hardware.frontUltraSonic
            .getDistanceFromNearestBumper() <= DISTANCE_TO_STOP_ALIGN
            || Hardware.driveWithCamera.driveToTargetClose(.1))
        {

        Hardware.alignAndStopButton.setValue(false);
        Hardware.driveWithCamera.state = DriveWithCameraState.INIT;
        return true;
        }

    return false;
}


// CONSTANTS AND MISC

public static double DELAY_ONE_TIME = 0.0;

public static double DELAY_TWO_TIME = 0.0;

public static double DELAY_THREE_TIME = 0.0;

// DISTANCES
public static double DISTANCE_TO_DRIVE_FORWARD = 11.0;

public static double LENGTH_OF_NESSIE_HEAD = 25.0;

public static double DISTANCE_TO_STOP_ALIGN = DISTANCE_TO_DRIVE_FORWARD
        + LENGTH_OF_NESSIE_HEAD;

public static double DISTANCE_TO_RETRIEVE = DISTANCE_TO_DRIVE_FORWARD
        + LENGTH_OF_NESSIE_HEAD;

public static double DISTANCE_TO_BACKUP_SLOWLY = 4.0;

// POSITIONS
public static double LIFT_HEIGHT_TO_RETRIEVE_HATCH = Forklift.PLAYER_STATION_HEIGHT;// lift
// height to go
// into the hatch

public static double LIFT_HEIGHT_TO_PULL_BACK_HATCH = LIFT_HEIGHT_TO_RETRIEVE_HATCH
        + 3.0;// height to get in the center of the hatch plus the height to
              // move up the lift to actually have the hatch resting on the hook

// SPEEDS
public static double SPEED_TO_BACKUP_SLOWLY = -0.25;

public static double SPEED_TO_DRIVE_FORWARD = 0.15;

// BOOLEANS

public static boolean forkliftHeightReached = false;

}
