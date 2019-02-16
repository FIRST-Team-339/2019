package frc.Utils;

import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.SingleSolenoid;
import frc.HardwareInterfaces.UltraSonic;
import frc.Utils.drive.Drive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class ClimbToLevelTwo
{

private DoubleSolenoid driveSolenoid = null;

private SpeedController armMotor = null;

private KilroyEncoder armEncoder = null;

private RobotPotentiometer armSensor = null;

// private DoubleSolenoid testSolenoid = null;

private Drive drive = null;

private Forklift lift = null;

private UltraSonic ultraSonic = null;

private Timer climbTimer = new Timer();

// state of the climb state machine
private static enum climberState
    {
    STANDBY, START_CLIMB, LOWER_FORKLIFT_TO_POSITION, DELAY_ONE, LOWER_ARM, DELAY_TWO, DEPLOY_BACK_WHEELS, DELAY_THREE, LOWER_FORKLIFT_COMPLETELY, DELAY_FOUR, DRIVE_FORWARD, DELAY_FIVE, RAISE_ARM, DELAY_SIX, RETRACT_WHEELS, DELAY_SEVEN, FINISH_DRIVING, DELAY_INIT, STOP
    }


// initializes climb state and prev climb state to standby
private static climberState climbState = climberState.STANDBY;

private static climberState prevState = climberState.STANDBY;


/**
 * default construstor
 */
public ClimbToLevelTwo ()
{
    this.driveSolenoid = null;
    this.armMotor = null;
    this.armSensor = null;
    this.armEncoder = null;
    this.drive = null;
    this.lift = null;
    this.ultraSonic = null;
}

/**
 * Constructor for testing on Frogger, swaps out the single solenoid for a test
 * double solenoid
 *
 * @author Ashley Espeland
 * @param -
 *            testSolenoid
 *            solenoid to simulate the drive solenoids to lower the back wheels
 *
 * @param-armMotor
 *                 motor controller that controls the nessie head's movements
 *                 about a point
 *
 * @param-armEncoder
 *                   encoder that stands in for the armPot that will be on the
 *                   new robot
 * @param-drive
 *              drive object that obviously drves and deals with all that
 * @param-lift
 *             Forklift object that controlls the forklift's up and down motions
 * @param- ultraSonic
 *         // * Ultrasonic that is mounted on the front of the robot and used to
 *         tell
 *         // * us when to stop based on how far away from the wall we are
 *         // *
 *         //
 */
// public ClimbToLevelTwo (DoubleSolenoid testSolenoid,
// SpeedController armMotor, RobotPotentiometer armSensor,
// Drive drive,
// Forklift lift, UltraSonic ultraSonic)
// {

// this.testSolenoid = testSolenoid;
// this.armMotor = armMotor;
// this.armSensor = armSensor;
// this.drive = drive;
// this.lift = lift;
// this.ultraSonic = ultraSonic;
// }

/**
 * Constructor for the new robot- has armPot and single solenoid
 *
 * @author Ashley Espeland
 * @param -
 *            driveSolenoid
 *            single solenoid to lower and raise the back wheels
 *
 * @param-armMotor
 *                 motor controller that controls the nessie head's movements
 *                 about a point
 *
 * @param-armPot
 *               potentiometer that reads the nessie head's position
 *
 * @param-drive
 *              drive object that obviously drves and deals with all that
 *
 * @param-lift
 *             Forklift object that controlls the forklift's up and down motions
 *
 * @param- ultraSonic
 *         Ultrasonic that is mounted on the front of the robot and used to tell
 *         us when to stop based on how far away from the wall we are
 *
 */
public ClimbToLevelTwo (DoubleSolenoid driveSolenoid,
        SpeedController armMotor, RobotPotentiometer armSensor,
        Drive drive, Forklift lift, UltraSonic ultraSonic)
{

    this.driveSolenoid = driveSolenoid;
    this.armMotor = armMotor;
    this.armSensor = armSensor;
    this.drive = drive;
    this.lift = lift;
    this.ultraSonic = ultraSonic;
}


// method that deals with the states of climbing
// should be called wherever climb() or reverseClimb() is used otherwise they
// will not work
public void climbUpdate ()
{
    // System.out.println(climbState);

    switch (climbState)
        {
        case STANDBY:
            // state to wait in during the match until climb() or reverseClimb()
            // is called
            // testSolenoid.set(Value.kForward);
            break;

        case START_CLIMB:
            // state the climb() function sets the state to and is basically an
            // init state
            climbState = climberState.LOWER_FORKLIFT_TO_POSITION;
            break;

        case LOWER_FORKLIFT_TO_POSITION:
            // lowers forklift to the first position needed to climb, not all
            // the way down but mostly dwn
            if (this.lowerForkliftToPosition() == true)
                {
                // moves on to delay init and then DELAY_ONE
                prevState = climberState.LOWER_FORKLIFT_TO_POSITION;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_ONE:
            // first delay state after LOWER_FORKLIFT_TO_POSITION and before
            // LOWER_ARM
            if (climbTimer.get() >= DELAY_ONE_TIME)
                {
                // sets state to LOWER_ARM
                climbTimer.stop();
                prevState = climberState.DELAY_ONE;
                // Hardware.intakeDeploySensor.reset();
                climbState = climberState.LOWER_ARM;
                }
            break;

        case LOWER_ARM:
            // lowers nessie head all the way
            if (this.lowerArm() == true)
                {
                // goes to DELAY_TWO after DELAY_INIT
                prevState = climberState.LOWER_ARM;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_TWO:
            // delay state after LOWER_ARM and before DEPLOY_BAK_WHEELS
            if (climbTimer.get() >= DELAY_TWO_TIME)
                {
                // sets state to DEPLOY_BACK_WHEELS
                climbTimer.stop();
                prevState = climberState.DELAY_TWO;
                climbState = climberState.DEPLOY_BACK_WHEELS;
                }
            break;

        case DEPLOY_BACK_WHEELS:
            // deploys back wheels
            if (this.deployBackWheels() == true)
                {
                // Goes to DELAY_THREE
                prevState = climberState.DEPLOY_BACK_WHEELS;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_THREE:
            // delay state after DEPLOY_BACK_WHEELS and before
            // LOWER_FORKLFT_COMPLETELY
            if (climbTimer.get() >= DELAY_THREE_TIME)
                {
                // goes to LOWER_FORKLIFT_COMPLETELY
                climbTimer.stop();
                prevState = climberState.DELAY_THREE;
                climbState = climberState.LOWER_FORKLIFT_COMPLETELY;
                }
            break;

        case LOWER_FORKLIFT_COMPLETELY:
            // lowers forklift all the way and lifts the front end of the robot
            // subsequentially
            if (this.lowerForkliftCompletely() == true)
                {
                // goes to delay four
                prevState = climberState.LOWER_FORKLIFT_COMPLETELY;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_FOUR:
            // delay after LOWER_FORKLIFT_COMPLETELY and before DRIVE_FORWARD
            if (climbTimer.get() >= DELAY_FOUR_TIME)
                {
                // goes to DRIVE_FORWARD
                climbTimer.stop();
                prevState = climberState.DELAY_FOUR;
                climbState = climberState.DRIVE_FORWARD;
                }
            break;

        case DRIVE_FORWARD:
            // drives forward to the point where the the rear wheels can be
            // retracted
            if (this.driveForward() == true)
                {
                // goes to DELAY_FIVE
                prevState = climberState.DRIVE_FORWARD;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_FIVE:
            // dely state after DRIVE_FORWARD and before RAISE_ARM
            if (climbTimer.get() >= DELAY_FIVE_TIME)
                {
                // goes to RAISE_ARM
                climbTimer.stop();
                prevState = climberState.DELAY_FIVE;
                // Hardware.intakeDeploySensor.reset();
                climbState = climberState.RAISE_ARM;
                }
            break;

        case RAISE_ARM:
            // raises nessie head all the way
            if (this.raiseArm() == true)
                {
                // goes to DELAY_SIX
                prevState = climberState.RAISE_ARM;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_SIX:
            // delay state after RAISE_ARM ad before RETRACT_WHEELS
            if (climbTimer.get() >= DELAY_SIX_TIME)
                {
                // goes to RETRACT_WHEELS
                climbTimer.stop();
                prevState = climberState.DELAY_SIX;
                climbState = climberState.RETRACT_WHEELS;
                }
            break;

        case RETRACT_WHEELS:
            // retracts wheels
            if (this.retractWheels() == true)
                {
                // goes to DELAY_SEVEN
                prevState = climberState.RETRACT_WHEELS;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_SEVEN:
            // delay after RETRACT_WHEELS and before FINISH_DRIVING
            if (climbTimer.get() >= DELAY_SEVEN_TIME)
                {
                // goes to FINISH_DRIVING
                climbTimer.stop();
                prevState = climberState.DELAY_SEVEN;
                climbState = climberState.FINISH_DRIVING;
                }
            break;

        case FINISH_DRIVING:
            // drives until we get close enough to the wall
            if (this.finishDriving() == true)
                {
                // goes to stop
                prevState = climberState.FINISH_DRIVING;
                climbState = climberState.STOP;
                }
            break;

        case STOP:
            // stops stuff
            this.stop();
            break;

        case DELAY_INIT:
            // state to initialize all the delay states
            climbTimer.reset();
            climbTimer.start();

            // determines the next state to go to based in the previous state
            if (prevState == climberState.LOWER_FORKLIFT_TO_POSITION)
                {
                climbState = climberState.DELAY_ONE;
                }
            else
                if (prevState == climberState.LOWER_ARM)
                    {
                    climbState = climberState.DELAY_TWO;
                    }
                else
                    if (prevState == climberState.DEPLOY_BACK_WHEELS)
                        {
                        climbState = climberState.DELAY_THREE;
                        }
                    else
                        if (prevState == climberState.LOWER_FORKLIFT_COMPLETELY)
                            {
                            climbState = climberState.DELAY_FOUR;
                            }
                        else
                            if (prevState == climberState.DRIVE_FORWARD)
                                {
                                climbState = climberState.DELAY_FIVE;
                                }
                            else
                                if (prevState == climberState.RAISE_ARM)
                                    {
                                    climbState = climberState.DELAY_SIX;
                                    }
                                else
                                    if (prevState == climberState.RETRACT_WHEELS)
                                        {
                                        climbState = climberState.DELAY_SEVEN;
                                        }
            break;

        // welp heres a default just in case
        default:
            System.out.println("DEFAULT CLIMB CASE REACHED");
            break;
        }

}

/**
 * method to call in teleop to climb onto the platform when you are already
 * lined up with the platform
 *
 */
public void climb ()
{
    if (climbState == climberState.STANDBY)
        {
        climbState = climberState.START_CLIMB;
        }
}


// state of the climb state machine
private static enum ReverseClimberState
    {
    STANDBY, START_REVERSE_CLIMB, DRIVE_BACKWARDS, DELAY_ONE, DEPLOY_BACK_WHEELS, DELAY_TWO, LOWER_FORKLIFT_COMPLETELY, DELAY_THREE, LOWER_ARM, DELAY_FOUR, DRIVE_OFF_COMPLETELY, DELAY_FIVE, RAISE_FORKLIFT_TO_POSITION, DELAY_SIX, RETRACT_WHEELS, DELAY_SEVEN, RAISE_ARM, DELAY_INIT, STOP, FINISH;
    }


// initializes climb state and prev climb state to standby
private static ReverseClimberState reverseClimbState = ReverseClimberState.STANDBY;

private static ReverseClimberState prevReverseState = ReverseClimberState.STANDBY;



/**
 * method that encompasses the reverseClimb state machine, must be called in
 * order for reverseClimb() to work
 */

// must reorganize @ANE
public void reverseClimbUpdate ()
{
    System.out.println(reverseClimbState);

    switch (reverseClimbState)
        {
        case STANDBY:
            // state to wait in during the match until climb() or reverseClimb()
            // is called
            driveSolenoid.set(Value.kForward);
            break;

        case START_REVERSE_CLIMB:
            // state the reverseClimb() function sets the state to and is
            // basically an
            // init state
            reverseClimbState = ReverseClimberState.DRIVE_BACKWARDS;
            break;


        case DRIVE_BACKWARDS:
            // drives until we get far enough from the wall
            if (this.driveBackwards() == true)
                {
                // goes to DELAY_ONE
                prevReverseState = ReverseClimberState.DRIVE_BACKWARDS;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_ONE:
            // first delay state after DRIVE_BACKWARDS and before
            // DEPLOY_BACK_WHEELS
            if (climbTimer.get() >= DELAY_ONE_TIME)
                {
                // sets state to DEPLOY_BACK_WHEELS
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_ONE;
                // Hardware.intakeDeployEncoder.reset();
                reverseClimbState = ReverseClimberState.DEPLOY_BACK_WHEELS;
                }
            break;

        case DEPLOY_BACK_WHEELS:
            // deploys back wheels
            if (this.deployBackWheels() == true)
                {
                // Goes to DELAY_TWO
                prevReverseState = ReverseClimberState.DEPLOY_BACK_WHEELS;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_TWO:
            // delay state after DEPLOY_BACK_WHEELS and before
            // LIFT_FORKLIFT_COMPLETELY
            if (climbTimer.get() >= DELAY_TWO_TIME)
                {
                // sets state to LOWER_FORKLIFT_COMPLETELY
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_TWO;
                reverseClimbState = ReverseClimberState.LOWER_FORKLIFT_COMPLETELY;
                }
            break;

        case LOWER_FORKLIFT_COMPLETELY:
            // lowers forklift all the way and lifts the front end of the robot
            // subsequentially
            if (this.lowerForkliftCompletely() == true)
                {
                // goes to DELAY_THREE
                prevReverseState = ReverseClimberState.LOWER_FORKLIFT_COMPLETELY;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_THREE:
            // delay state after LOWER_FORKLFT_COMPLETELY and before LOWER_ARM
            if (climbTimer.get() >= DELAY_THREE_TIME)
                {
                // goes to LOWER_ARM
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_THREE;
                reverseClimbState = ReverseClimberState.LOWER_ARM;
                }
            break;


        case LOWER_ARM:
            // lowers nessie head all the way
            if (Hardware.manipulator.deployArm() == true)
                {
                // goes to DELAY_FOUR
                prevReverseState = ReverseClimberState.LOWER_ARM;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;


        case DELAY_FOUR:
            // delay after LOWER_ARM and before DRIVE_OFF_COMPLETELY
            if (climbTimer.get() >= DELAY_FOUR_TIME)
                {
                // goes to DRIVE_OFF_COMPLETELY
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_FOUR;
                reverseClimbState = ReverseClimberState.DRIVE_OFF_COMPLETELY;
                }
            break;

        case DRIVE_OFF_COMPLETELY:
            // drives backwards to the point where the the rear wheels can be
            // retracted
            if (this.reverseDriveOffCompletely() == true)
                {
                // goes to DELAY_FIVE
                prevReverseState = ReverseClimberState.DRIVE_OFF_COMPLETELY;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_FIVE:
            // delay state after DRIVE_FORWARD and before RETRACT_WHEELS
            if (climbTimer.get() >= DELAY_FIVE_TIME)
                {
                // goes to RAISE_ARM
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_FIVE;
                // Hardware.intakeDeployEncoder.reset();
                reverseClimbState = ReverseClimberState.RETRACT_WHEELS;
                }
            break;

        case RETRACT_WHEELS:
            // retracts wheels
            if (this.retractWheels() == true)
                {
                // goes to DELAY_SIX
                prevReverseState = ReverseClimberState.RETRACT_WHEELS;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_SIX:
            // delay state after RETRACT_WHEELS and before RAISE_ARM
            if (climbTimer.get() >= DELAY_SIX_TIME)
                {
                // goes to RAISE_ARM
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_SIX;
                reverseClimbState = ReverseClimberState.RAISE_FORKLIFT_TO_POSITION;
                }
            break;


        case RAISE_FORKLIFT_TO_POSITION:
            // raises forklift to the position needed to lower robot, not all
            // the way up
            if (this.lowerForkliftToPosition() == true)
                {
                // moves on to STOP
                prevReverseState = ReverseClimberState.RAISE_FORKLIFT_TO_POSITION;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_SEVEN:
            // delay after RAISE_ARM and before RAISE_FORKLIFT_TO_POSITION
            if (climbTimer.get() >= DELAY_SEVEN_TIME)
                {
                // goes to RAISE_FORKLIFT_TO_POSITION
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_SEVEN;
                reverseClimbState = ReverseClimberState.RAISE_ARM;
                }
            break;

        case RAISE_ARM:
            // raises nessie head all the way
            if (Hardware.manipulator.retractArm() == true)
                {
                // goes to DELAY_SEVEN
                prevReverseState = ReverseClimberState.RAISE_ARM;
                reverseClimbState = ReverseClimberState.STOP;
                }

        case STOP:
            // stops stuff
            this.stop();
            if (finishedEarly == true)
                {
                climbState = climberState.STANDBY;
                }
            break;

        case DELAY_INIT:
            // state to initialize all the delay states
            climbTimer.reset();
            climbTimer.start();

            // determines the next state to go to based in the previous state
            if (prevReverseState == ReverseClimberState.DRIVE_BACKWARDS)
                {
                reverseClimbState = ReverseClimberState.DELAY_ONE;
                }
            else
                if (prevReverseState == ReverseClimberState.DEPLOY_BACK_WHEELS)
                    {
                    reverseClimbState = ReverseClimberState.DELAY_TWO;
                    }
                else
                    if (prevReverseState == ReverseClimberState.LOWER_FORKLIFT_COMPLETELY)
                        {
                        reverseClimbState = ReverseClimberState.DELAY_THREE;
                        }
                    else
                        if (prevReverseState == ReverseClimberState.LOWER_ARM)
                            {
                            reverseClimbState = ReverseClimberState.DELAY_FOUR;
                            }
                        else
                            if (prevReverseState == ReverseClimberState.DRIVE_OFF_COMPLETELY)
                                {
                                reverseClimbState = ReverseClimberState.DELAY_FIVE;
                                }
                            else
                                if (prevReverseState == ReverseClimberState.RETRACT_WHEELS)
                                    {
                                    reverseClimbState = ReverseClimberState.DELAY_SIX;
                                    }
                                else
                                    if (prevReverseState == ReverseClimberState.RAISE_FORKLIFT_TO_POSITION)
                                        {
                                        reverseClimbState = ReverseClimberState.DELAY_SEVEN;
                                        }
            break;

        case FINISH:
            this.stop();
            System.out.println(
                    "WE HAVE FINISHED WHAT WE WANTED TO OF REVERSE CLIMB");
            break;


        // welp heres a default just in case
        default:
            System.out.println("DEFAULT CLIMB CASE REACHED");
            break;
        }

}

/**
 * method to get down from the platform by running the states of climb in
 * reverse
 */
public void reverseClimb ()
{
    reverseClimbState = ReverseClimberState.START_REVERSE_CLIMB;
}


/**
 * method to lower forklift to the height where wecn start the sequence of
 * climbing on to the second level
 */
private boolean lowerForkliftToPosition ()
{
    // System.out.println(liftEncoder.get());
    // if (liftEncoder.get() >= LIFT_HEIGHT_TO_START_CLIMB) {
    // liftMotor.set(LOWER_LIFT_SPEED);
    // } else {
    // liftMotor.set(0.0);
    System.out.println("Trying to lower forklift to position ");
    if (this.lift.setLiftPosition(LIFT_HEIGHT_TO_START_CLIMB,
            LOWER_LIFT_SPEED))
        {
        return true;
        }
    return false;

}

/**
 * method to lower arm to the position needed to climb- all the way down
 */
private boolean lowerArm ()
{
    // checks the position of the arm and if it has not gone far enough, sets
    // the motor to the sped to lower the arm
    System.out.println("Trying to lower arm");
    // if (this.armSensor.get() <= LOWERED_ARM_POSITION)
    // {
    // armMotor.set(LOWER_ARM_SPEED);
    // }
    // else
    // {
    // armMotor.set(0.0);
    // return true;
    // }
    if (Hardware.manipulator.deployArm() == true)
        {
        return true;
        }
    return false;
}

/**
 * method that calls the forklift method and sets it to the the min position
 *
 * @return whether it has completed the movement
 */
private boolean lowerForkliftCompletely ()
{
    System.out.println("Trying to lower forklift completely");
    // if (liftEncoder.get() >= MIN_LIFT_HEIGHT_TO_CLIMB) {
    // liftMotor.set(LOWER_LIFT_SPEED);
    // } else {
    // liftMotor.set(0.0);
    // }
    // armMotor.set(ARM_HOLD_SPEED);
    if (lift.setLiftPosition(MIN_LIFT_HEIGHT_TO_CLIMB,
            LOWER_LIFT_SPEED))
        {
        return true;
        }
    return false;
}

/**
 * method to lower the back wheels by pneumatics in order to climb
 *
 * @return whether the maction has been completed
 */
private boolean deployBackWheels ()
{
    System.out.println("Trying to deploy back wheels");
    driveSolenoid.setForward(LOWER_WHEELS_POSITION);
    // sets test solnoid to a position
    // testSolenoid.set(Value.kReverse);
    return true;

}

/**
 * drives to the point that we can raise our wheels and continue climbing
 *
 * @return- whether its been completed
 */
private boolean driveForward ()
{
    // drive forward a set distance
    System.out.println("Trying to drive forward");
    if (drive.driveStraightInches(DISTANCE_TO_DRIVE_B4_RETRACTION_NORM,
            SPEED_TO_DRIVE_UP, ACCELERATION_TIME, false) == true)
        {
        return true;
        }
    return false;
}

/**
 * method to raise arm all the way back
 *
 * @return- whether it has been completed
 */
private boolean raiseArm ()
{
    // checks to see if the arm has gone far enough, if not it sets th ar to the
    // raise arm speed
    System.out.println("Trying to raise arm");
    // if (armSensor.get() >= RAISED_ARM_POSITION)
    // {
    // armMotor.set(RAISE_ARM_SPEED);
    // }
    // else
    // {
    // armMotor.set(0.0);
    // return true;
    // }
    if (Hardware.manipulator.retractArm() == true)
        {
        return true;
        }

    return false;
}

/**
 * retracts the wheels
 *
 * @return - whether its completed
 */
private boolean retractWheels ()
{
    // sets the solenoid to the correct position
    System.out.println("Trying to retract wheels");
    driveSolenoid.setForward(RETRACT_WHEELS_POSITION);
    // testSolenoid.set(Value.kForward);
    return true;
}


/**
 * drives until the ultraSonic reads the right value
 *
 * @return - whether it has been completed
 */
private boolean finishDriving ()
{
    // checks th ultrasonic and if it does not red the right value it drives
    // forward
    // System.out.println("Trying to finish driving");
    if (this.ultraSonic
            .getDistanceFromNearestBumper() >= DISTANCE_B4_STOPPING)
        {
        this.drive.driveStraight(SPEED_TO_FINISH_DRIVING, .6, false);
        return false;
        }
    else
        {
        stop();
        return true;
        }

}


// reverse specific methods

public boolean driveBackwards ()
{
    if (ultraSonic
            .getDistanceFromNearestBumper() >= DISTANCE_TO_DRIVE_B4_DEPLOYMENT)
        {
        drive.stop();
        return true;
        }
    else
        {
        drive.drive(SPEED_TO_FINISH_REVERSE_DRIVING,
                SPEED_TO_FINISH_REVERSE_DRIVING);
        return false;
        }

}


public boolean reverseDriveOffCompletely ()
{
    if (drive.driveStraightInches(DISTANCE_BEFORE_END_REVERSE_CLIMB,
            SPEED_TO_FINISH_REVERSE_CLIMB, ACCELERATION_TIME,
            true) == true)
        {
        drive.stop();
        return true;
        }
    return false;
}








/**
 * stops all the various mechanisms so we just sit there on the platform
 */
private void stop ()
{
    // System.out.println("Trying to stop");
    drive.stop();
    Hardware.manipulator.setDeployMovementState(
            GamePieceManipulator.DeployMovementState.STAY_AT_POSITION);
    lift.liftState = Forklift.ForkliftState.STOP;
    driveSolenoid.setForward(RETRACT_WHEELS_POSITION);
}

/**
 * method to end climbing early
 */
public void finishEarly ()
{
    // sets state to STOP
    finishedEarly = true;
    climbState = climberState.STOP;
}


// =======================================================================
//

// ---------------------------------------------
// Constants
// ---------------------------------------------

// Positions
private static final double LOWERED_ARM_POSITION = 260;

private static final double RAISED_ARM_POSITION = 225;

private static final double LIFT_HEIGHT_TO_START_CLIMB = 30.0;

private static final double MIN_LIFT_HEIGHT_TO_CLIMB = 10.0;

private static final Boolean LOWER_WHEELS_POSITION = true;

private static final Boolean RETRACT_WHEELS_POSITION = false;


// SPEEDS

// private static final double RAISE_ARM_SPEED = .7;

// private static final double LOWER_ARM_SPEED = -.4;

private static final double ARM_HOLD_SPEED = 1.0;

private static final double SPEED_TO_DRIVE_UP = .4;

private static final double SPEED_TO_REVERSE_DRIVE_OFF = -.4;

private static final double SPEED_TO_FINISH_DRIVING = .4;

private static final double SPEED_TO_FINISH_REVERSE_DRIVING = -.4;

private static final double LOWER_LIFT_SPEED = .3;

private static final double SPEED_TO_FINISH_REVERSE_CLIMB = -.4;

// DISTANCES

private static final int DISTANCE_B4_STOPPING = 10;

private static final int DISTANCE_TO_DRIVE_B4_RETRACTION_NORM = 20;

private static final int DISTANCE_TO_DRIVE_B4_DEPLOYMENT = 20;

private static final int DISTANCE_BEFORE_END_REVERSE_CLIMB = 10;


// TIMES

private static final double ACCELERATION_TIME = .2;

private static final double DELAY_ONE_TIME = 0.0;

private static final double DELAY_TWO_TIME = 0.0;

private static final double DELAY_THREE_TIME = 1.0;

private static final double DELAY_FOUR_TIME = 0.0;

private static final double DELAY_FIVE_TIME = 0.0;

private static final double DELAY_SIX_TIME = 0.0;

private static final double DELAY_SEVEN_TIME = 1.0;


// Extra tuneable stuff

public static boolean finishedEarly = false;

public static boolean reverseClimb = false;

}
