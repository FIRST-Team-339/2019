package frc.Utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.SingleSolenoid;
import frc.HardwareInterfaces.UltraSonic;
import frc.Utils.drive.Drive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class ClimbToLevelTwo
{

private SingleSolenoid driveSolenoid = null;

private SpeedController armMotor = null;

private KilroyEncoder armEncoder = null;

private static DoubleSolenoid testSolenoid = null;

private Drive drive = null;

private Forklift lift = null;

private UltraSonic ultraSonic = null;

private Timer climbTimer = new Timer();

private static enum climberState
    {
STANDBY, START_CLIMB, LOWER_FORKLIFT_TO_POSITION, DELAY_ONE, LOWER_ARM, DELAY_TWO, DEPLOY_BACK_WHEELS, DELAY_THREE, LOWER_FORKLIFT_COMPLETELY, DELAY_FOUR, DRIVE_FORWARD, DELAY_FIVE, RAISE_ARM, DELAY_SIX, RETRACT_WHEELS, DELAY_SEVEN, FINISH_DRIVING, DELAY_INIT, STOP
    }

private static climberState climbState = climberState.STANDBY;

private static climberState prevState = climberState.STANDBY;

public ClimbToLevelTwo ()
{
    this.driveSolenoid = null;
    this.armMotor = null;
    lift = null;
    this.drive = null;
}

public ClimbToLevelTwo (DoubleSolenoid testSolenoid,
        SpeedController armMotor, KilroyEncoder armEncoder, Drive drive,
        Forklift lift, UltraSonic uS)
{

    this.testSolenoid = testSolenoid;
    this.armMotor = armMotor;
    this.armEncoder = armEncoder;
    this.drive = drive;
    this.lift = lift;
    this.ultraSonic = uS;
}

public ClimbToLevelTwo (SingleSolenoid driveSolenoid,
        SpeedController armMotor, KilroyEncoder armEncoder,
        Drive drive, Forklift lift, UltraSonic uS)
{

    this.driveSolenoid = driveSolenoid;
    this.armMotor = armMotor;
    this.armEncoder = armEncoder;
    this.drive = drive;
    this.lift = lift;
    this.ultraSonic = uS;
}

public void climbUpdate ()
{
    // System.out.println(climbState);

    switch (climbState)
        {
        case STANDBY:
            testSolenoid.set(Value.kForward);
            break;

        case START_CLIMB:
            climbState = climberState.LOWER_FORKLIFT_TO_POSITION;
            break;

        case LOWER_FORKLIFT_TO_POSITION:
            if (this.lowerForkliftToPosition() == true)
                {
                prevState = climberState.LOWER_FORKLIFT_TO_POSITION;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_ONE:
            if (climbTimer.get() >= DELAY_ONE_TIME)
                {
                climbTimer.stop();
                prevState = climberState.DELAY_ONE;
                climbState = climberState.LOWER_ARM;
                }
            break;

        case LOWER_ARM:
            if (this.lowerArm() == true)
                {
                prevState = climberState.LOWER_ARM;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_TWO:
            if (climbTimer.get() >= DELAY_TWO_TIME)
                {
                climbTimer.stop();
                prevState = climberState.DELAY_TWO;
                climbState = climberState.DEPLOY_BACK_WHEELS;
                }
            break;

        case DEPLOY_BACK_WHEELS:
            if (this.deployBackWheels() == true)
                {
                prevState = climberState.DEPLOY_BACK_WHEELS;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_THREE:
            if (climbTimer.get() >= DELAY_THREE_TIME)
                {
                climbTimer.stop();
                prevState = climberState.DELAY_THREE;
                climbState = climberState.LOWER_FORKLIFT_COMPLETELY;
                }
            break;

        case LOWER_FORKLIFT_COMPLETELY:
            if (this.lowerForkliftCompletely() == true)
                {
                prevState = climberState.LOWER_FORKLIFT_COMPLETELY;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_FOUR:
            if (climbTimer.get() >= DELAY_FOUR_TIME)
                {
                climbTimer.stop();
                prevState = climberState.DELAY_FOUR;
                climbState = climberState.DRIVE_FORWARD;
                }
            break;

        case DRIVE_FORWARD:
            if (this.driveForward() == true)
                {
                prevState = climberState.DRIVE_FORWARD;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_FIVE:
            if (climbTimer.get() >= DELAY_FIVE_TIME)
                {
                climbTimer.stop();
                prevState = climberState.DELAY_FIVE;
                climbState = climberState.RAISE_ARM;
                }
            break;

        case RAISE_ARM:
            if (this.raiseArm() == true)
                {
                prevState = climberState.RAISE_ARM;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_SIX:
            if (climbTimer.get() >= DELAY_SIX_TIME)
                {
                climbTimer.stop();
                prevState = climberState.DELAY_SIX;
                climbState = climberState.RETRACT_WHEELS;
                }
            break;

        case RETRACT_WHEELS:
            if (this.retractWheels() == true)
                {
                prevState = climberState.RETRACT_WHEELS;
                climbState = climberState.DELAY_INIT;
                }
            break;

        case DELAY_SEVEN:
            if (climbTimer.get() >= DELAY_SEVEN_TIME)
                {
                climbTimer.stop();
                prevState = climberState.DELAY_SEVEN;
                climbState = climberState.FINISH_DRIVING;
                }
            break;

        case FINISH_DRIVING:
            if (this.finishDriving() == true)
                {
                prevState = climberState.FINISH_DRIVING;
                climbState = climberState.STOP;
                }
            break;

        case STOP:
            this.stop();
            break;

        case DELAY_INIT:
            climbTimer.reset();
            climbTimer.start();

            // determines the next state to go to based in the previous state
            if (prevState == climberState.LOWER_FORKLIFT_TO_POSITION)
                {
                climbState = climberState.DELAY_ONE;
                } else if (prevState == climberState.LOWER_ARM)
                {
                climbState = climberState.DELAY_TWO;
                } else if (prevState == climberState.DEPLOY_BACK_WHEELS)
                {
                climbState = climberState.DELAY_THREE;
                } else if (prevState == climberState.LOWER_FORKLIFT_COMPLETELY)
                {
                climbState = climberState.DELAY_FOUR;
                } else if (prevState == climberState.DRIVE_FORWARD)
                {
                climbState = climberState.DELAY_FIVE;
                } else if (prevState == climberState.RAISE_ARM)
                {
                climbState = climberState.DELAY_SIX;
                } else if (prevState == climberState.RETRACT_WHEELS)
                {
                climbState = climberState.DELAY_SEVEN;
                }
            break;

        default:
            break;
        }

}

/**
 *
 *
 */
public void climb ()
{
    climbState = climberState.START_CLIMB;
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
    // if (this.lift.setLiftPosition(LIFT_HEIGHT_TO_START_CLIMB,
    // LOWER_LIFT_SPEED))
    // {
    return true;
}
// return false;

// }

private boolean lowerArm ()
{
    System.out.println("Trying to lower arm");
    if (this.armEncoder.get() <= LOWERED_ARM_POSITION)
        {
        armMotor.set(LOWER_ARM_SPEED);
        } else
        {
        armMotor.set(0.0);
        return true;
        }
    return false;
}

private boolean lowerForkliftCompletely ()
{
    System.out.println("Trying to lower forklift completely");
    // if (liftEncoder.get() >= MIN_LIFT_HEIGHT_TO_CLIMB) {
    // liftMotor.set(LOWER_LIFT_SPEED);
    // } else {
    // liftMotor.set(0.0);
    // }
    // armMotor.set(ARM_HOLD_SPEED);
    // if (lift.setLiftPosition(MIN_LIFT_HEIGHT_TO_CLIMB, LOWER_LIFT_SPEED)) {
    return true;
    // }
    // return false;
}

private boolean deployBackWheels ()
{
    System.out.println("Trying to deploy back wheels");
    // driveSolenoid.set(LOWER_WHEELS_POSITION);
    testSolenoid.set(Value.kReverse);
    return true;

}

private boolean driveForward ()
{
    System.out.println("Trying to drive forward");
    if (drive.driveStraightInches(DISTANCE_TO_DRIVE_B4_RETRACTION,
            SPEED_TO_DRIVE_UP, .6, false) == true)
        {
        return true;
        }
    return false;
}

private boolean raiseArm ()
{
    System.out.println("Trying to raise arm");
    if (armEncoder.get() >= RAISED_ARM_POSITION)
        {
        armMotor.set(RAISE_ARM_SPEED);
        } else
        {
        armMotor.set(0.0);
        return true;
        }
    return false;
}

private boolean retractWheels ()
{
    System.out.println("Trying to retract wheels");
    // driveSolenoid.set(RETRACT_WHEELS_POSITION);
    testSolenoid.set(Value.kForward);
    return true;
}

private boolean finishDriving ()
{
    System.out.println("Trying to finish driving");
    if (this.ultraSonic
            .getDistanceFromNearestBumper() >= DISTANCE_B4_STOPPING)
        {
        this.drive.driveStraight(SPEED_TO_DRIVE_UP, .6, false);
        return false;
        } else
        {
        stop();
        return true;
        }

}

private void stop ()
{
    System.out.println("Trying to stop");
    drive.stop();
    armMotor.set(0.0);
    // lift.liftState = Forklift.ForkliftState.STOP;
    // driveSolenoid.set(RETRACT_WHEELS_POSITION);
    // drive.stop();
}

public void finishEarly ()
{
    climbState = climberState.STOP;
}

// ---------------------------------------------
// Constants
// ---------------------------------------------
private static final double LOWER_LIFT_SPEED = .3;

private static final double LIFT_HEIGHT_TO_START_CLIMB = 30.0;

private static final double MIN_LIFT_HEIGHT_TO_CLIMB = 10.0;

private static final boolean CLIMB_ARM_POSITION = true;

private static final boolean RETRACTED_ARM_POSITION = false;

private static final boolean LOWER_WHEELS_POSITION = true;

private static final boolean RETRACT_WHEELS_POSITION = false;

private static final double SPEED_TO_DRIVE_UP = .5;

private static boolean timerInit = false;

private static final int DISTANCE_TO_DRIVE_B4_RETRACTION = 200;

private static final int DISTANCE_TO_FINISH_DRIVING = 20;

private static final double LOWERED_ARM_POSITION = 180;

private static final double LOWER_ARM_SPEED = -.4;

private static final double ARM_HOLD_SPEED = 1.0;

private static final double RAISED_ARM_POSITION = 10.0;

private static final double RAISE_ARM_SPEED = .7;

private static final double DELAY_ONE_TIME = 0.5;

private static final double DELAY_TWO_TIME = 0.5;

private static final double DELAY_THREE_TIME = 0.5;

private static final double DELAY_FOUR_TIME = 0.5;

private static final double DELAY_FIVE_TIME = 0.5;

private static final double DELAY_SIX_TIME = 0.5;

private static final double DELAY_SEVEN_TIME = 0.5;

private static final int DISTANCE_B4_STOPPING = 20;

}
