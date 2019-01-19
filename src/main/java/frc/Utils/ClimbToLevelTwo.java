package frc.Utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DoubleSolenoid;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.SingleSolenoid;
import frc.Utils.drive.Drive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class ClimbToLevelTwo {

    public static SingleSolenoid driveSolenoid = null;

    public static SpeedController armMotor = null;

    public static KilroyEncoder armEncoder = null;

    // public static DoubleSolenoid testSolenoid = null;

    public static Drive drive = null;

    public static Forklift lift = null;

    public ClimbToLevelTwo() {
        this.driveSolenoid = null;
        this.armMotor = null;
        this.lift = null;
        this.drive = null;
    }

    public ClimbToLevelTwo(DoubleSolenoid testSolenoid, SpeedController armMotor, KilroyEncoder armEncoder, Drive drive,
            Forklift lift) {
        // this.testSolenoid = testSolenoid;
    }

    public ClimbToLevelTwo(SingleSolenoid driveSolenoid, SpeedController armMotor, KilroyEncoder armEncoder,
            Drive drive, Forklift lift) {

        this.driveSolenoid = driveSolenoid;
        this.armMotor = armMotor;
        this.armEncoder = armEncoder;
        this.lift = lift;

    }

    public static void climb() {
        if (lowerForkliftToPosition() == true) {
            if (lowerArm() == true) {
                if (lowerForkliftCompletely() == true) {
                    deployBackWheels();
                    if (driveForward() == true) {
                        if (raiseArm() == true) {
                            retractWheels();
                            if (finishDriving() == true) {
                                stop();
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * method to lower forklift to the height where wecn start the sequence of
     * climbing on to the second level
     */
    private static boolean lowerForkliftToPosition() {
        // System.out.println(liftEncoder.get());
        // if (liftEncoder.get() >= LIFT_HEIGHT_TO_START_CLIMB) {
        // liftMotor.set(LOWER_LIFT_SPEED);
        // } else {
        // liftMotor.set(0.0);
        System.out.println("Trying to lower forklift to position ");
        if (lift.setLiftPosition(LIFT_HEIGHT_TO_START_CLIMB, LOWER_LIFT_SPEED)) {
            return true;
        }
        return false;

    }

    private static boolean lowerArm() {
        System.out.println("Trying to lower arm");
        if (armEncoder.get() <= LOWERED_ARM_POSITION) {
            armMotor.set(LOWER_ARM_SPEED);
        } else {
            armMotor.set(0.0);
            return true;
        }
        return false;
    }

    private static boolean lowerForkliftCompletely() {
        System.out.println("Trying to lower forklift completely");
        // if (liftEncoder.get() >= MIN_LIFT_HEIGHT_TO_CLIMB) {
        // liftMotor.set(LOWER_LIFT_SPEED);
        // } else {
        // liftMotor.set(0.0);
        // }
        armMotor.set(ARM_HOLD_SPEED);
        if (lift.setLiftPosition(MIN_LIFT_HEIGHT_TO_CLIMB, LOWER_LIFT_SPEED)) {
            return true;
        }
        return false;
    }

    private static void deployBackWheels() {
        System.out.println("Trying to deploy back wheels");
        driveSolenoid.set(LOWER_WHEELS_POSITION);

    }

    private static boolean driveForward() {
        System.out.println("Trying to drive forward");
        if (drive.driveInches(DISTANCE_TO_DRIVE_B4_RETRACTION, SPEED_TO_DRIVE_UP) == true) {
            return true;
        }
        // System.out.println(timer.get());
        // if (timerInit == false) {
        // timer.reset();
        // timer.start();
        // timerInit = true;
        // Hardware.cubeIntakeMotor.set(.5);
        // }

        // if (timer.get() * 500 > 5.0) {
        // Hardware.cubeIntakeMotor.set(0.0);
        // timer.stop();
        // System.out.println("CATS CATS");
        // return true;
        // }
        return false;
    }

    private static boolean raiseArm() {
        System.out.println("Trying to raise arm");
        if (armEncoder.get() >= RAISED_ARM_POSITION) {
            armMotor.set(RAISE_ARM_SPEED);
        } else {
            armMotor.set(0.0);
            return true;
        }
        return false;
    }

    private static void retractWheels() {
        System.out.println("Trying to retract wheels");
        driveSolenoid.set(RETRACT_WHEELS_POSITION);
    }

    private static boolean finishDriving() {
        System.out.println("Trying to finish driving");
        if (drive.driveInches(DISTANCE_TO_FINISH_DRIVING, SPEED_TO_DRIVE_UP)) {
            stop();
            return true;
        }
        // timer.reset();
        // timer.start();
        // Hardware.cubeIntakeMotor.set(.5);
        // if (timer.get() > 5.0) {
        // Hardware.cubeIntakeMotor.set(0.0);
        // timer.stop();
        // return true;
        // }
        return false;
    }

    private static void stop() {
        System.out.println("Trying to stop");
        lift.liftState = Forklift.ForkliftState.STOP;
        driveSolenoid.set(RETRACT_WHEELS_POSITION);
        drive.stop();
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

    private static final int DISTANCE_TO_DRIVE_B4_RETRACTION = 20;

    private static final int DISTANCE_TO_FINISH_DRIVING = 20;

    private static final double LOWERED_ARM_POSITION = 90;

    private static final double LOWER_ARM_SPEED = .4;

    private static final double ARM_HOLD_SPEED = 1.0;

    private static final double RAISED_ARM_POSITION = 15;

    private static final double RAISE_ARM_SPEED = .7;

}
