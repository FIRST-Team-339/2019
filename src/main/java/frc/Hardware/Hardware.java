// ====================================================================
// FILE NAME: Hardware.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 2, 2011
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file contains all of the global definitions for the
// hardware objects in the systemr
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.Hardware;

import frc.HardwareInterfaces.DriveWithCamera;
import frc.HardwareInterfaces.DoubleSolenoid;
import frc.HardwareInterfaces.DoubleThrowSwitch;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.KilroySPIGyro;
import frc.HardwareInterfaces.LVMaxSonarEZ;
import frc.HardwareInterfaces.LightSensor;
import frc.HardwareInterfaces.MomentarySwitch;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.SingleThrowSwitch;
import frc.HardwareInterfaces.SixPositionSwitch;
import frc.Utils.drive.Drive;
import frc.Utils.drive.DrivePID;
import frc.vision.AutoGenVision;
import frc.vision.VisionProcessor;
import frc.vision.VisionProcessor.CameraModel;
import frc.HardwareInterfaces.Transmission.TankTransmission;
import frc.Utils.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ------------------------------------------------------- puts all of the
 * hardware declarations into one place. In addition, it makes them available to
 * both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */

public class Hardware
{
// ------------------------------------
// Public Constants
// ------------------------------------
public enum RobotYear
    {
KILROY_2018, KILROY_2019, TEST_BOARD
    }

public static final RobotYear whichRobot = RobotYear.KILROY_2018;

// -------------------------------------
// Private Constants
// -------------------------------------

// ---------------------------------------
// Hardware Tunables
// ---------------------------------------
public static boolean demoMode = false;

// **********************************************************
// DIGITAL I/O CLASSES
// **********************************************************

// ====================================
// PWM classes
// ====================================

// ------------------------------------
// Jaguar classes
// ------------------------------------

// ------------------------------------
// Talon classes
// ------------------------------------

// ------------------------------------
// Victor Classes
// ------------------------------------
public static VictorSP intakeDeployArm;

public static VictorSP armMotor;

// ------------------------------------
// Servo classes
// ------------------------------------

// ====================================
// CAN classes
// ====================================
public static PowerDistributionPanel pdp;

public static SpeedController liftMotor;

/** The right front drive motor */
public static SpeedController rightFrontCANMotor;

/** The left front drive motor */
public static SpeedController leftFrontCANMotor;

/** The right rear drive motor */
public static SpeedController rightRearCANMotor;

/** The left rear drive motor */
public static SpeedController leftRearCANMotor;

public static SpeedController armRoller;

// ====================================
// Relay classes
// ====================================
public static Relay ringLightRelay;

// ====================================
// Digital Inputs
// ====================================
// ------------------------------------
// Single and double throw switches
// ------------------------------------
public static SingleThrowSwitch leftAutoSwitch;

public static SingleThrowSwitch rightAutoSwitch;

public static DoubleThrowSwitch autoCenterSwitch;

public static SingleThrowSwitch levelOneSwitch;

public static SingleThrowSwitch levelTwoSwitch;

public static DoubleThrowSwitch autoDisableSwitch;

public static SixPositionSwitch autoSixPosSwitch;

// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------
public static KilroyEncoder leftFrontDriveEncoder;

public static KilroyEncoder rightFrontDriveEncoder;

public static KilroyEncoder liftingEncoder;

// -----------------------
// Wiring diagram
// -----------------------
// Orange - Red PWM 1
// Yellow - White PWM 1 Signal
// Brown - Black PWM 1 (or PWM 2)
// Blue - White PWM 2 Signal
// For the AMT103 Encoders UNVERIFIED
// B - White PWM 2
// 5V - Red PWM 1 or 2
// A - White PWM 1
// X - index channel, unused
// G - Black PWM 1 or 2
// see http://www.cui.com/product/resource/amt10-v.pdf page 4 for Resolution
// (DIP Switch) Settings (currently all are off)

// -------------------------------------
// Red Light/IR Sensor class
// -------------------------------------

public static LightSensor armIR;
// TODO check port for 2018 robot

public static LightSensor leftBackIR = null;

// the IR on the back right part of the robot
public static LightSensor rightBackIR = null;

// ====================================
// I2C Classes
// ====================================

// **********************************************************
// SOLENOID I/O CLASSES
// **********************************************************
// ====================================
// Compressor class - runs the compressor
// ====================================
public static Compressor compressor;

// ====================================
// Pneumatic Control Module
// ====================================

// ====================================
// Solenoids
// ====================================
// ------------------------------------
// Double Solenoids
// ------------------------------------

public static DoubleSolenoid armIntakeSolenoid;

// ------------------------------------
// Single Solenoids
// ------------------------------------

// **********************************************************
// ANALOG I/O CLASSES
// **********************************************************
// ====================================
// Analog classes
// ====================================
// ------------------------------------
// Gyro class
// ------------------------------------
// P/N ADW22307

// --------------------------------------
// Potentiometers
// --------------------------------------
public static RobotPotentiometer delayPot;

public static RobotPotentiometer intakeDeploySensor;

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------
public static LVMaxSonarEZ frontUltraSonic;

// =====================================
// SPI Bus
// =====================================

// -------------------------------------
// Analog Interfaces
// -------------------------------------
// if you are getting a null pointer exception from the gyro, try setting the
// parameter you are passing into this declaration to false. The null pointer
// exception is probably because there is not a gyro on the robot, and passing
// in a false will tell the robot we do not have a gyro without requiring us to
// comment out the gyro declaration.

public static KilroySPIGyro gyro;


// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------

public static VisionProcessor axisCamera;

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------

public static UsbCamera USBCam;

public static UsbCamera USBCamII;

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------

// **********************************************************
// DRIVER STATION CLASSES
// **********************************************************
// ------------------------------------
// DriverStations class
// ------------------------------------
public static DriverStation driverStation;

// ------------------------------------
// Joystick classes
// ------------------------------------
public static Joystick leftDriver;

public static Joystick rightDriver;

public static Joystick leftOperator;

public static Joystick rightOperator;

// ------------------------------------
// Buttons classes
// ------------------------------------
// ----- Left Operator -----

// left trigger
public static JoystickButton intakeTrigger;

public static JoystickButton outtakeButton;

public static JoystickButton intakeOverride;

public static JoystickButton deployOverride;

public static JoystickButton cargoShipCargoButton;

public static JoystickButton cargoShipHatchButton;

// ----- Right Operator -----

public static JoystickButton chooseCargoRocketHeights;

public static JoystickButton forkliftOverride;

public static JoystickButton nextHigherForkliftTargetHeight;

public static JoystickButton nextLowerForkliftTargetHeight;


// ------------------------------------
// Momentary Switches
// ------------------------------------
public static MomentarySwitch descendButton;

public static MomentarySwitch ringLightButton;

// **********************************************************
// Kilroy's Ancillary classes
// **********************************************************

// -------------------------------------
// PID tuneables
// -------------------------------------

// -------------------------------------
// PID classes
// -------------------------------------

// ------------------------------------
// Utility classes
// ------------------------------------
public static Timer autoTimer;

public static Timer deployTimer;

public static Telemetry telemetry;

// ------------------------------------
// Transmission class
// ------------------------------------
public static TankTransmission transmission;

// ------------------------------------
// Drive system
// ------------------------------------
// @ANE
public static Drive drive;

public static DrivePID drivePID;

// TODO CHANGE TO FRONT ENCODERS ON REAL ROBOT
// TODO update with encoders once fixed
public static DriveWithCamera driveWithCamera;

// -------------------
// Assembly classes (e.g. forklift)
// -------------------

public static GamePieceManipulator manipulator;

public static Forklift lift;

public static ClimbToLevelTwo climber;

public static AlignPerpendicularToTape alignByTape;

// ====================================
// Methods
// ====================================


/**
 * This initializes the hardware for the robot depending on which year we are
 * using
 */
public static void initialize ()
{
    switch (whichRobot)
        {
        default:
        case KILROY_2018:
            robotInitialize2018();
            break;

        case KILROY_2019:
            robotInitialize2019();
            break;
        case TEST_BOARD:
            gyro = new KilroySPIGyro(false);
            break;
        }
}

/**
 * it's a switch statement for the current robot, a robot we don't have,
 * and a robot Mr. Brown said not to use.
 *
 * @author Patrick
 */
public static void setHardwareSettings ()
{
    switch (whichRobot)
        {
        default:
        case KILROY_2018:
            setHardwareSettings2018();
            break;

        case KILROY_2019:
            setHardwareSettings2019();
            break;

        case TEST_BOARD:
            // Mr. Brown said no, so that means no. Specifically, he said "No,
            // no no no no no, no way, no how. No."
            break;
        }
}

/**
 * This sets up the settings and resets for the hardware objects so we
 * don't have to write them all in robotInit. it's to keep it not cluttered.
 *
 * @author Patrick
 */
public static void setHardwareSettings2018 ()
{
    Hardware.compressor.setClosedLoopControl(true);

    Hardware.gyro.calibrate();
    Hardware.gyro.reset();


    // written by Meghan Brown 2019
    // makes the camera work --- can we get this in colour somehow?
    // Hardware.USBCam.setResolution(320, 240);
    // Hardware.USBCam.setFPS(20);
    // Hardware.USBCam.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    // Hardware.USBCam.setWhiteBalanceManual(2500);

    // Hardware.USBCamII.setResolution(320, 240);
    // Hardware.USBCamII.setFPS(20);
    // Hardware.USBCamII.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    // Hardware.USBCamII.setWhiteBalanceManual(2500);

    // Hardware.USBCamUp.setResolution(320, 240);
    // Hardware.USBCamUp.setFPS(20);
    // Hardware.USBCamUp.setPixelFormat(VideoMode.PixelFormat.kYUYV);

    // Hardware.rightFrontDriveEncoder.setReverseDirection(true);
    // Hardware.leftFrontDriveEncoder.setReverseDirection(true);
    Hardware.liftingEncoder.setReverseDirection(true);

    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------

    // -------------------------------------
    // Resets encoder values
    // -------------------------------------
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.liftingEncoder.reset();

    // -------------------------------------
    // Manually sets encoders Distance per Pulse
    // -------------------------------------
    Hardware.leftFrontDriveEncoder
            .setDistancePerPulse(KILROY_XIX_DRIVE_ENCODER_DPP);
    Hardware.rightFrontDriveEncoder
            .setDistancePerPulse(KILROY_XIX_DRIVE_ENCODER_DPP);

    Hardware.liftingEncoder
            .setDistancePerPulse(KILROY_XIX_LIFT_ENCODER_DPP);

    System.out.println("Connor would say \"we have done the thing\"");

}

private static final double KILROY_XIX_DRIVE_ENCODER_DPP = 0.0346;

private static final double KILROY_XIX_LIFT_ENCODER_DPP = 0.02;


public static void setHardwareSettings2019 ()
{

}

/**
 * This initializes all of the components in Hardware
 */
public static void robotInitialize2018 ()
{
    // **********************************************************
    // DIGITAL I/O CLASSES
    // **********************************************************

    // ====================================
    // PWM classes
    // ====================================

    // ----- Jaguar classes -----
    // ----- Talon classes -----
    // ----- Victor classes -----

    armMotor = new VictorSP(4);
    // hanging
    // ----- Servo classes -----


    // ====================================
    // CAN classes
    // ====================================
    pdp = new PowerDistributionPanel(2);

    liftMotor = new WPI_TalonSRX(23);

    rightFrontCANMotor = new WPI_TalonSRX(14);

    leftFrontCANMotor = new WPI_TalonSRX(11);

    rightRearCANMotor = new WPI_TalonSRX(15);

    leftRearCANMotor = new WPI_TalonSRX(13);

    armRoller = new WPI_TalonSRX(10);// fix CANID

    // ====================================
    // Relay classes
    // ====================================
    ringLightRelay = new Relay(0);

    // ====================================
    // Digital Inputs
    // ====================================
    // -------------------------------------
    // Single and double throw switches
    // -------------------------------------
    leftAutoSwitch = new SingleThrowSwitch(
            20);

    rightAutoSwitch = new SingleThrowSwitch(
            25);

    autoCenterSwitch = new DoubleThrowSwitch(
            leftAutoSwitch, rightAutoSwitch);

    levelOneSwitch = new SingleThrowSwitch(22);

    levelTwoSwitch = new SingleThrowSwitch(23);

    autoDisableSwitch = new DoubleThrowSwitch(levelOneSwitch,
            levelTwoSwitch);

    autoSixPosSwitch = new SixPositionSwitch(13, 14, 15, 16, 17, 18);



    // Gear Tooth Sensors


    // Encoders

    leftFrontDriveEncoder = new KilroyEncoder(
            4, 5);

    rightFrontDriveEncoder = new KilroyEncoder(
            6, 7);

    liftingEncoder = new KilroyEncoder(10,
            11);

    // -------------------------------------
    // Red Light/IR Sensor class
    // -------------------------------------

    armIR = new LightSensor(21);

    leftBackIR = new LightSensor(8);

    rightBackIR = new LightSensor(9);

    // ====================================
    // I2C Classes
    // ====================================

    // **********************************************************
    // SOLENOID I/O CLASSES
    // **********************************************************
    // ====================================
    // Compressor class - runs the compressor
    // ====================================
    compressor = new Compressor();

    // ====================================
    // Pneumatic Control Module
    // ====================================

    // ====================================
    // Solenoids
    // ====================================

    // Double Solenoids


    armIntakeSolenoid = new DoubleSolenoid(
            0,
            1);


    // Single Solenoids


    // **********************************************************
    // ANALOG I/O CLASSES
    // **********************************************************
    // ====================================
    // Analog classes
    // ====================================

    // Gyro class

    // P/N ADW22307


    // Potentiometers

    delayPot = new RobotPotentiometer(
            2,
            270);

    intakeDeploySensor = new RobotPotentiometer(
            0,
            270);


    // Sonar/Ultrasonic
    frontUltraSonic = new LVMaxSonarEZ(
            3);

    // =====================================
    // SPI Bus
    // =====================================

    // Analog Interfaces
    gyro = new KilroySPIGyro(true);



    // **********************************************************
    // roboRIO CONNECTIONS CLASSES
    // **********************************************************

    // Axis/USB Camera class


    axisCamera = new VisionProcessor(
            "10.3.39.11", CameraModel.AXIS_M1013,
            ringLightRelay);

    // -------------------------------------
    // declare the USB camera server and the
    // USB camera it serves at the same time
    // -------------------------------------

    USBCam = CameraServer.getInstance()
            .startAutomaticCapture(0);

    USBCamII = CameraServer.getInstance()
            .startAutomaticCapture(1);




    // **********************************************************
    // DRIVER STATION CLASSES
    // **********************************************************

    // DriverStations class

    driverStation = DriverStation
            .getInstance();

    // Joystick classes
    leftDriver = new Joystick(0);

    rightDriver = new Joystick(1);

    leftOperator = new Joystick(2);

    rightOperator = new Joystick(3);

    // Buttons classes
    // ----- Left Operator -----

    // left trigger
    intakeTrigger = new JoystickButton(
            leftOperator, 1);

    outtakeButton = new JoystickButton(
            leftOperator, 2);

    intakeOverride = new JoystickButton(
            leftOperator, 3);

    deployOverride = new JoystickButton(
            leftOperator, 5);

    cargoShipCargoButton = new JoystickButton(
            leftOperator, 6);

    cargoShipHatchButton = new JoystickButton(
            leftOperator, 7);

    // ----- Right Operator -----

    chooseCargoRocketHeights = new JoystickButton(
            rightOperator, 4);

    forkliftOverride = new JoystickButton(
            rightOperator, 5);

    nextHigherForkliftTargetHeight = new JoystickButton(
            rightOperator, 6);

    nextLowerForkliftTargetHeight = new JoystickButton(
            rightOperator, 7);


    //
    // Momentary Switches
    //

    descendButton = new MomentarySwitch(
            leftOperator, 5, false);

    ringLightButton = new MomentarySwitch(
            leftOperator, 6, false);


    // **********************************************************
    // Kilroy's Ancillary classes
    // **********************************************************
    // PID tuneables
    // PID classes
    // Utility classes

    autoTimer = new Timer();

    deployTimer = new Timer();

    telemetry = new Telemetry(10000);

    // Transmission class
    transmission = new TankTransmission(
            new SpeedControllerGroup(leftFrontCANMotor,
                    leftRearCANMotor),
            new SpeedControllerGroup(rightFrontCANMotor,
                    rightRearCANMotor));

    // ------------------------------------
    // Drive system
    // ------------------------------------
    // @ANE
    drive = new Drive(transmission,
            leftFrontDriveEncoder, rightFrontDriveEncoder,
            // leftFrontDriveEncoder, rightFrontDriveEncoder,
            gyro);

    drivePID = new DrivePID(transmission,
            leftFrontDriveEncoder, rightFrontDriveEncoder,
            leftFrontDriveEncoder, rightFrontDriveEncoder, gyro);

    driveWithCamera = new DriveWithCamera(
            transmission, null, null, frontUltraSonic,
            frontUltraSonic, gyro, axisCamera);

    // Assembly classes (e.g. forklift)
    manipulator = new GamePieceManipulator(
            intakeDeployArm, intakeDeploySensor/* armEncoder */,
            armRoller,
            null/* photoSwitch */);

    lift = new Forklift(liftMotor,
            liftingEncoder,
            manipulator);

    climber = new ClimbToLevelTwo(
            armIntakeSolenoid, intakeDeployArm, intakeDeploySensor,
            drive, lift, frontUltraSonic);

    alignByTape = new AlignPerpendicularToTape(leftBackIR, rightBackIR,
            drive);

}

public static void robotInitialize2019 ()
{
    // **********************************************************
    // DIGITAL I/O CLASSES
    // **********************************************************

    // ====================================
    // PWM classes
    // ====================================

    // ----- Jaguar classes -----
    // ----- Talon classes -----
    // ----- Victor classes -----

    intakeDeployArm = new VictorSP(4);
    // hanging
    // ----- Servo classes -----


    // ====================================
    // CAN classes
    // ====================================
    pdp = new PowerDistributionPanel(2);

    liftMotor = new CANSparkMax(23, MotorType.kBrushless);

    rightFrontCANMotor = new CANSparkMax(14, MotorType.kBrushless);

    leftFrontCANMotor = new CANSparkMax(11, MotorType.kBrushless);

    rightRearCANMotor = new CANSparkMax(15, MotorType.kBrushless);

    leftRearCANMotor = new CANSparkMax(13, MotorType.kBrushless);

    armRoller = new CANSparkMax(10, MotorType.kBrushless);// fix CANID

    // ====================================
    // Relay classes
    // ====================================
    ringLightRelay = new Relay(0);

    // ====================================
    // Digital Inputs
    // ====================================
    // -------------------------------------
    // Single and double throw switches
    // -------------------------------------
    leftAutoSwitch = new SingleThrowSwitch(
            20);

    rightAutoSwitch = new SingleThrowSwitch(
            25);

    autoCenterSwitch = new DoubleThrowSwitch(
            leftAutoSwitch, rightAutoSwitch);

    levelOneSwitch = new SingleThrowSwitch(22);

    levelTwoSwitch = new SingleThrowSwitch(23);

    autoDisableSwitch = new DoubleThrowSwitch(levelOneSwitch,
            levelTwoSwitch);

    autoSixPosSwitch = new SixPositionSwitch(13, 14, 15, 16, 17, 18);



    // Gear Tooth Sensors


    // Encoders

    leftFrontDriveEncoder = new KilroyEncoder(
            4, 5);

    rightFrontDriveEncoder = new KilroyEncoder(
            6, 7);

    liftingEncoder = new KilroyEncoder(10,
            11);

    // -------------------------------------
    // Red Light/IR Sensor class
    // -------------------------------------

    armIR = new LightSensor(21);

    leftBackIR = new LightSensor(8);

    rightBackIR = new LightSensor(9);

    // ====================================
    // I2C Classes
    // ====================================

    // **********************************************************
    // SOLENOID I/O CLASSES
    // **********************************************************
    // ====================================
    // Compressor class - runs the compressor
    // ====================================
    compressor = new Compressor();

    // ====================================
    // Pneumatic Control Module
    // ====================================

    // ====================================
    // Solenoids
    // ====================================

    // Double Solenoids


    armIntakeSolenoid = new DoubleSolenoid(
            0,
            1);


    // Single Solenoids


    // **********************************************************
    // ANALOG I/O CLASSES
    // **********************************************************
    // ====================================
    // Analog classes
    // ====================================

    // Gyro class

    // P/N ADW22307


    // Potentiometers

    delayPot = new RobotPotentiometer(
            2,
            270);

    intakeDeploySensor = new RobotPotentiometer(
            0,
            270);


    // Sonar/Ultrasonic
    frontUltraSonic = new LVMaxSonarEZ(
            3);

    // =====================================
    // SPI Bus
    // =====================================

    // Analog Interfaces
    gyro = new KilroySPIGyro(true);



    // **********************************************************
    // roboRIO CONNECTIONS CLASSES
    // **********************************************************

    // Axis/USB Camera class


    axisCamera = new VisionProcessor(
            "10.3.39.11", CameraModel.AXIS_M1013,
            ringLightRelay);

    // -------------------------------------
    // declare the USB camera server and the
    // USB camera it serves at the same time
    // -------------------------------------

    USBCam = CameraServer.getInstance()
            .startAutomaticCapture(0);

    USBCamII = CameraServer.getInstance()
            .startAutomaticCapture(1);




    // **********************************************************
    // DRIVER STATION CLASSES
    // **********************************************************

    // DriverStations class

    driverStation = DriverStation
            .getInstance();

    // Joystick classes
    leftDriver = new Joystick(0);

    rightDriver = new Joystick(1);

    leftOperator = new Joystick(2);

    rightOperator = new Joystick(3);

    // Buttons classes
    // ----- Left Operator -----

    // left trigger
    intakeTrigger = new JoystickButton(
            leftOperator, 1);

    outtakeButton = new JoystickButton(
            leftOperator, 2);

    intakeOverride = new JoystickButton(
            leftOperator, 3);

    deployOverride = new JoystickButton(
            leftOperator, 5);

    cargoShipCargoButton = new JoystickButton(
            leftOperator, 6);

    cargoShipHatchButton = new JoystickButton(
            leftOperator, 7);

    // ----- Right Operator -----

    chooseCargoRocketHeights = new JoystickButton(
            rightOperator, 4);

    forkliftOverride = new JoystickButton(
            rightOperator, 5);

    nextHigherForkliftTargetHeight = new JoystickButton(
            rightOperator, 6);

    nextLowerForkliftTargetHeight = new JoystickButton(
            rightOperator, 7);


    //
    // Momentary Switches
    //

    descendButton = new MomentarySwitch(
            leftOperator, 5, false);

    ringLightButton = new MomentarySwitch(
            leftOperator, 6, false);

    // **********************************************************
    // Kilroy's Ancillary classes
    // **********************************************************
    // PID tuneables
    // PID classes
    // Utility classes

    autoTimer = new Timer();

    deployTimer = new Timer();

    telemetry = new Telemetry(10000);

    // Transmission class
    transmission = new TankTransmission(
            new SpeedControllerGroup(leftFrontCANMotor,
                    leftRearCANMotor),
            new SpeedControllerGroup(rightFrontCANMotor,
                    rightRearCANMotor));

    // ------------------------------------
    // Drive system
    // ------------------------------------
    drive = new Drive(transmission,
            leftFrontDriveEncoder, rightFrontDriveEncoder,
            // leftFrontDriveEncoder, rightFrontDriveEncoder,
            gyro);

    drivePID = new DrivePID(transmission,
            leftFrontDriveEncoder, rightFrontDriveEncoder,
            leftFrontDriveEncoder, rightFrontDriveEncoder, gyro);

    driveWithCamera = new DriveWithCamera(
            transmission, null, null, frontUltraSonic,
            frontUltraSonic, gyro, axisCamera);

    // Assembly classes (e.g. forklift)
    manipulator = new GamePieceManipulator(
            armMotor, intakeDeploySensor/* armEncoder */,
            armRoller,
            null/* photoSwitch */);

    lift = new Forklift(liftMotor,
            liftingEncoder,
            manipulator);

    climber = new ClimbToLevelTwo(
            armIntakeSolenoid, armMotor, intakeDeploySensor,
            drive, lift, frontUltraSonic);
}

} // end class
