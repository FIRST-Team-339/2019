// ====================================================================
// FILE NAME: Hardware.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 2, 2011
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file contains all of the global definitions for the
// hardware objects in the system
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
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.SingleThrowSwitch;
import frc.HardwareInterfaces.SixPositionSwitch;
import frc.Utils.Telemetry;
import frc.Utils.drive.Drive;
import frc.Utils.drive.DrivePID;
import frc.vision.VisionProcessor;
import frc.vision.VisionProcessor.CameraModel;
import frc.HardwareInterfaces.Transmission.TankTransmission;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * ------------------------------------------------------- puts all of the
 * hardware declarations into one place. In addition, it makes them available to
 * both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */

public class Hardware {
    // ------------------------------------
    // Public Constants
    // ------------------------------------
    public enum robotYear {
        KILROY2018, KILROY2019
    }

    public robotYear whichRobot = robotYear.KILROY2018;

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
    // public static Talon rightDriveMotor = new Talon(2);// on CAN now

    // public static Talon leftDriveMotor = new Talon(3);// on CAN now

    // ------------------------------------
    // Victor Classes
    // ------------------------------------

    // public static VictorSP liftingMotor = new VictorSP(0);//on CAN now

    public static VictorSP cubeIntakeMotor = new VictorSP(1);// left intake on CAN

    public static VictorSP intakeDeployArm = new VictorSP(4); // hanging

    // ------------------------------------
    // Servo classes
    // ------------------------------------

    // ====================================
    // CAN classes
    // ====================================
    public static PowerDistributionPanel pdp = new PowerDistributionPanel(2);
    public static WPI_TalonSRX liftMotorOne = new WPI_TalonSRX(23);
    // CAN version

    public static WPI_TalonSRX liftMotorTwo = new WPI_TalonSRX(6);
    // CAN version

    public static WPI_TalonSRX rightFrontCANMotor = new WPI_TalonSRX(14);

    public static WPI_TalonSRX leftFrontCANMotor = new WPI_TalonSRX(11);

    public static WPI_TalonSRX rightRearCANMotor = new WPI_TalonSRX(12);
    // TODO - fix number

    public static WPI_TalonSRX leftRearCANMotor = new WPI_TalonSRX(13);
    // TODO - fix number

    // ====================================
    // Relay classes
    // ====================================

    public static DigitalOutput ringLightRelay = new DigitalOutput(0);

    // ====================================
    // Digital Inputs
    // ====================================
    // ------------------------------------
    // Single and double throw switches
    // ------------------------------------
    public static SingleThrowSwitch leftAutoSwitch = new SingleThrowSwitch(20);

    public static SingleThrowSwitch rightAutoSwitch = new SingleThrowSwitch(25);

    public static DoubleThrowSwitch disableAutonomousSwitch = new DoubleThrowSwitch(leftAutoSwitch, rightAutoSwitch);

    public static DoubleThrowSwitch levelStartSwitch = new DoubleThrowSwitch(18, 13); // false port numbers

    public static SixPositionSwitch autoSixPosSwitch = new SixPositionSwitch(1, 2, 3, 4, 5, 6);

    // ------------------------------------
    // Gear Tooth Sensors
    // ------------------------------------

    // ------------------------------------
    // Encoders
    // ------------------------------------
    public static KilroyEncoder leftFrontDriveEncoder = new KilroyEncoder(14, 15);

    public static KilroyEncoder rightFrontDriveEncoder = new KilroyEncoder(16, 17);

    // public static KilroyEncoder liftingEncoder = new KilroyEncoder(18, 19);

    // public static KilroyEncoder intakeDeployEncoder = new KilroyEncoder(23,
    // 24);// being removed???

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

    public static LightSensor armIR = new LightSensor(21);
    // TODO check port for 2018 robot

    public static LightSensor redLight = new LightSensor(7);

    // ====================================
    // I2C Classes
    // ====================================

    // **********************************************************
    // SOLENOID I/O CLASSES
    // **********************************************************
    // ====================================
    // Compressor class - runs the compressor
    // ====================================
    public static Compressor compressor = new Compressor();

    // ====================================
    // Pneumatic Control Module
    // ====================================

    // ====================================
    // Solenoids
    // ====================================
    // ------------------------------------
    // Double Solenoids
    // ------------------------------------

    public static DoubleSolenoid armIntakeSolenoid = new DoubleSolenoid(0, 1);

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
    public static RobotPotentiometer delayPot = new RobotPotentiometer(2, 300);

    // -------------------------------------
    // Sonar/Ultrasonic
    // -------------------------------------
    public static LVMaxSonarEZ frontUltraSonic = new LVMaxSonarEZ(3);

    // =====================================
    // SPI Bus
    // =====================================

    // -------------------------------------
    // Analog Interfaces
    // -------------------------------------
    public static KilroySPIGyro gyro = new KilroySPIGyro(true);

    // **********************************************************
    // roboRIO CONNECTIONS CLASSES
    // **********************************************************
    // -------------------------------------
    // Axis/USB Camera class
    // -------------------------------------

    public static VisionProcessor axisCamera = new VisionProcessor("10.3.39.11", CameraModel.AXIS_M1013,
            ringLightRelay);

    // -------------------------------------
    // declare the USB camera server and the
    // USB camera it serves at the same time
    // -------------------------------------
    public static UsbCamera USBCam = CameraServer.getInstance().startAutomaticCapture(0);

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
    public static DriverStation driverStation = DriverStation.getInstance();

    // ------------------------------------
    // Joystick classes
    // ------------------------------------
    public static Joystick leftDriver = new Joystick(0);

    public static Joystick rightDriver = new Joystick(1);

    public static Joystick leftOperator = new Joystick(2);

    public static Joystick rightOperator = new Joystick(3);

    // ------------------------------------
    // Momentary Switches
    // ------------------------------------

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
    public static final Timer autoTimer = new Timer();

    public static final Timer deployTimer = new Timer();

    public static final Telemetry telemetry = new Telemetry(10000);

    // ------------------------------------
    // Transmission class
    // ------------------------------------
    public static TankTransmission transmission = new TankTransmission(
            new SpeedControllerGroup(leftFrontCANMotor, leftRearCANMotor),
            new SpeedControllerGroup(rightFrontCANMotor, rightRearCANMotor));

    // ------------------------------------
    // Drive system
    // ------------------------------------
    public static Drive drive = new Drive(transmission, leftFrontDriveEncoder, rightFrontDriveEncoder,
            // leftFrontDriveEncoder, rightFrontDriveEncoder,
            gyro);

    // public static DrivePID drivePID = new DrivePID(transmission,
    // leftFrontDriveEncoder, rightFrontDriveEncoder,
    // leftFrontDriveEncoder, rightFrontDriveEncoder, gyro);
    // TODO CHANGE TO FRONT ENCODERS ON REAL ROBOT

    // TODO update with encoders once fixed
    public static DriveWithCamera driveWithCamera = new DriveWithCamera(transmission, null, null, frontUltraSonic,
            frontUltraSonic, gyro, axisCamera);

    // -------------------
    // Assembly classes (e.g. forklift)
    // -------------------

} // end class
