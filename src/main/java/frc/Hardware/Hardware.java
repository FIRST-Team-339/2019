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

import frc.HardwareInterfaces.DoubleThrowSwitch;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.SingleThrowSwitch;
import frc.Utils.Telemetry;
import frc.vision.VisionProcessor;
import frc.vision.VisionProcessor.CameraModel;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

/**
 * -------------------------------------------------------
 * puts all of the hardware declarations into one place. In addition, it makes
 * them available to both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 * @written Jan 2, 2011
 * -------------------------------------------------------
 */

public class Hardware
{
// ------------------------------------
// Public Constants
// ------------------------------------
public enum robotYear 
{
        KILROY2018,
        KILROY2019
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
public static Talon rightDriveMotor = new Talon(2);
//
public static Talon leftDriveMotor = new Talon(3);

// ------------------------------------
// Victor Classes
// ------------------------------------

// ------------------------------------
// Servo classes
// ------------------------------------

// ====================================
// CAN classes
// ====================================
public static PowerDistributionPanel pdp = new PowerDistributionPanel(
        2);


// ====================================
// Relay classes
// ====================================

// ====================================
// Digital Inputs
// ====================================
// ------------------------------------
// Single and double throw switches
// ------------------------------------
public static SingleThrowSwitch leftAutoSwitch = new SingleThrowSwitch(
        20);

public static SingleThrowSwitch rightAutoSwitch = new SingleThrowSwitch(
        25);

public static DoubleThrowSwitch disableAutonomousSwitch = new DoubleThrowSwitch(
        leftAutoSwitch, rightAutoSwitch);

public static SingleThrowSwitch demoModeSwitch = new SingleThrowSwitch(
        8);

// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------
public static KilroyEncoder leftFrontDriveEncoder = new KilroyEncoder(
        14, 15);

public static KilroyEncoder rightFrontDriveEncoder = new KilroyEncoder(
        16, 17);

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

// --------------------------------------
// Potentiometers
// --------------------------------------
public static RobotPotentiometer delayPot = new RobotPotentiometer(2,
        300);

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------

// =====================================
// SPI Bus
// =====================================

// -------------------------------------
// Analog Interfaces
// -------------------------------------

// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------

public static VisionProcessor axisCamera = new VisionProcessor(
        "10.3.39.11", CameraModel.AXIS_M1013);

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------
public static UsbCamera USBCam = CameraServer.getInstance()
        .startAutomaticCapture(0);

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

// ------------------------------------
// Drive system
// ------------------------------------

// -------------------
// Assembly classes (e.g. forklift)
// -------------------

} // end class
