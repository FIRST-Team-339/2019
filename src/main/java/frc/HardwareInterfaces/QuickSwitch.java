package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class QuickSwitch
{

Joystick joystick;

int buttonNumber;

JoystickButton button;

boolean isOn = false;

boolean wasPreviouslyOn = false;


public QuickSwitch (Joystick joystick, int buttonNumber)
{
    this.button = new JoystickButton(joystick, buttonNumber);
}

public QuickSwitch (JoystickButton button)
{
    this.button = button;
}

public boolean getCurrentValue ()
{

    return isOn;
}

public void update ()
{

}

}
