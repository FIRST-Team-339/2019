package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class QuickSwitch
{

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
    this.update();
    return isOn;
}

private void update ()
{
    if (this.button.get() == true)
        {
        if (this.wasPreviouslyOn == false)
            this.isOn = true;
        else
            this.isOn = false;

        wasPreviouslyOn = true;
        } else
        {
        this.isOn = false;
        this.wasPreviouslyOn = false;
        }
}

}
