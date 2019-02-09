package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Class used to solve a common problem with buttons in teleop where
 * the button only needs to return true for one loop
 *
 * Apparently this can be done with Momentary Switch, but it was not
 * documented or intuitize to figure out without having to declare
 * other variables/ work arounds
 *
 * NOTE: Mr. Brown does not want this to be used at the moment
 * if it can be avoided because he would prefer MomentarySwitch
 *
 * @author Cole
 *
 */
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

/**
 * Returns true if the button was just pressed, false otherwise.
 * Will only return true the first time teleop periodic calls this
 * method, false otherwise (no matter how long the associated button
 * is held, this method will only return true once)
 *
 * Should be called every loop of Teleop periodic (in place of the
 * associated button) in order to work properly
 */
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
        }
    else
        {
        this.isOn = false;
        this.wasPreviouslyOn = false;
        }
}

}
