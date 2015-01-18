#include "AdvancedJoystick.h"

/**** CONSTRUCTORS ****/
AdvancedJoystick::AdvancedJoystick (Joystick* gamepad)
{
	m_gamepad = gamepad;
	m_timer = new Timer;

    m_deadband = JOYSTICK_DEADBAND;
    m_deadbandType = kNone;

    m_buttonTimeout = JOYSTICK_TIMEOUT;
	
	m_timer->Reset();
}

AdvancedJoystick::AdvancedJoystick (Joystick* gamepad, deadband_t deadbandType, float deadband, float timeout)
{
    m_gamepad = gamepad;
    m_timer = new Timer;

    m_deadband = deadband;
    m_deadbandType = deadbandType;

    m_buttonTimeout = timeout;

    m_timer->Reset();
}

AdvancedJoystick::AdvancedJoystick (Joystick* gamepad, deadband_t deadbandType)
{
    m_gamepad = gamepad;
    m_timer = new Timer;

    m_deadband = JOYSTICK_DEADBAND;
    m_deadbandType = deadbandType;

    m_buttonTimeout = JOYSTICK_TIMEOUT;

    m_timer->Reset();
}

AdvancedJoystick::AdvancedJoystick (Joystick* gamepad, float deadband, float timeout)
{
    m_gamepad = gamepad;
    m_timer = new Timer;

    m_deadband = deadband;

    if (m_deadband != 0)
        m_deadbandType = kFlat;
    else
        m_deadbandType = kNone;

    m_buttonTimeout = timeout;

    m_timer->Reset();
}

AdvancedJoystick::AdvancedJoystick (int gamepad)
{
	m_gamepad = new Joystick (gamepad);
	m_timer = new Timer;

    m_deadband = JOYSTICK_DEADBAND;
    m_deadbandType = kNone;

    m_buttonTimeout = JOYSTICK_TIMEOUT;
	
	m_timer->Reset();
}

AdvancedJoystick::AdvancedJoystick (int gamepad, deadband_t deadbandType, float deadband, float timeout)
{
    m_gamepad = new Joystick (gamepad);
    m_timer = new Timer;

    m_deadband = deadband;
    m_deadbandType = deadbandType;

    m_buttonTimeout = timeout;

    m_timer->Reset();
}

AdvancedJoystick::AdvancedJoystick (int gamepad, deadband_t deadbandType)
{
    m_gamepad = new Joystick (gamepad);
    m_timer = new Timer;

    m_deadband = JOYSTICK_DEADBAND;
    m_deadbandType = deadbandType;

    m_buttonTimeout = JOYSTICK_TIMEOUT;

    m_timer->Reset();
}

AdvancedJoystick::AdvancedJoystick (int gamepad, float deadband, float timeout)
{
    m_gamepad = new Joystick (gamepad);
    m_timer = new Timer;

    m_deadband = deadband;

    if (m_deadband != 0)
        m_deadbandType = kFlat;
    else
        m_deadbandType = kNone;

    m_buttonTimeout = timeout;

    m_timer->Reset();
}

/**** JOYSTICK ACCESS FUNCTIONS ****/

bool AdvancedJoystick::GetRawButton (button_t channel) {
    update();

    if (channel < 10)
        return m_gamepad->GetRawButton(channel);
    else
    {
        if (channel == AdvancedJoystick::kTriggerL)
        {
            return GetRawAxis(AdvancedJoystick::kLeftTrigger) > 0.4;
        }
        else if (channel == AdvancedJoystick::kTriggerR)
        {
            return GetRawAxis(AdvancedJoystick::kRightTrigger) > 0.4;
        }
    }
}

bool AdvancedJoystick::GetButtonPress (button_t channel) {
    update();

    if (GetRawButton(channel) && (m_timer->Get() == 0.0))
    {
        m_timer->Start();
        return true;
    }
    else
        return false;

}

bool AdvancedJoystick::GetButtonPress_new (button_t channel)
{
    update();

    if (m_gamepad->GetRawButton(channel) && !isPressed(channel))
    {
        f_pressedButtons.push_back(channel);
        return true;
    }
    else
        return false;

}

float AdvancedJoystick::GetRawAxis (axis_t channel) {
    update();

    if ((channel < 2) || (channel > 3))
        return applyDeadband(m_gamepad->GetRawAxis(channel));
    else
        return m_gamepad->GetRawAxis(channel);
}

/**** INTERNAL FUNCTIONS ****/

float AdvancedJoystick::applyDeadband (float input) {
    if (m_deadbandType == kFlat)
        return applyDeadbandFlat(input);
    else if (m_deadbandType == kQuad)
        return applyDeadbandQuad(input);
    else if (m_deadbandType == kCube)
        return applyDeadbandQuad(input);
    else
        return input;
}

float AdvancedJoystick::applyDeadbandFlat (float input)
{
    if (input < -m_deadband) {
        return input;
    } else if (input < m_deadband) {
        return 0.0;
    } else {
        return input;
    }
}

float AdvancedJoystick::applyDeadbandQuad (float input)
{
    if (fabs(input) < m_deadband)
        return 0.0;
    else if (input < 0)
        return -pow(((input-m_deadband)*(1/(1-m_deadband))),2);
    else
        return pow(((input-m_deadband)*(1/(1-m_deadband))),2);
}

float AdvancedJoystick::applyDeadbandCube (float input)
{
    if (fabs(input) < m_deadband)
        return 0.0;
    else if (input < 0)
        return -pow(((input-m_deadband)*(1/(1-m_deadband))),2);
    else
        return pow(((input-m_deadband)*(1/(1-m_deadband))),2);

}

void AdvancedJoystick::trackTimer () {
    if (m_timer->HasPeriodPassed(m_buttonTimeout))
	{
		m_timer->Stop();
		m_timer->Reset();
	}
}

void AdvancedJoystick::trackPresses ()
{
    for (unsigned int x = 0; x < f_pressedButtons.size(); x++)
    {
        if (!GetRawButton(f_pressedButtons[x]))
            f_pressedButtons.erase(f_pressedButtons.begin()+x);
    }
}

void AdvancedJoystick::update ()
{
    trackTimer();
    trackPresses();
}

bool AdvancedJoystick::isPressed (button_t button)
{
    bool buttonFlag;
    buttonFlag = false;

    for (unsigned int x = 0; x < f_pressedButtons.size(); x++)
    {
        if (f_pressedButtons[x] == button)
            buttonFlag = true;
    }

    return buttonFlag;
}
