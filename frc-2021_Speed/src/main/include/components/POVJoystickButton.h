#ifndef POV_JOYSTICK_BUTTON_H
#define POV_JOYSTICK_BUTTON_H

#include <frc2/command/button/Button.h>
#include <frc/GenericHID.h>

class POVJoystickButton : public frc2::Button {
public:
    POVJoystickButton(frc::GenericHID *joystick, uint32_t povNumber, int angle);
    virtual ~POVJoystickButton();

    virtual bool Get();

private:
    frc::GenericHID *m_pJoystick;
	uint32_t m_povNumber;
	int m_angle;
};

#endif // POV_JOYSTICK_BUTTON_H
