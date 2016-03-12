#ifndef SRC_HOTJOYSTICK_H_
#define SRC_HOTJOYSTICK_H_

#include "WPILib.h"

class HotJoystick;

class HotButton {
public:
	typedef enum {
		kA, kB, kX, kY, kLB, kRB, kBack, kStart, kLT, kRT
	} Button;

	HotButton(HotJoystick *joystick, uint32_t buttonID, bool isAxis = false, float threshold = 0.4);

	/*
	 * 	Getters
	 */
	uint32_t GetButtonID();
	bool IsAxis();

	/*
	 * 	Configuration
	 */
	void SetModifier(HotButton *modifier);

	/*
	 * 	Read
	 */
	bool ReadRaw();
	bool Read();
	bool Read(HotButton *modifier);

private:
	HotJoystick *m_joystick;
	uint32_t m_buttonID;
	bool f_axis;
	float m_threshold;

	std::vector<HotButton*> m_modifiers;
};

class HotJoystick: public Joystick {
public:
	HotJoystick(unsigned int port);

	HotButton* GetButton(HotButton::Button btn);

	/*
	 * 	Configuration
	 */
	void SetDeadband(float value);
	void SetModifier(HotButton::Button btn, HotButton::Button modifier);

	/*
	 * 	Read Button
	 */
	bool Button(HotButton* btn);
	bool Button(HotButton* btn, HotButton* modifier);

	bool ButtonA();
	bool ButtonA(HotButton* modifier);
	bool ButtonA(HotButton::Button modifier);

	bool ButtonB();
	bool ButtonB(HotButton* modifier);
	bool ButtonB(HotButton::Button modifier);

	bool ButtonX();
	bool ButtonX(HotButton* modifier);
	bool ButtonX(HotButton::Button modifier);

	bool ButtonY();
	bool ButtonY(HotButton* modifier);
	bool ButtonY(HotButton::Button modifier);

	bool ButtonLB();
	bool ButtonLB(HotButton* modifier);
	bool ButtonLB(HotButton::Button modifier);

	bool ButtonRB();
	bool ButtonRB(HotButton* modifier);
	bool ButtonRB(HotButton::Button modifier);

	bool ButtonBack();
	bool ButtonBack(HotButton* modifier);
	bool ButtonBack(HotButton::Button modifier);

	bool ButtonStart();
	bool ButtonStart(HotButton* modifier);
	bool ButtonStart(HotButton::Button modifier);

	bool ButtonLT();
	bool ButtonLT(HotButton* modifier);
	bool ButtonLT(HotButton::Button modifier);

	bool ButtonRT();
	bool ButtonRT(HotButton* modifier);
	bool ButtonRT(HotButton::Button modifier);

	/*
	 * 	read Axis
	 */
	float AxisLX();
	float AxisLY();
	float AxisRX();
	float AxisRY();
	float AxisLT();
	float AxisRT();

private:
	float m_deadband;
	HotButton *A, *B, *X, *Y, *LB, *RB, *Back, *Start, *LT, *RT;
};

#endif /* SRC_HOTJOYSTICK_H_ */
