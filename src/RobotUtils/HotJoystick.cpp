#include <RobotUtils/HotJoystick.h>

HotButton::HotButton(HotJoystick *joystick, uint32_t buttonID, bool isAxis, float threshold) {
	m_joystick = joystick;
	m_buttonID = buttonID;
	f_axis = isAxis;
	m_threshold = threshold;
}

uint32_t HotButton::GetButtonID() {
	return m_buttonID;
}

bool HotButton::IsAxis() {
	return f_axis;
}

void HotButton::SetModifier(HotButton *modifier) {
	m_modifiers.push_back(modifier);
}

bool HotButton::ReadRaw() {
	if (IsAxis()) {
		return m_joystick->GetRawAxis(GetButtonID()) > m_threshold;
	} else {
		return m_joystick->GetRawButton(GetButtonID());
	}
}

bool HotButton::Read() {
	//	When No other button is pressed
	if (!ReadRaw()) {
		return false;
	}

	for (unsigned int i = 0; i < m_modifiers.size(); i++) {
		if (m_modifiers[i]->ReadRaw()) {
			return false;
		}
	}

	return true;
}
bool HotButton::Read(HotButton *modifier) {
	if (!ReadRaw()) {
		return false;
	}

	if (!modifier->ReadRaw()) {
		return false;
	}

	for (unsigned int i = 0; i < m_modifiers.size(); i++) {
		if (!(m_modifiers[i]->GetButtonID() == modifier->GetButtonID() && m_modifiers[i]->IsAxis() == modifier->IsAxis()) && m_modifiers[i]->ReadRaw()) {
			return false;
		}
	}
	return true;
}


HotJoystick::HotJoystick(unsigned int port) : Joystick(port) {
	/*
	 * 	Initialize Buttons
	 */
	A = new HotButton(this, 1);
	B = new HotButton(this, 2);
	X = new HotButton(this, 3);
	Y = new HotButton(this, 4);
	LB = new HotButton(this, 5);
	RB = new HotButton(this, 6);
	Back = new HotButton(this, 7);
	Start = new HotButton(this, 8);
	LT = new HotButton(this, 2, true, 0.4);
	RT = new HotButton(this, 3, true, 0.4);

	m_deadband = 0.4;
}

HotButton* HotJoystick::GetButton(HotButton::Button btn) {
	switch (btn) {
	case HotButton::kA:
		return A;
		break;
	case HotButton::kB:
		return B;
		break;
	case HotButton::kX:
		return X;
		break;
	case HotButton::kY:
		return Y;
		break;
	case HotButton::kLB:
		return LB;
		break;
	case HotButton::kRB:
		return RB;
		break;
	case HotButton::kBack:
		return Back;
		break;
	case HotButton::kStart:
		return Start;
		break;
	case HotButton::kLT:
		return LT;
		break;
	case HotButton::kRT:
		return RT;
		break;
	}
	return A;
}

/**
 * 	Configuration
 */
void HotJoystick::SetDeadband(float threshold) {
	m_deadband = threshold;
}

void HotJoystick::SetModifier(HotButton::Button btn, HotButton::Button modifier) {
	GetButton(btn)->SetModifier(GetButton(modifier));
}

/*
 * 	Read Button
 */
bool HotJoystick::Button(HotButton* btn) {
	return btn->Read();
}
bool HotJoystick::Button(HotButton* btn, HotButton* modifier) {
	return btn->Read(modifier);
}

bool HotJoystick::ButtonA() {
	return Button(A);
}
bool HotJoystick::ButtonA(HotButton* modifier) {
	return Button(A, modifier);
}
bool HotJoystick::ButtonA(HotButton::Button btn) {
	return Button(A, GetButton(btn));
}
bool HotJoystick::ButtonB() {
	return Button(B);
}
bool HotJoystick::ButtonB(HotButton* modifier) {
	return Button(B, modifier);
}
bool HotJoystick::ButtonB(HotButton::Button btn) {
	return Button(B, GetButton(btn));
}
bool HotJoystick::ButtonX() {
	return Button(X);
}
bool HotJoystick::ButtonX(HotButton* modifier) {
	return Button(X, modifier);
}
bool HotJoystick::ButtonX(HotButton::Button btn) {
	return Button(X, GetButton(btn));
}
bool HotJoystick::ButtonY() {
	return Button(Y);
}
bool HotJoystick::ButtonY(HotButton* modifier) {
	return Button(Y, modifier);
}
bool HotJoystick::ButtonY(HotButton::Button btn) {
	return Button(Y, GetButton(btn));
}
bool HotJoystick::ButtonLB() {
	return Button(LB);
}
bool HotJoystick::ButtonLB(HotButton* modifier) {
	return Button(LB, modifier);
}
bool HotJoystick::ButtonLB(HotButton::Button btn) {
	return Button(LB, GetButton(btn));
}
bool HotJoystick::ButtonRB() {
	return Button(RB);
}
bool HotJoystick::ButtonRB(HotButton* modifier) {
	return Button(RB, modifier);
}
bool HotJoystick::ButtonRB(HotButton::Button btn) {
	return Button(RB, GetButton(btn));
}
bool HotJoystick::ButtonBack() {
	return Button(Back);
}
bool HotJoystick::ButtonBack(HotButton* modifier) {
	return Button(Back, modifier);
}
bool HotJoystick::ButtonBack(HotButton::Button btn) {
	return Button(Back, GetButton(btn));
}
bool HotJoystick::ButtonStart() {
	return Button(Start);
}
bool HotJoystick::ButtonStart(HotButton* modifier) {
	return Button(Start, modifier);
}
bool HotJoystick::ButtonStart(HotButton::Button btn) {
	return Button(Start, GetButton(btn));
}
bool HotJoystick::ButtonLT() {
	return Button(LT);
}
bool HotJoystick::ButtonLT(HotButton* modifier) {
	return Button(LT, modifier);
}
bool HotJoystick::ButtonLT(HotButton::Button btn) {
	return Button(LT, GetButton(btn));
}
bool HotJoystick::ButtonRT() {
	return Button(RT);
}
bool HotJoystick::ButtonRT(HotButton* modifier) {
	return Button(RT, modifier);
}
bool HotJoystick::ButtonRT(HotButton::Button btn) {
	return Button(RT, GetButton(btn));
}

float HotJoystick::AxisLX() {
	return fabs(GetRawAxis(0)) > m_deadband ? GetRawAxis(0) : 0.0;
}
float HotJoystick::AxisLY() {
	return fabs(GetRawAxis(1)) > m_deadband ? GetRawAxis(1) : 0.0;
}
float HotJoystick::AxisLT() {
	return fabs(GetRawAxis(2)) > m_deadband ? GetRawAxis(2) : 0.0;
}
float HotJoystick::AxisRT() {
	return fabs(GetRawAxis(3)) > m_deadband ? GetRawAxis(3) : 0.0;
}
float HotJoystick::AxisRX() {
	return fabs(GetRawAxis(4)) > m_deadband ? GetRawAxis(4) : 0.0;
}
float HotJoystick::AxisRY() {
	return fabs(GetRawAxis(5)) > m_deadband ? GetRawAxis(5) : 0.0;
}
