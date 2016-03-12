/*
 * DoubleBuffer.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Jin
 */

#include <RobotUtils/DoubleBuffer.h>

DoubleBuffer::DoubleBuffer(BufferPriority type, double def, llvm::StringRef logKey) {
	m_type = type;
	m_def = def;
	m_logKey = logKey;
	m_value = def;
}

void DoubleBuffer::Write(double value, int priority) {
	//	Check if locked
	if (IsLocked()) {
		return;
	}

	//	Check priority
	if (priority < m_priority) {
		return;
	} else if (priority > m_priority) {
		m_value = value;
		m_priority = priority;
		return;
	}

	//	Check type
	switch (m_type) {
	case kMax:
		if (value > m_value) {
			m_value = value;
			m_priority = priority;
		}
		break;
	case kMin:
		if (value < m_value) {
			m_value = value;
			m_priority = priority;
		}
		break;
	case kAbsMax:
		if (fabs(value) > fabs(m_value)) {
			m_value = value;
			m_priority = priority;
		}
		break;
	case kAbsMin:
		if (fabs(value) < fabs(m_value)) {
			m_value = value;
			m_priority = priority;
		}
		break;
	}
}

double DoubleBuffer::Read() {
	double value = m_value;
	ResetState();
	if (!m_logKey.compare("")) {
		SmartDashboard::PutNumber(m_logKey, value);
	}

	return value;
}

void DoubleBuffer::ResetState() {
	m_value = m_def;
	m_priority = 0;
}

void DoubleBuffer::Lock() {
	m_locked = true;
	ResetState();
}

void DoubleBuffer::Unlock() {
	m_locked = false;
}

bool DoubleBuffer::IsLocked() {
	return m_locked;
}
