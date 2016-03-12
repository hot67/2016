/*
 * BooleanBuffer.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Jin
 */

#include <RobotUtils/BooleanBuffer.h>

BooleanBuffer::BooleanBuffer(bool def, llvm::StringRef logKey) {
	m_def = def;
	m_logKey = logKey;
	m_value = def;
}

void BooleanBuffer::Write(bool value, int priority) {
	//	Lock
	if (IsLocked()) {
		return;
	}

	//	Check Priority
	if (priority < m_priority) {
		return;
	} else if (priority > m_priority) {
		m_value = value;
		m_priority = priority;
		return;
	}

	if (value != m_def) {
		m_value = value;
		m_priority = priority;
	}
}

bool BooleanBuffer::Read() {
	bool value = m_value;
	ResetState();
	if (!m_logKey.compare("")) {
		SmartDashboard::PutBoolean(m_logKey, value);
	}
	return value;
	return m_value;
}

void BooleanBuffer::ResetState() {
	m_value = m_def;
	m_priority = 0;
}

void BooleanBuffer::Lock() {
	m_locked = true;
}

void BooleanBuffer::Unlock() {
	m_locked = false;
}

bool BooleanBuffer::IsLocked() {
	return m_locked;
}
