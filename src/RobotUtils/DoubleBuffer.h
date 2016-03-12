/*
 * DoubleBuffer.h
 *
 *  Created on: Mar 11, 2016
 *      Author: Jin
 */

#ifndef SRC_ROBOTUTILS_DOUBLEBUFFER_H_
#define SRC_ROBOTUTILS_DOUBLEBUFFER_H_

#include <string>
#include "WPILib.h"

typedef enum {
	kMax,
	kMin,
	kAbsMax,
	kAbsMin
} BufferPriority;

class DoubleBuffer {
public:
	DoubleBuffer(BufferPriority type, double def = 0.0, llvm::StringRef logKey = "");

	/*
	 * 	Read Write
	 */
	void Write(double value, int priority = 0);
	double Read();

	/*
	 * 	Reset status
	 */
	void ResetState();

	/*
	 * 	Lock
	 */
	void Lock();
	void Unlock();

	bool IsLocked();
private:
	/*
	 * 	Configuration
	 */
	BufferPriority m_type;
	double m_def;
	llvm::StringRef m_logKey;

	/*
	 * 	State
	 */
	double m_value;
	int m_priority = 0;
	bool m_locked = false;
};

#endif /* SRC_ROBOTUTILS_DOUBLEBUFFER_H_ */
