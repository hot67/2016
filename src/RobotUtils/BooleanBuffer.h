/*
 * BooleanBuffer.h
 *
 *  Created on: Mar 11, 2016
 *      Author: Jin
 */

#ifndef SRC_ROBOTUTILS_BOOLEANBUFFER_H_
#define SRC_ROBOTUTILS_BOOLEANBUFFER_H_

#include <string>
#include "WPILib.h"

class BooleanBuffer {
public:
	BooleanBuffer(bool def = false, llvm::StringRef logKey = "");

	/*
	 * 	Read Write
	 */
	void Write(bool def, int priority = 0);
	bool Read();

	void ResetState();

	void Lock();
	void Unlock();
	bool IsLocked();

private:
	/*
	 * 	Configuration
	 */
	bool m_def;
	llvm::StringRef m_logKey;

	/*
	 *  State
	 */
	bool m_value;
	int m_priority = 0;
	bool m_locked = false;
};

#endif /* SRC_ROBOTUTILS_BOOLEANBUFFER_H_ */
