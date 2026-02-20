/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * WWW    : https://github.com/gdwnldsKSC/es40
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Although this is not required, the author would appreciate being notified of,
 * and receiving any modifications you may make to the source code that might serve
 * the general public.
 *
 */

#include "Exception.h"

CException::CException(int code) : _code(code) {}

CException::CException(const std::string& msg, int code)
	: _msg(msg), _code(code) {
}

CException::CException(const std::string& msg, const std::string& arg, int code)
	: _msg(msg), _code(code)
{
	if (!arg.empty()) { _msg += ": "; _msg += arg; }
}

CException::CException(const CException& other)
	: std::exception(other), _msg(other._msg), _code(other._code) {}

CException::~CException() noexcept = default;

CException& CException::operator=(const CException& other)
{
	if (this != &other) { _msg = other._msg; _code = other._code; }
	return *this;
}

const char* CException::name() const noexcept { return "Exception"; }
const char* CException::what() const noexcept { return name(); }

std::string CException::displayText() const
{
	std::string txt = name();
	if (!_msg.empty()) { txt += ": "; txt += _msg; }
	return txt;
}

ES40_IMPLEMENT_EXCEPTION(CLogicException, "Logic exception")
ES40_IMPLEMENT_EXCEPTION(CAssertionViolationException, "Assertion violation")
ES40_IMPLEMENT_EXCEPTION(CNullPointerException, "Null pointer")
ES40_IMPLEMENT_EXCEPTION(CBugcheckException, "Bugcheck")
ES40_IMPLEMENT_EXCEPTION(CInvalidArgumentException, "Invalid argument")
ES40_IMPLEMENT_EXCEPTION(CNotImplementedException, "Not implemented exception")
ES40_IMPLEMENT_EXCEPTION(CIllegalStateException, "Illegal state")

ES40_IMPLEMENT_EXCEPTION(CRuntimeException, "Runtime exception")
ES40_IMPLEMENT_EXCEPTION(CSystemException, "System exception")
ES40_IMPLEMENT_EXCEPTION(COutOfMemoryException, "Out of memory")
ES40_IMPLEMENT_EXCEPTION(CIOException, "I/O error")
ES40_IMPLEMENT_EXCEPTION(CFileException, "File access error")
ES40_IMPLEMENT_EXCEPTION(CFileNotFoundException, "File not found")

ES40_IMPLEMENT_EXCEPTION(CConfigurationException, "Configuration error")
ES40_IMPLEMENT_EXCEPTION(CThreadException, "Threading error")
ES40_IMPLEMENT_EXCEPTION(CWin32Exception, "Win32 error")
ES40_IMPLEMENT_EXCEPTION(CSDLException, "SDL error")
ES40_IMPLEMENT_EXCEPTION(CGracefulException, "Graceful exit")
ES40_IMPLEMENT_EXCEPTION(CAbortException, "Abort requested")
