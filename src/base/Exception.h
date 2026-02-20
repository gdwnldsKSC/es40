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

#ifndef ES40_EXCEPTION_H
#define ES40_EXCEPTION_H

#include <exception>
#include <string>

class CException : public std::exception
{
public:
  CException(const std::string& msg, int code = 0);
  CException(const std::string& msg, const std::string& arg, int code = 0);
  CException(const CException& other);
  ~CException() noexcept override;

  CException& operator=(const CException& other);

  virtual const char* name() const noexcept;
  const char* what() const noexcept override;
  const std::string& message() const noexcept { return _msg; }
  int                 code()    const noexcept { return _code; }

  std::string displayText() const;

protected:
  CException(int code = 0);

private:
  std::string _msg;
  int         _code;
};

#define ES40_DECLARE_EXCEPTION(CLS, BASE)                                   \
  class CLS : public BASE {                                                 \
  public:                                                                   \
    CLS(int code = 0)                              : BASE(code)          {} \
    CLS(const std::string& msg, int code = 0)      : BASE(msg, code)    {} \
    CLS(const std::string& msg,                                             \
        const std::string& arg, int code = 0)      : BASE(msg, arg, code){}\
    const char* name() const noexcept override;                             \
  };

#define ES40_IMPLEMENT_EXCEPTION(CLS, NAME)                                 \
  const char* CLS::name() const noexcept { return NAME; }

ES40_DECLARE_EXCEPTION(CLogicException, CException)
ES40_DECLARE_EXCEPTION(CAssertionViolationException, CLogicException)
ES40_DECLARE_EXCEPTION(CNullPointerException, CLogicException)
ES40_DECLARE_EXCEPTION(CBugcheckException, CLogicException)
ES40_DECLARE_EXCEPTION(CInvalidArgumentException, CLogicException)
ES40_DECLARE_EXCEPTION(CNotImplementedException, CLogicException)
ES40_DECLARE_EXCEPTION(CIllegalStateException, CLogicException)

ES40_DECLARE_EXCEPTION(CRuntimeException, CException)
ES40_DECLARE_EXCEPTION(CSystemException, CRuntimeException)
ES40_DECLARE_EXCEPTION(COutOfMemoryException, CRuntimeException)
ES40_DECLARE_EXCEPTION(CIOException, CRuntimeException)
ES40_DECLARE_EXCEPTION(CFileException, CIOException)
ES40_DECLARE_EXCEPTION(CFileNotFoundException, CFileException)

ES40_DECLARE_EXCEPTION(CConfigurationException, CException)
ES40_DECLARE_EXCEPTION(CThreadException, CException)
ES40_DECLARE_EXCEPTION(CWin32Exception, CException)
ES40_DECLARE_EXCEPTION(CSDLException, CException)
ES40_DECLARE_EXCEPTION(CGracefulException, CException)
ES40_DECLARE_EXCEPTION(CAbortException, CException)

#endif // Foundation_Exception_INCLUDED
