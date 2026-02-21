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
 * Parts of this file based upon the Poco C++ Libraries, which is Copyright (C) 
 * 2004-2006, Applied Informatics Software Engineering GmbH. and Contributors.
 */

#ifndef ES40_THREAD_H
#define ES40_THREAD_H

#include <thread>
#include <atomic>
#include <chrono>
#include <string>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <cstdio>
#include <stdexcept>

#include "Runnable.h"

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#else
#  include <pthread.h>
#  include <sched.h>
#  include <signal.h>
#endif

class CThread;

inline thread_local const char* tl_threadName = nullptr;
inline thread_local CThread* tl_currentThread = nullptr;

class CThread
{
public:
  enum Priority
  {
    PRIO_LOWEST = 0,
    PRIO_LOW = 1,
    PRIO_NORMAL = 2,
    PRIO_HIGH = 3,
    PRIO_HIGHEST = 4
  };

  CThread()
    : _id(uniqueId()), _name(makeName()), _prio(PRIO_NORMAL), _running(false)
  {
  }

  explicit CThread(const std::string& threadName)
    : _id(uniqueId()), _name(threadName), _prio(PRIO_NORMAL), _running(false)
  {
  }

  ~CThread()
  {
    if (_thread.joinable())
      _thread.detach();
  }

  int id() const { return _id; }

  std::string getName() const
  {
    std::lock_guard<std::mutex> lk(_nameMutex);
    return _name;
  }

  void setName(const std::string& n)
  {
    std::lock_guard<std::mutex> lk(_nameMutex);
    _name = n;
  }

  void setPriority(Priority prio)
  {
    if (prio == _prio)
      return;
    _prio = prio;
    if (_thread.joinable())
      applyPriority();
  }

  Priority getPriority() const { return _prio; }

  void start(CRunnable& target)
  {
    if (_thread.joinable())
      throw std::runtime_error("CThread::start: thread already running");

    _running.store(true, std::memory_order_release);

    auto stableName = std::make_shared<std::string>(_name);
    CThread* self = this;

    _thread = std::thread([&target, self, stableName]()
      {
        tl_threadName = stableName->c_str();
        tl_currentThread = self;

#if !defined(_WIN32)
        sigset_t sset;
        sigemptyset(&sset);
        sigaddset(&sset, SIGQUIT);
        sigaddset(&sset, SIGTERM);
        sigaddset(&sset, SIGPIPE);
        pthread_sigmask(SIG_BLOCK, &sset, 0);
#endif

        try
        {
          target.run();
        }
        catch (const std::exception& ex)
        {
          printf("Thread '%s' terminated with exception: %s\n",
            stableName->c_str(), ex.what());
        }
        catch (...)
        {
          printf("Thread '%s' terminated with an unknown exception.\n",
            stableName->c_str());
        }

        {
          std::lock_guard<std::mutex> lk(self->_doneMutex);
          self->_running.store(false, std::memory_order_release);
        }
        self->_doneCond.notify_all();
      });

    if (_prio != PRIO_NORMAL)
      applyPriority();
  }

  void join()
  {
    if (_thread.joinable())
      _thread.join();
  }

  void join(long milliseconds)
  {
    if (!tryJoin(milliseconds))
      throw std::runtime_error("CThread::join: timeout");
  }

  bool tryJoin(long milliseconds)
  {
    if (!_thread.joinable())
      return true;

    {
      std::unique_lock<std::mutex> lk(_doneMutex);
      if (!_doneCond.wait_for(lk, std::chrono::milliseconds(milliseconds),
        [this] { return !_running.load(std::memory_order_acquire); }))
      {
        return false;   // timeout
      }
    }

    if (_thread.joinable())
      _thread.join();
    return true;
  }

  bool isRunning() const
  {
    return _running.load(std::memory_order_acquire);
  }

  static void sleep(long milliseconds)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }

  static void yield()
  {
    std::this_thread::yield();
  }

  static CThread* current()
  {
    return tl_currentThread;
  }

private:
  static int uniqueId()
  {
    static std::atomic<int> counter{ 0 };
    return ++counter;
  }

  std::string makeName() const
  {
    std::ostringstream oss;
    oss << '#' << _id;
    return oss.str();
  }

#if defined(_WIN32)

  static int mapPrioWin32(Priority prio)
  {
    switch (prio)
    {
    case PRIO_LOWEST:   return THREAD_PRIORITY_LOWEST;
    case PRIO_LOW:      return THREAD_PRIORITY_BELOW_NORMAL;
    case PRIO_NORMAL:   return THREAD_PRIORITY_NORMAL;
    case PRIO_HIGH:     return THREAD_PRIORITY_ABOVE_NORMAL;
    case PRIO_HIGHEST:  return THREAD_PRIORITY_HIGHEST;
    default:            return THREAD_PRIORITY_NORMAL;
    }
  }

  void applyPriority()
  {
    HANDLE h = static_cast<HANDLE>(_thread.native_handle());
    if (!SetThreadPriority(h, mapPrioWin32(_prio)))
      printf("Warning: SetThreadPriority failed for thread '%s' (error %lu)\n",
        _name.c_str(), GetLastError());
  }

#else // POSIX

  static int mapPrioPosix(Priority prio)
  {
#if defined(__VMS) || defined(__digital__)
    static const int pmin = PRI_OTHER_MIN;
    static const int pmax = PRI_OTHER_MAX;
#else
    static const int pmin = sched_get_priority_min(SCHED_OTHER);
    static const int pmax = sched_get_priority_max(SCHED_OTHER);
#endif
    switch (prio)
    {
    case PRIO_LOWEST:   return pmin;
    case PRIO_LOW:      return pmin + (pmax - pmin) / 4;
    case PRIO_NORMAL:   return pmin + (pmax - pmin) / 2;
    case PRIO_HIGH:     return pmin + 3 * (pmax - pmin) / 4;
    case PRIO_HIGHEST:  return pmax;
    default:            return pmin + (pmax - pmin) / 2;
    }
  }

  void applyPriority()
  {
    struct sched_param par {};
    par.sched_priority = mapPrioPosix(_prio);
    int rc = pthread_setschedparam(
      _thread.native_handle(), SCHED_OTHER, &par);
    if (rc != 0)
      printf("Warning: pthread_setschedparam failed for thread '%s' (rc=%d).\n",
        _name.c_str(), rc);
  }

#endif // _WIN32 / POSIX

  int                         _id;
  std::string                 _name;
  Priority                    _prio;
  mutable std::mutex          _nameMutex;
  std::thread                 _thread;
  std::atomic<bool>           _running;

  std::mutex                  _doneMutex;
  std::condition_variable     _doneCond;

  CThread(const CThread&) = delete;
  CThread& operator=(const CThread&) = delete;
};

#endif // ES40_THREAD_H
