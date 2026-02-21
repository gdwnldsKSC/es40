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

#ifndef ES40_MUTEX_H
#define ES40_MUTEX_H

#include <mutex>
#include <chrono>
#include <thread>
#include <cstdio>
#include <stdexcept>

#if defined(NO_LOCK_TIMEOUTS)
#if !defined(LOCK_TIMEOUT_MS)
#define LOCK_TIMEOUT_MS
#endif
#else
#if !defined(LOCK_TIMEOUT_MS)
#define LOCK_TIMEOUT_MS 5000
#endif
#endif

extern thread_local const char* tl_threadName;

#define CURRENT_THREAD_NAME (tl_threadName ? tl_threadName : "main")

/// Recursive mutex with optional debug logging..
class CMutex
{
public:
  using ScopedLock = std::lock_guard<CMutex>;

  CMutex() : lockName("?") {}
  explicit CMutex(const char* lName) : lockName(lName) {}
  ~CMutex() = default;

  void lock()
  {
#if defined(DEBUG_LOCKS)
    printf("        LOCK mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
    _mutex.lock();
#if defined(DEBUG_LOCKS)
    printf("      LOCKED mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
  }

  void lock(long milliseconds)
  {
#if defined(DEBUG_LOCKS)
    printf("   TIMED LOCK mutex %s from thread %s (%ld ms).\n", lockName, CURRENT_THREAD_NAME, milliseconds);
#endif
    if (!tryLock(milliseconds))
    {
      printf("TIMEOUT locking mutex %s from thread %s after %ld ms.\n",
        lockName, CURRENT_THREAD_NAME, milliseconds);
      throw std::runtime_error("Mutex lock timeout");
    }
#if defined(DEBUG_LOCKS)
    printf("      LOCKED mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
  }

  bool tryLock() { return _mutex.try_lock(); }

  bool tryLock(long milliseconds)
  {
    auto deadline = std::chrono::steady_clock::now()
      + std::chrono::milliseconds(milliseconds);
    do
    {
      if (_mutex.try_lock())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void unlock()
  {
#if defined(DEBUG_LOCKS)
    printf("      UNLOCK mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
    _mutex.unlock();
  }

  const char* lockName;

private:
  std::recursive_mutex _mutex;
  CMutex(const CMutex&) = delete;
  CMutex& operator=(const CMutex&) = delete;
};

/// Non-recursive timed mutex (lighter weight than CMutex).
class CFastMutex
{
public:
  using ScopedLock = std::lock_guard<CFastMutex>;

  CFastMutex() : lockName("?") {}
  explicit CFastMutex(const char* lName) : lockName(lName) {}
  ~CFastMutex() = default;

  void lock()
  {
#if defined(DEBUG_LOCKS)
    printf("        LOCK fast-mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
    _mutex.lock();
#if defined(DEBUG_LOCKS)
    printf("      LOCKED fast-mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
  }

  void lock(long milliseconds)
  {
#if defined(DEBUG_LOCKS)
    printf("   TIMED LOCK fast-mutex %s from thread %s (%ld ms).\n", lockName, CURRENT_THREAD_NAME, milliseconds);
#endif
    if (!tryLock(milliseconds))
    {
      printf("TIMEOUT locking fast-mutex %s from thread %s after %ld ms.\n",
        lockName, CURRENT_THREAD_NAME, milliseconds);
      throw std::runtime_error("FastMutex lock timeout");
    }
#if defined(DEBUG_LOCKS)
    printf("      LOCKED fast-mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
  }

  bool tryLock() { return _mutex.try_lock(); }

  bool tryLock(long milliseconds)
  {
    auto deadline = std::chrono::steady_clock::now()
      + std::chrono::milliseconds(milliseconds);
    do
    {
      if (_mutex.try_lock())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void unlock()
  {
#if defined(DEBUG_LOCKS)
    printf("      UNLOCK fast-mutex %s from thread %s.\n", lockName, CURRENT_THREAD_NAME);
#endif
    _mutex.unlock();
  }

  const char* lockName;

private:
  std::mutex _mutex;
  CFastMutex(const CFastMutex&) = delete;
  CFastMutex& operator=(const CFastMutex&) = delete;
};

/// RAII guard that locks a mutex pointer in constructor, unlocks in destructor.
template <class M>
class CScopedLock
{
public:
  explicit CScopedLock(M* mutex) : _mutex(mutex) { _mutex->lock(); }
  ~CScopedLock() { _mutex->unlock(); }
  CScopedLock(const CScopedLock&) = delete;
  CScopedLock& operator=(const CScopedLock&) = delete;
private:
  M* _mutex;
};

// Convenience macros used throughout ES40 source.
#define MUTEX_LOCK(mutex)         (mutex)->lock(LOCK_TIMEOUT_MS)
#define MUTEX_UNLOCK(mutex)       (mutex)->unlock()
#define MUTEX_READ_LOCK(mutex)    (mutex)->readLock(LOCK_TIMEOUT_MS)
#define MUTEX_WRITE_LOCK(mutex)   (mutex)->writeLock(LOCK_TIMEOUT_MS)
#define SCOPED_M_LOCK(mutex)      CScopedLock<CMutex>     _sml_##__LINE__(mutex)
#define SCOPED_FM_LOCK(mutex)     CScopedLock<CFastMutex> _sfml_##__LINE__(mutex)

#endif // ES40_MUTEX_H
