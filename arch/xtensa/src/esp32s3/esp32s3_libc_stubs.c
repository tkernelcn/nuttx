/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_libc_stubs.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/lib/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>

#include "rom/esp32s3_libc_stubs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROM_MUTEX_MAGIC   0xbb10c433

/* ROM newlib reads _global_impure_ptr from this fixed .bss address (see
 * boards/espressif/esp32s3/nuttx-config/scripts/esp32s3_rom.ld). NuttX's
 * lib_impure.o is often dropped by the linker because the ROM ld script
 * provides an absolute _global_impure_ptr symbol instead.
 */

#define ROM_GLOBAL_IMPURE_PTR  ((struct _reent **)(uintptr_t)0x3fceffd0)

/* Toolchain newlib is built with _REENT_SMALL (see sys/reent.h). */

#define ROM_REENT_SIZE  0xf0

/* ESP32-S3 ROM newlib passes magic lock pointers (see esp_rom_newlib_init_common
 * _mutexes). Redirect them to app-owned mutexes, matching ESP-IDF locks.c.
 */

#define MAYBE_OVERRIDE_LOCK(_lock, _lock_to_use_instead) \
  do \
    { \
      if ((_lock) != 0 && *(int *)(_lock) == ROM_MUTEX_MAGIC) \
        { \
          (_lock) = (_lock_t)(_lock_to_use_instead); \
        } \
    } \
  while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct _reent;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ROM newlib reent: NuttX lib_impure.o pulls in toolchain __sf which is not
 * linked when ROM provides printf/strtoul. Keep a BSS blob for ROM instead.
 */

static uint8_t g_rom_reent[ROM_REENT_SIZE];

static struct _reent *rom_reent(void)
{
  return (struct _reent *)(uintptr_t)g_rom_reent;
}

mutex_t g_common_mutex;
rmutex_t g_common_recursive_mutex;

static int g_rom_mutex_magic = ROM_MUTEX_MAGIC;
static _lock_t g_rom_magic_mutex = (_lock_t)&g_rom_mutex_magic;
static bool g_libc_locks_inited;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int _close_r(struct _reent *r, int fd)
{
  UNUSED(r);
  return nx_close(fd);
}

int _fstat_r(struct _reent *r, int fd, struct stat *statbuf)
{
  UNUSED(r);
  return fstat(fd, statbuf);
}

int _getpid_r(struct _reent *r)
{
  UNUSED(r);
  return (int)getpid();
}

int _kill_r(struct _reent *r, int pid, int sig)
{
  UNUSED(r);
  return nxsig_kill(pid, sig);
}

int _link_r(struct _reent *r, const char *oldpath, const char *newpath)
{
  UNUSED(r);
  UNUSED(oldpath);
  UNUSED(newpath);
  return 0;
}

int lseek_r(struct _reent *r, int fd, int offset, int whence)
{
  UNUSED(r);
  return (int)nx_seek(fd, offset, whence);
}

int _open_r(struct _reent *r, const char *pathname, int flags, int mode)
{
  UNUSED(r);
  return nx_open(pathname, flags, mode);
}

int read_r(struct _reent *r, int fd, void *buf, int count)
{
  UNUSED(r);
  return (int)nx_read(fd, buf, count);
}

int _rename_r(struct _reent *r, const char *oldpath, const char *newpath)
{
  UNUSED(r);
  return rename(oldpath, newpath);
}

void *_sbrk_r(struct _reent *r, ptrdiff_t increment)
{
  UNUSED(r);
  UNUSED(increment);
  errno = ENOMEM;
  return (void *)-1;
}

int _stat_r(struct _reent *r, const char *pathname, struct stat *statbuf)
{
  UNUSED(r);
  return nx_stat(pathname, statbuf, 1);
}

clock_t _times_r(struct _reent *r, struct tms *buf)
{
  UNUSED(r);
  return times(buf);
}

int _unlink_r(struct _reent *r, const char *pathname)
{
  UNUSED(r);
  return nx_unlink(pathname);
}

int write_r(struct _reent *r, int fd, const void *buf, int count)
{
  UNUSED(r);
  return (int)nx_write(fd, buf, count);
}

int _gettimeofday_r(struct _reent *r, struct timeval *tv, void *tz)
{
  UNUSED(r);
  return gettimeofday(tv, tz);
}

void *_malloc_r(struct _reent *r, size_t size)
{
  UNUSED(r);
  return lib_malloc(size);
}

void *_realloc_r(struct _reent *r, void *ptr, size_t size)
{
  UNUSED(r);
  return lib_realloc(ptr, size);
}

void *_calloc_r(struct _reent *r, size_t nmemb, size_t size)
{
  UNUSED(r);
  return lib_zalloc(nmemb * size);
}

void _free_r(struct _reent *r, void *ptr)
{
  UNUSED(r);
  lib_free(ptr);
}

void _abort(void)
{
  abort();
}

void _raise_r(struct _reent *r)
{
  UNUSED(r);
}

static void libc_locks_init_once(void)
{
  if (!g_libc_locks_inited)
    {
      nxmutex_init(&g_common_mutex);
      nxrmutex_init(&g_common_recursive_mutex);
      g_libc_locks_inited = true;
    }
}

static void lock_init_generic(_lock_t *lock, bool recursive)
{
  irqstate_t flags = enter_critical_section();

  if (*lock == 0)
    {
      if (recursive)
        {
          rmutex_t *rmutex = (rmutex_t *)kmm_malloc(sizeof(rmutex_t));

          if (rmutex != NULL)
            {
              nxrmutex_init(rmutex);
              *lock = (_lock_t)rmutex;
            }
        }
      else
        {
          mutex_t *mutex = (mutex_t *)kmm_malloc(sizeof(mutex_t));

          if (mutex != NULL)
            {
              nxmutex_init(mutex);
              *lock = (_lock_t)mutex;
            }
        }
    }

  leave_critical_section(flags);
}

void _lock_init(_lock_t *lock)
{
  *lock = 0;
  lock_init_generic(lock, false);
}

void _lock_init_recursive(_lock_t *lock)
{
  *lock = 0;
  lock_init_generic(lock, true);
}

void _lock_close(_lock_t *lock)
{
  irqstate_t flags = enter_critical_section();

  if (*lock != 0 &&
      (_lock_t)&g_common_mutex != *lock &&
      (_lock_t)&g_common_recursive_mutex != *lock)
    {
      nxmutex_destroy((mutex_t *)(*lock));
      kmm_free((FAR void *)(*lock));
      *lock = 0;
    }

  leave_critical_section(flags);
}

void _lock_close_recursive(_lock_t *lock)
{
  irqstate_t flags = enter_critical_section();

  if (*lock != 0 &&
      (_lock_t)&g_common_mutex != *lock &&
      (_lock_t)&g_common_recursive_mutex != *lock)
    {
      nxrmutex_destroy((rmutex_t *)(*lock));
      kmm_free((FAR void *)(*lock));
      *lock = 0;
    }

  leave_critical_section(flags);
}

static void lock_acquire_generic(_lock_t *lock, bool recursive, bool trylock)
{
  if (*lock == 0)
    {
      lock_init_generic(lock, recursive);
    }

  if (*lock == 0)
    {
      return;
    }

  if (recursive)
    {
      if (trylock)
        {
          (void)nxrmutex_trylock((rmutex_t *)(*lock));
        }
      else
        {
          nxrmutex_lock((rmutex_t *)(*lock));
        }
    }
  else
    {
      if (trylock)
        {
          (void)nxmutex_trylock((mutex_t *)(*lock));
        }
      else
        {
          nxmutex_lock((mutex_t *)(*lock));
        }
    }
}

void _lock_acquire(_lock_t *lock)
{
  lock_acquire_generic(lock, false, false);
}

void _lock_acquire_recursive(_lock_t *lock)
{
  lock_acquire_generic(lock, true, false);
}

int _lock_try_acquire(_lock_t *lock)
{
  lock_acquire_generic(lock, false, true);
  return 0;
}

int _lock_try_acquire_recursive(_lock_t *lock)
{
  lock_acquire_generic(lock, true, true);
  return 0;
}

void _lock_release(_lock_t *lock)
{
  if (*lock != 0)
    {
      nxmutex_unlock((mutex_t *)(*lock));
    }
}

void _lock_release_recursive(_lock_t *lock)
{
  if (*lock != 0)
    {
      nxrmutex_unlock((rmutex_t *)(*lock));
    }
}

static inline void check_lock_nonzero(_lock_t lock)
{
  UNUSED(lock);
}

static void esp32s3_sinit(struct _reent *r)
{
  if (r == NULL)
    {
      r = rom_reent();
    }
}

void __retarget_lock_init(_lock_t *lock)
{
  _lock_init(lock);
}

void __retarget_lock_init_recursive(_lock_t *lock)
{
  _lock_init_recursive(lock);
}

void __retarget_lock_close(_lock_t lock)
{
  _lock_close(&lock);
}

void __retarget_lock_close_recursive(_lock_t lock)
{
  _lock_close_recursive(&lock);
}

void __retarget_lock_acquire(_lock_t lock)
{
  libc_locks_init_once();

  if (lock == 0)
    {
      lock = (_lock_t)&g_common_mutex;
    }

  MAYBE_OVERRIDE_LOCK(lock, &g_common_mutex);
  _lock_acquire(&lock);
}

void __retarget_lock_acquire_recursive(_lock_t lock)
{
  libc_locks_init_once();

  if (lock == 0)
    {
      lock = (_lock_t)&g_common_recursive_mutex;
    }

  MAYBE_OVERRIDE_LOCK(lock, &g_common_recursive_mutex);
  _lock_acquire_recursive(&lock);
}

int __retarget_lock_try_acquire(_lock_t lock)
{
  libc_locks_init_once();

  if (lock == 0)
    {
      lock = (_lock_t)&g_common_mutex;
    }

  MAYBE_OVERRIDE_LOCK(lock, &g_common_mutex);
  return _lock_try_acquire(&lock);
}

int __retarget_lock_try_acquire_recursive(_lock_t lock)
{
  libc_locks_init_once();

  if (lock == 0)
    {
      lock = (_lock_t)&g_common_recursive_mutex;
    }

  MAYBE_OVERRIDE_LOCK(lock, &g_common_recursive_mutex);
  return _lock_try_acquire_recursive(&lock);
}

void __retarget_lock_release(_lock_t lock)
{
  if (lock == 0)
    {
      return;
    }

  MAYBE_OVERRIDE_LOCK(lock, &g_common_mutex);
  _lock_release(&lock);
}

void __retarget_lock_release_recursive(_lock_t lock)
{
  if (lock == 0)
    {
      return;
    }

  MAYBE_OVERRIDE_LOCK(lock, &g_common_recursive_mutex);
  _lock_release_recursive(&lock);
}

struct _reent *__getreent(void)
{
  return rom_reent();
}

int _system_r(struct _reent *r, const char *command)
{
  UNUSED(r);
  UNUSED(command);
  return 0;
}

void noreturn_function __assert_func(const char *file, int line,
                                     const char *func, const char *expr)
{
  UNUSED(func);
  UNUSED(expr);
  _assert(file, line);
}

void _cleanup_r(struct _reent *r)
{
  UNUSED(r);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct syscall_stub_table g_stub_table =
{
  .__getreent = &__getreent,
  ._malloc_r = &_malloc_r,
  ._free_r = &_free_r,
  ._realloc_r = &_realloc_r,
  ._calloc_r = &_calloc_r,
  ._abort = &_abort,
  ._system_r = &_system_r,
  ._rename_r = &_rename_r,
  ._times_r = &_times_r,
  ._gettimeofday_r = &_gettimeofday_r,
  ._raise_r = &_raise_r,
  ._unlink_r = &_unlink_r,
  ._link_r = &_link_r,
  ._stat_r = &_stat_r,
  ._fstat_r = &_fstat_r,
  ._sbrk_r = &_sbrk_r,
  ._getpid_r = &_getpid_r,
  ._kill_r = &_kill_r,
  ._exit_r = NULL,
  ._close_r = &_close_r,
  ._open_r = &_open_r,
  ._write_r = &write_r,
  ._lseek_r = &lseek_r,
  ._read_r = &read_r,
  ._retarget_lock_init = &__retarget_lock_init,
  ._retarget_lock_init_recursive = &__retarget_lock_init_recursive,
  ._retarget_lock_close = &__retarget_lock_close,
  ._retarget_lock_close_recursive = &__retarget_lock_close_recursive,
  ._retarget_lock_acquire = &__retarget_lock_acquire,
  ._retarget_lock_acquire_recursive = &__retarget_lock_acquire_recursive,
  ._retarget_lock_try_acquire = &__retarget_lock_try_acquire,
  ._retarget_lock_try_acquire_recursive =
    &__retarget_lock_try_acquire_recursive,
  ._retarget_lock_release = &__retarget_lock_release,
  ._retarget_lock_release_recursive = &__retarget_lock_release_recursive,
  ._printf_float = NULL,
  ._scanf_float = NULL,
  .__assert_func = &__assert_func,
  .__sinit = &esp32s3_sinit,
  ._cleanup_r = &_cleanup_r
};

/****************************************************************************
 * Name: esp_setup_syscall_table
 ****************************************************************************/

void esp_setup_syscall_table(void)
{
  extern void esp_rom_newlib_init_common_mutexes(_lock_t, _lock_t);

  libc_locks_init_once();
  g_rom_mutex_magic = ROM_MUTEX_MAGIC;
  g_rom_magic_mutex = (_lock_t)&g_rom_mutex_magic;

  memset(g_rom_reent, 0, sizeof(g_rom_reent));

  syscall_table_ptr = (struct syscall_stub_table *)&g_stub_table;

  /* Point ROM newlib at NuttX's reent structure. Without this, ROM printf/
   * strtoul use a NULL/garbage reent and corrupt memory over time.
   */

  *ROM_GLOBAL_IMPURE_PTR = rom_reent();

  esp_rom_newlib_init_common_mutexes(g_rom_magic_mutex, g_rom_magic_mutex);
}
