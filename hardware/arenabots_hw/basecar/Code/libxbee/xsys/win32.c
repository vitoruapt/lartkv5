/*
  libxbee - a C library to aid the use of Digi's Series 1 XBee modules
            running in API mode (AP=2).

  Copyright (C) 2009  Attie Grande (attie@attie.co.uk)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* ################################################################# */
/* ### Win32 Code ################################################## */
/* ################################################################# */

/*  this file contains code that is used by Win32 ONLY */
#ifndef _WIN32
#error "This file should only be used on a Win32 system"
#endif

#include "win32.h"
#include "win32.dll.c"

static int init_serial(xbee_hnd xbee, int baudrate) {
  int chosenbaud;
  DCB tc;
  int evtMask;
  COMMTIMEOUTS timeouts;

  /* open the serial port */
  xbee->tty = CreateFile(TEXT(xbee->path),
                        GENERIC_READ | GENERIC_WRITE,
                        0,    /* exclusive access */
                        NULL, /* default security attributes */
                        OPEN_EXISTING,
                        FILE_FLAG_OVERLAPPED,
                        NULL);
  if (xbee->tty == INVALID_HANDLE_VALUE) {
    xbee_log("Invalid file handle...");
    xbee_log("Is the XBee plugged in and avaliable on the correct port?");
    xbee_mutex_destroy(xbee->conmutex);
    xbee_mutex_destroy(xbee->pktmutex);
    xbee_mutex_destroy(xbee->sendmutex);
    Xfree(xbee->path);
    return -1;
  }

  GetCommState(xbee->tty, &tc);
  tc.BaudRate          = baudrate;
  tc.fBinary           = TRUE;
  tc.fParity           = FALSE;
  tc.fOutxCtsFlow      = FALSE;
  tc.fOutxDsrFlow      = FALSE;
  tc.fDtrControl       = DTR_CONTROL_DISABLE;
  tc.fDsrSensitivity   = FALSE;
  tc.fTXContinueOnXoff = FALSE;
  tc.fOutX             = FALSE;
  tc.fInX              = FALSE;
  tc.fErrorChar        = FALSE;
  tc.fNull             = FALSE;
  tc.fRtsControl       = RTS_CONTROL_DISABLE;
  tc.fAbortOnError     = FALSE;
  tc.ByteSize          = 8;
  tc.Parity            = NOPARITY;
  tc.StopBits          = ONESTOPBIT;
  SetCommState(xbee->tty, &tc);

  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 0;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  SetCommTimeouts(xbee->tty, &timeouts);

  SetCommMask(xbee->tty, EV_RXCHAR);

  return 0;
}

/* a replacement for the linux select() function... for a serial port */
static int xbee_select(xbee_hnd xbee, struct timeval *timeout) {
  int evtMask = 0;
  COMSTAT status;
  int ret;

  for (;;) {
    /* find out how many bytes are in the Rx buffer... */
    if (ClearCommError(xbee->tty,NULL,&status) && (status.cbInQue > 0)) {
      /* if there is data... return! */
      return 1; /*status.cbInQue;*/
    } else if (timeout && timeout->tv_sec == 0 && timeout->tv_usec == 0) {
      /* if the timeout was 0 (return immediately) then return! */
      return 0;
    }

    /* otherwise wait for an Rx event... */
    memset(&(xbee->ttyovrs),0,sizeof(OVERLAPPED));
    xbee->ttyovrs.hEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
    if (!WaitCommEvent(xbee->tty,&evtMask,&(xbee->ttyovrs))) {
      if (GetLastError() == ERROR_IO_PENDING) {
        DWORD timeoutval;
        if (!timeout) {
          /* behave like the linux function... if the timeout pointer was NULL
             then wait indefinately */
          timeoutval = INFINITE;
        } else {
          /* Win32 doesn't give the luxury of microseconds and seconds... just miliseconds! */
          timeoutval = (timeout->tv_sec * 1000) + (timeout->tv_usec / 1000);
        }
        ret = WaitForSingleObject(xbee->ttyovrs.hEvent,timeoutval);
        if (ret == WAIT_TIMEOUT) {
          /* cause the WaitCommEvent() call to stop */
          SetCommMask(xbee->tty, EV_RXCHAR);
          /* if a timeout occured, then return 0 */
          CloseHandle(xbee->ttyovrs.hEvent);
          return 0;
        }
      } else {
        return -1;
      }
    }
    CloseHandle(xbee->ttyovrs.hEvent);
  }

  /* always return -1 (error) for now... */
  return -1;
}

/* this offers the same behavior as non-blocking I/O under linux */
int xbee_write(xbee_hnd xbee, const void *ptr, size_t size) {
  if (!WriteFile(xbee->tty, ptr, size, NULL, &(xbee->ttyovrw)) &&
      (GetLastError() != ERROR_IO_PENDING)) return 0;
  if (!GetOverlappedResult(xbee->tty, &(xbee->ttyovrw), &(xbee->ttyw), TRUE)) return 0;
  return xbee->ttyw;
}

/* this offers the same behavior as non-blocking I/O under linux */
int xbee_read(xbee_hnd xbee, void *ptr, size_t size) {
  if (!ReadFile(xbee->tty, ptr, size, NULL, &(xbee->ttyovrr)) &&
      (GetLastError() != ERROR_IO_PENDING)) return 0;
  if (!GetOverlappedResult(xbee->tty, &(xbee->ttyovrr), &(xbee->ttyr), TRUE)) return 0;
  return xbee->ttyr;
}

/* this is because Win32 has some weird memory management rules...
   - the thread that allocated the memory, must free it... */
void xbee_free(void *ptr) {
  if (!ptr) return;
  free(ptr);
}

/* win32 equivalent of unix gettimeofday() */
int gettimeofday(struct timeval *tv, struct timezone *tz) {
  if (tv) {
    struct _timeb timeb;
    _ftime(&timeb);
    tv->tv_sec = timeb.time;
    tv->tv_usec = timeb.millitm * 1000;
  }
  /* ignore tz for now */
  return 0;
}

/* ################################################################# */
/* ### Helper Functions (Mainly for VB6 use) ####################### */
/* ################################################################# */

/* enable the debug output to a custom file or fallback to stderr */
int xbee_setupDebugAPI(char *path, int baudrate, char *logfile, char cmdSeq, int cmdTime) {
  xbee_hnd xbee = default_xbee;
  int fd, ret;
  if ((fd = _open(logfile,_O_WRONLY | _O_CREAT | _O_TRUNC)) == -1) {
    ret = xbee_setuplogAPI(path,baudrate,2,cmdSeq,cmdTime);
  } else {
    ret = xbee_setuplogAPI(path,baudrate,fd,cmdSeq,cmdTime);
  }
  if (fd == -1) {
    xbee_log("Error opening logfile '%s' (errno=%d)... using stderr instead...",logfile,errno);
  }
  return ret;
}
int xbee_setupDebug(char *path, int baudrate, char *logfile) {
  return xbee_setupDebugAPI(path,baudrate,logfile,0,0);
}

/* These silly little functions are required for VB6
   - it freaks out when you call a function that uses va_args... */
xbee_con *xbee_newcon_simple(unsigned char frameID, xbee_types type) {
  return xbee_newcon(frameID, type);
}
xbee_con *xbee_newcon_16bit(unsigned char frameID, xbee_types type, int addr) {
  return xbee_newcon(frameID, type, addr);
}
xbee_con *xbee_newcon_64bit(unsigned char frameID, xbee_types type, int addrL, int addrH) {
  return xbee_newcon(frameID, type, addrL, addrH);
}

void xbee_enableACKwait(xbee_con *con) {
  con->waitforACK = 1;
}
void xbee_disableACKwait(xbee_con *con) {
  con->waitforACK = 0;
}

void xbee_enableDestroySelf(xbee_con *con) {
  con->destroySelf = 1;
}

/* for vb6... it will send a message to the given hWnd which can in turn check for a packet */
void xbee_callback(xbee_con *con, xbee_pkt *pkt) {
  xbee_hnd xbee = default_xbee;
  win32_callback_info *p = callbackMap;
  
  /* grab the mutex */
  xbee_mutex_lock(callbackmutex);
  
  /* see if there is an existing callback for this connection */
  while (p) {
    if (p->con == con) break;
    p = p->next;
  }
  
  /* release the mutex (before the SendMessage, as this could take time...) */
  xbee_mutex_unlock(callbackmutex);
  
  /* if there is, continue! */
  if (p) {
    xbee_log("Callback message sent!");
    SendMessage(p->hWnd, p->uMsg, (int)con, (int)pkt);
  } else {
    xbee_log("Callback message NOT sent... Unmapped callback! (con=0x%08X)",con);
  }
}

/* very simple C function to provide more functionality to VB6 */
int xbee_runCallback(int(*func)(xbee_con*,xbee_pkt*), xbee_con *con, xbee_pkt *pkt) {
  return func(con,pkt);
}

void xbee_attachCallback(xbee_con *con, HWND hWnd, UINT uMsg) {
  xbee_hnd xbee = default_xbee;
  win32_callback_info *l, *p;
  
  /* grab the mutex */
  xbee_mutex_lock(callbackmutex);
  
  l = NULL;
  p = callbackMap;
  
  /* see if there is an existing callback for this connection */
  while (p) {
    if (p->con == con) break;
    l = p;
    p = p->next;
  }
  /* if not, then add it */
  if (!p) {
    p = Xcalloc(sizeof(win32_callback_info));
    p->next = NULL;
    p->con = con;
    if (!l) {
      xbee_log("Mapping the first callback...");
      callbackMap = p;
    } else {
      xbee_log("Mapping another callback...");
      l->next = p;
    }
  } else {
    xbee_log("Updating callback map...");
  }
  /* setup / update the parameters */
  xbee_log("  connection @ 0x%08X",con);
  xbee_log("  hWnd       = 0x%08X",hWnd);
  xbee_log("  uMsg       = 0x%08X",uMsg);
  p->hWnd = hWnd;
  p->uMsg = uMsg;
  
  /* setup the callback function */
  con->callback = xbee_callback;
  
  /* release the mutex */
  xbee_mutex_unlock(callbackmutex);
}

void xbee_detachCallback(xbee_con *con) {
  xbee_hnd xbee = default_xbee;
  win32_callback_info *l = NULL, *p = callbackMap;
  xbee_mutex_lock(callbackmutex);
  
  /* see if there is an existing callback for this connection */
  while (p) {
    if (p->con == con) break;
    l = p;
    p = p->next;
  }
  /* if there is, then remove it! */
  if (p) {
    if (!l) {
      callbackMap = NULL;
    } else if (l->next) {
      l->next = l->next->next;
    } else {
      l->next = NULL;
    }
    xbee_log("Unmapping callback...");
    xbee_log("  connection @ 0x%08X",con);
    xbee_log("  hWnd       = 0x%08X",p->hWnd);
    xbee_log("  uMsg       = 0x%08X",p->uMsg);
    Xfree(p);
  }
  
  con->callback = NULL;
  
  /* release the mutex */
  xbee_mutex_unlock(callbackmutex);
}
