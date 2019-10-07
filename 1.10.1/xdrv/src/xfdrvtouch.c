/*
 * Copyright 2004-2006 by HIT.Chang <jonathanchan77@yahoo.com.tw>
 *      somehow based on xf86Summa.c:
 * Copyright 1996 by Steven Lang <tiger@tyger.org>
 *
 * This work is sponsored by Idealtek-TWN.
 *
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that
 * copyright notice and this permission notice appear in supporting
 * documentation, and that the name of the authors not be used in advertising
 * or publicity pertaining to distribution of the software without specific,
 * written prior permission.  The authors make no representations about the
 * suitability of this software for any purpose.  It is provided "as is"
 * without express or implied warranty.
 *
 * THE AUTHORS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL STEVEN LANG BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTIONS, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/* $XFree86: <This string is added by official archiving> */

// static const char identification[] = "$Identification: 18 $";

// #define XINPUT	// ajd not needed, defined in the xorg headers
#include <xorg-server.h>
#include <bits/types/sigset_t.h>
#include <xf86Xinput.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h> 

#ifdef BUS_PCI
#undef BUS_PCI
#endif

#ifdef BUS_ISA
#undef BUS_ISA
#endif


#define XFREE86_V4

#ifndef XFree86LOADER
#include <unistd.h>
#endif

#include <misc.h>
#include <xf86.h>
#define NEED_XF86_TYPES
#if !defined(DGUX)
#include <xisb.h>
#endif

#include <xf86_OSproc.h>
#include <xf86Xinput.h>
#include <exevents.h>           /* Needed for InitValuator/Proximity stuff */
#include <X11/keysym.h>
#include <mipointer.h>

#ifdef XFree86LOADER
#include <xf86Module.h>
#endif

#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
#include <xserver-properties.h>
#include <X11/extensions/XKB.h>
#include <xkbsrv.h>
#endif

/*#undef memset
#define memset xf86memset*/
#undef sleep
#define sleep(t) xf86WaitForInput(-1, 1000 * (t))
#define wait_for_fd(fd, timeout) xf86WaitForInput((fd), timeout)
#define tcflush(fd, n) xf86FlushInput((fd))
#undef read
#define read(a,b,c) xf86ReadSerial((a),(b),(c))
#undef write
#define write(a,b,c) xf86WriteSerial((a),(char*)(b),(c))
#undef close
#define close(a) xf86CloseSerial((a))
#define XCONFIG_PROBED "(==)"
#define XCONFIG_GIVEN "(**)"
#define xf86Verbose 1
#undef PRIVATE
#define PRIVATE(x) XI_PRIVATE(x)

/*
 * Be sure to set vmin appropriately for your device's protocol. You want to
 * read a full packet before returning
 */
static const char *default_options[] =
{
    "BaudRate", "9600",
    "StopBits", "1",
    "DataBits", "8",
    "Parity", "None",
    "Vmin", "1",
    "Vtime", "10",
    "FlowControl", "None",
    NULL
};

static InputDriverPtr idtDrv;

#if defined(__QNX__) || defined(__QNXNTO__)
#define POSIX_TTY
#endif

#include "Ctrl_XInput.h" /* Definitions for the led meanings */
#include "idealtek.h"

/*
** Debugging macros
*/
#ifdef DBG
#undef DBG
#endif
#ifdef DEBUG
#undef DEBUG
#endif
static int ReadInput = 0; /* xf86IdealtekReadInput flag */
static int      debug_level = 0;
#define DEBUG   1
#if DEBUG
#define         DBG(lvl, f)         {if ((lvl) <= debug_level) f;}
#else
#define         DBG(lvl, f)
#endif


#define WORD_ASSEMBLY01(byte1, byte2)	(((byte2) << 7) | (byte1))
#define WORD_ASSEMBLY00(byte1, byte2)	((byte2) | ((byte1)<<7))

/*
** Device records
*/
#define IDEALTEK_MAXPHYSCOORD             16383
#define IDEALTEK_MAXCOORD                 16383    /* oversampled, synthetic value */
#define FLAG_PRE_TAPPING                  1        /* this is motion, despite being pen-down */
#define FLAG_WAS_UP                       2        /* last event was a pen-up event */
#define FLAG_RES12BIT                     0x100
#define IdT_BUFFER_SIZE		          256      /* Size of input buffer.*/
#define CLOCK_TICK_RATE                   1193180
#define KIOCSOUND	0x4B2F	/* start sound generation (0 for off) */
#define KDMKTONE	0x4B30	/* generate tone */
#define TPK_MAX_BUTTONS		3	/* maximum number of tablet buttons */

/*
 * Prototype for a callback to handle delayed release event.
 * This is handy for integrating out the on/off jitter that
 * the Idealtek device tends to produce during a user drag (Chris Howe)
 */


typedef struct
{
    char           *idtDevice;           /* device file name */
    int            flags;                /* various flags */
    int            idtType;              /* TYPE_SERIAL, etc */
    int            idtBaud;              /* 9600 or 19200 */

    unsigned char  rawData[12];          /* be parsed data buffer*/
    unsigned char  RespData[128];        /* Command Response */
    unsigned char  In_Index;             /* Data In Index */
    unsigned char  Out_Index;            /* Data Out Index */
    int            idtCalib[3][2];          /* Hmm.... */
    int            idtBPnt[2];
    char           *idtConfig;           /* filename for configuration */
    LedCtrl        *idtLeds;
    unsigned char  leftdown;
    unsigned char  rightdown;
    unsigned char  curByte;
    unsigned char  CheckSum;
    unsigned char  RspState;
    unsigned char  MBytes;
    unsigned char  protocol;
    int            screen_no;
    unsigned char  swap_y;
    int		   rotation;
    int		   RightButtonON;
    unsigned char  Upsound;
    unsigned char  Dnsound;
    int TwinXResolution;
    int TwinYResolution;
    int TwinXOrigin;
    int TwinYOrigin;

} IdealtekDeviceRec, *IdealtekDevicePtr;

#define MICROTOUCH_MODE           3
#define ELO_MODE                  2
#define IDEALTEK_MODE             1

/*
** Configuration data
*/
#define IDEALTEK_SECTION_NAME    "IdealtekTS"
#define IDEALTEK_DEFAULT_CFGFILE "/etc/touch.calib"

enum devicetypeitems
{
    TYPE_UNKNOWN = 0,
    TYPE_SERIAL = 1,
    TYPE_USB
};


int Beep(int freq, int dur);

#ifdef XFREE86_V3
enum cfgitems
{
    PORT = 1,
    DEVICENAME,
    DEVICETYPE,
    TAPPING,
    SPEED,
    CALIBRATION_FILE,
    SMOOTHNESS,
    HISTORY_SIZE,
    DEBUG_LEVEL,
    ALWAYS_CORE,
    RES12BIT
};

static SymTabRec CfgTab[] =
{
    {ENDSUBSECTION,    "endsubsection"},
    {DEVICENAME,       "devicename"},       /* user-defined */
    {PORT,             "port"},             /* /dev/.... */
    {DEVICETYPE,       "devicetype"},       /* "serial", "USB" */
    {SPEED,            "speed"},            /* 9600 (default) or 19200 */
    {CALIBRATION_FILE, "calibrationfile"},  /* default "/etc/touch.calib" */
    {DEBUG_LEVEL,      "debuglevel"},       /* default is 0 */
    {ALWAYS_CORE,      "alwayscore"},       /* usually needed */
    { -1,               ""}
};

static SymTabRec CfgDeviceType[] =
{
    {TYPE_SERIAL,  "serial"},
    {TYPE_USB,     "usb"},
    { -1,      ""}
};


/* From: X-4.0.3::xc/programs/Xserver/os/libcwrapper.c */
void
xf86getsecs(long *secs, long *usecs)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    *secs = tv.tv_sec;
    *usecs = tv.tv_usec;

    return;
}

#endif /* XFREE86_V4 */

/*
** Contants and macro
*/
#define BUFFER_SIZE     64          /* size of reception buffer */
#define XI_NAME         "IDEALTOUCH" /* X device name for the touch screen */
#define MSGID           "xf86Idtouoch: "

#define SYSCALL(call) while( ( (call) == -1) && (errno == EINTR) )

#ifdef XFREE86_V3
/*
 * xf86IdealtekConfig
 * Reads the Idealtek touch screen section from the XF86Config file
 */
static Bool
xf86IdealtekConfig(InputInfoPtr *array, int inx, int max, LexPtr val)
{
    InputInfoPtr      dev = array[inx];
    IdealtekDevicePtr      priv = (IdealtekDevicePtr)(dev->private);
    int                 token;
    int                 mtoken;

    DBG(1, ErrorF("xf86IdealtekConfig\n"));

    while ((token = xf86GetToken(CfgTab)) != ENDSUBSECTION)
    {
        switch (token)
        {
            case DEVICENAME:
                if (xf86GetToken(NULL) != STRING)
                {
                    xf86ConfigError(MSGID "Option string expected");
                }
                else
                {
                    dev->name = strdup(val->str);
                    if (xf86Verbose)
                        ErrorF("%s Idealtek touch screen X device name is %s\n",
                               XCONFIG_GIVEN,
                               dev->name);
                }
                break;

            case PORT:
                if (xf86GetToken(NULL) != STRING)
                {
                    xf86ConfigError("Option string expected");
                }
                else
                {
                    priv->idtDevice = strdup(val->str);
                    if (xf86Verbose)
                        ErrorF("%s Idtouch port is %s\n", XCONFIG_GIVEN,
                               priv->idtDevice);
                }
                break;

            case DEVICETYPE:
                mtoken = xf86GetToken(CfgDeviceType);
                if ((mtoken == EOF) || (mtoken == STRING) || (mtoken == NUMBER))
                {
                    xf86ConfigError(MSGID "Mode type token expected");
                }
                else
                {
                    switch (mtoken)
                    {
                        case TYPE_SERIAL:
                        case TYPE_USB:
                            priv->idtType = mtoken;
                            if (xf86Verbose)
                                ErrorF("%s Device type is %s\n", XCONFIG_GIVEN,
                                       mtoken == TYPE_SERIAL ? "serial" : "USB";
                            break;
                        default:
                            xf86ConfigError(MSGID "Illegal device type");
                            break;
                    }
                }
                break;

            case SPEED:
                if (xf86GetToken(NULL) != NUMBER)
                {
                    xf86ConfigError(MSGID "Baud rate expected");
                    break;
                }
                if (val->num != 9600 && val->num != 19200)
                {
                    xf86ConfigError(MSGID "Baud rate must be 9600 or 19200");
                    break;
                }
                priv->idtBaud = val->num;
                if (xf86Verbose)
                    ErrorF("%s Idealtek baud rate %i\n", XCONFIG_GIVEN,
                           priv->idtBaud);
                break;

            case CALIBRATION_FILE:
                if (xf86GetToken(NULL) != STRING)
                {
                    xf86ConfigError(MSGID "File name expected as a string");
                    break;
                }
                SYSCALL(mtoken = access(val->str, R_OK));
                if (mtoken < 0)
                {
                    xf86ConfigError(MSGID "Can't read Idealtek configuration file");
                    break;
                }
                priv->idtConfig = strdup(val->str);
                if (xf86Verbose)
                    ErrorF("%s Idealtek touch screen calibration file is %s\n",
                           XCONFIG_GIVEN,
                           priv->idtConfig);
                break;

            case DEBUG_LEVEL:
                if (xf86GetToken(NULL) != NUMBER)
                {
                    xf86ConfigError("Option number expected");
                }
                debug_level = val->num;
                if (xf86Verbose)
                {
                    ErrorF("%s Idealtek touch screen debug level set to %d\n",
                           XCONFIG_GIVEN, debug_level);
                }
                break;

            case ALWAYS_CORE:
                xf86AlwaysCore(dev, TRUE);
                if (xf86Verbose)
                    ErrorF("%s Idealtek touch screen device is always core pointer\n",
                           XCONFIG_GIVEN);
                break;

            case EOF:
                FatalError("Unexpected EOF (missing EndSubSection)");
                break;

            default:
                xf86ConfigError("Idealtek touch screen subsection keyword expected");
                break;
        }
    }

    DBG(1, ErrorF("xf86IdealtekConfig name=%s\n", priv->idtDevice));

    return Success;
}
#endif /* XFREE86_V3 */

#if 0

/*
 ***************************************************************************
 ** xf86IdealtekConvert--
 ** Convert valuators to X and Y. Since calibration data has already been used,
 ** this only requires downscaling to screen size
 ***************************************************************************
 */
static Bool
xf86IdealtekConvert(InputInfoPtr	local,
                    int		first,
                    int		num,
                    int		v0,
                    int		v1,
                    int		v2,
                    int		v3,
                    int		v4,
                    int		v5,
                    int		*x,
                    int		*y)
{
    IdealtekDevicePtr   priv = (IdealtekDevicePtr) local->private;

    if (first != 0 || num != 2)
    {
        return FALSE;
    }

    *x = (screenInfo.screens[priv->screen_no]->width * v0) / IDEALTEK_MAXCOORD;
    *y = (screenInfo.screens[priv->screen_no]->height * v1) / IDEALTEK_MAXCOORD;

    if (*x < 0)
    {
        *x = 0;
    }
    else if (*x > screenInfo.screens[priv->screen_no]->width)
    {
        *x = screenInfo.screens[priv->screen_no]->width;
    }

    if (*y < 0)
    {
        *y = 0;
    }
    else if (*y > screenInfo.screens[priv->screen_no]->height)
    {
        *y = screenInfo.screens[priv->screen_no]->height;
    }

    if (priv->swap_y)
    {
        *y = screenInfo.screens[priv->screen_no]->height - (*y);
    }


#ifdef XFREE86_V4
    /*
     * Need to check if still on the correct screen.
     * This call is here so that this work can be done after
     * calib and before posting the event.
     */
    xf86XInputSetScreen(local, priv->screen_no, *x, *y);
#endif

    return TRUE;
}

#endif 

/*======================================================================
Function Name :
    Do3PCalibration
Description:
    Do 3 points Calibration for scaling and offset.
Author:
     idealTEK, 2004-2-12
Comment:
     None
======================================================================*/

void Do3PCalibration(InputInfoPtr  local,
                     int X,
                     int Y,
                     int *Return)
{
    long  q, xp1, yp1, x11, y11, x21, y21;
    long  Lx, Ly;
    long  dY, dX;
    IdealtekDevicePtr      priv = (IdealtekDevicePtr) local->private;

    Lx = (4096 - priv->idtBPnt[0]) - priv->idtBPnt[0];
    Lx = Lx >> 2; /* 10 Bits */
    Ly = (4096 - priv->idtBPnt[1]) - priv->idtBPnt[1];
    Ly = Ly >> 2; /*10 Bits */

    xp1 = X - priv->idtCalib[0][0]; /*BASE POINT*/
    xp1 = xp1 >> 2; /*10 bits*/
    yp1 = Y - priv->idtCalib[0][1];
    yp1 = yp1 >> 2; /*10 bits*/

    x11 = priv->idtCalib[1][0] - priv->idtCalib[0][0];
    x11 = x11 >> 2;
    y11 = priv->idtCalib[1][1] - priv->idtCalib[0][1];
    y11 = y11 >> 2;
    x21 = priv->idtCalib[2][0] - priv->idtCalib[0][0];
    x21 = x21 >> 2;
    y21 = priv->idtCalib[2][1] - priv->idtCalib[0][1];
    y21 = y21 >> 2;

    q = x11 * y21 - x21 * y11; /*20 bits*/

    dX = (y21 * xp1) - (x21 * yp1); /*30 bits*/
    dX = (dX * Lx) / q; /*10 bits*/
    Return[0] = dX << 2; /*12 bits*/

    dY = (x11 * yp1) - (y11 * xp1); /*30 bits*/
    dY = (dY * Ly) / q; /*10 bits*/

    Return[1] = dY << 2; /*12 bits*/

    Return[0] += priv->idtBPnt[0];
    Return[1] += priv->idtBPnt[1];
}


/*======================================================================
Function Name :
    MReportCoord
Description:
    Report the coordinate using MicroTouch format
Author:
     idealTEK, 2003-2-28
Comment:
     None
======================================================================*/

void MReportCoord(InputInfoPtr  local)
{
    int                    cur_x, cur_y;
    int                    CXY[2];

    IdealtekDevicePtr      priv = (IdealtekDevicePtr) local->private;

    if (priv->rawData[0] & 0x10)
    {
        cur_x = WORD_ASSEMBLY01(priv->rawData[1], priv->rawData[2]);
        cur_y = WORD_ASSEMBLY01(priv->rawData[3], priv->rawData[4]);
    }
    else
    {
        cur_x = WORD_ASSEMBLY00(priv->rawData[1], priv->rawData[2]);
        cur_y = WORD_ASSEMBLY00(priv->rawData[3], priv->rawData[4]);
    }
    cur_x = cur_x >> 2; /*12 bits*/
    cur_y = cur_y >> 2; /*12 bits*/

    if (cur_x < 0)
    {
        cur_x = 0;
    }
    else if (cur_x > 4095)
    {
        cur_x = 4095;
    }

    if (cur_y < 0)
    {
        cur_y = 0;
    }
    else if (cur_y > 4095)
    {
        cur_y = 4095;
    }
    
    if (priv->TwinXResolution > 0 && priv->TwinYResolution > 0)
    {
        cur_x = (cur_x * priv->TwinXResolution / screenInfo.screens[priv->screen_no]->width) + (4095 * priv->TwinXOrigin / screenInfo.screens[priv->screen_no]->width);
        cur_y = (cur_y * priv->TwinYResolution / screenInfo.screens[priv->screen_no]->height) + (4095 * priv->TwinYOrigin / screenInfo.screens[priv->screen_no]->height);
    }
    Do3PCalibration(local, cur_x, cur_y, CXY);

    if (cur_x < 0)
    {
        cur_x = 0;
    }
    else if (cur_x > 4095)
    {
        cur_x = 4095;
    }

    if (cur_y < 0)
    {
        cur_y = 0;
    }
    else if (cur_y > 4095)
    {
        cur_y = 4095;
    }

    switch (priv->rotation)
    {
        case 90:
            {
                long TmpVal;
                TmpVal = CXY[0];
                CXY[0] = 4095 - CXY[1];
                CXY[1] = TmpVal;
            }
            break;
        case 180:
            {
                CXY[0] = 4095 - CXY[0];
                CXY[1] = 4095 - CXY[1];
            }
            break;
        case 270:
            {
                long TmpVal;
                TmpVal = CXY[0];
                CXY[0] = CXY[1];
                CXY[1] = 4095 - TmpVal;
            }
            break;
    }
    cur_x = CXY[0] << 2; /*14 bits*/
    cur_y = CXY[1] << 2; /*14 bits*/
    xf86PostMotionEvent(local->dev, TRUE, 0, 2, cur_x, cur_y);

    /*  Emit a button press or release. */
    if (priv->rawData[0] & 0x01)  /*Right Mouse Down*/
    {
        if (priv->rightdown == 0) /*generate left up*/
        {
            //xf86PostButtonEvent(local->dev, TRUE, 1, 0,
            //	    0, 2, cur_x, cur_y);
            priv->rightdown = 1;
        }
        else if (priv->rightdown == 1) /*Generate right down*/
        {
            if (priv->RightButtonON == 1)
            {
                xf86PostButtonEvent(local->dev, TRUE, 3, 1, 0, 2, cur_x, cur_y);
            }
            priv->rightdown = 2;
        }
    }
    else
    {
        if (priv->rawData[0] & 0x40)  /*Left Mouse Down*/
        {
            if (priv->leftdown == 0)
            {
                xf86PostButtonEvent(local->dev, TRUE, 1, 1, 0, 2, cur_x, cur_y);
                if (priv->Dnsound == 1)
                {
                    Beep(2500, 10);
                }
                priv->leftdown = 1;
            }
        }
        else  /* mouse button Up */
        {
            if (priv->rightdown == 2)
            {
                if (priv->RightButtonON == 1)
                {
                    xf86PostButtonEvent(local->dev, TRUE, 3, 0, 0, 2, cur_x, cur_y);
                }
                else
                {
                    xf86PostButtonEvent(local->dev, TRUE, 1, 0, 0, 2, cur_x, cur_y);
                }
                if (priv->Upsound == 1)
                {
                    Beep(1500, 10);
                }
            }
            else
            {
                xf86PostButtonEvent(local->dev, TRUE, 1, 0, 0, 2, cur_x, cur_y);
                if (priv->Upsound == 1)
                {
                    Beep(1500, 10);
                }
            }
            priv->leftdown = 0;
            priv->rightdown = 0;
        }
    }
}

void  Queue_A_Char(InputInfoPtr  local, unsigned char  theChar)
{
    IdealtekDevicePtr      priv = (IdealtekDevicePtr) local->private;

    priv->RespData[priv->In_Index] = theChar;
    priv->In_Index++;
    if (priv->In_Index >= 128)
    {
        priv->In_Index = 0;
    }
}

int Get_Buffer_From_Queue(InputInfoPtr  local, unsigned char *buffer, int Len)
{
    int Total_Len, Out_Len, i;
    IdealtekDevicePtr      priv = (IdealtekDevicePtr) local->private;

    if (priv->In_Index == priv->Out_Index)
    {
        return 0;
    }

    if (priv->In_Index > priv->Out_Index)
    {
        Total_Len = priv->In_Index - priv->Out_Index;
    }
    else
    {
        Total_Len = 128 - priv->Out_Index + priv->In_Index;
    }

    if (Total_Len > Len)
    {
        Out_Len = Len;
    }
    else
    {
        Out_Len = Total_Len;
    }

    for (i = 0; i < Out_Len ; i++)
    {
        buffer[i] = priv->RespData[priv->Out_Index];
        priv->Out_Index++;
        if (priv->Out_Index >= 128)
        {
            priv->Out_Index = 0;
        }
    }

    return Out_Len;
}

void Flush_Queue_Buffer(InputInfoPtr  local)
{
    IdealtekDevicePtr      priv = (IdealtekDevicePtr) local->private;
    priv->In_Index = priv->Out_Index = 0;
    memset(priv->RespData, 0, 128);
}

/*==========================================================================
Function Name :
    MDecodeData
Description:
     Decode income data using Microtouch format and decide to report them.
Author:
     idealTEK, 2003-2-28
Comment:
     None
============================================================================*/

void MDecodeData(InputInfoPtr  local, unsigned char *pReadCommBuffer, int ReadBytes)
{
    int  i;
    IdealtekDevicePtr      priv = (IdealtekDevicePtr) local->private;

    for (i = 0; i < ReadBytes; i++)
    {
        priv->rawData[priv->curByte] = pReadCommBuffer[ i ];

        switch (priv->curByte)
        {
            case 0:  /* First byte has the sync bit set, next two don't.*/
                if ((priv->rawData[0] & 0x80) != 0)    /*Enter MOUSE DATA Packet*/
                {
                    priv->curByte++;
                }
                else
                {
                    if (priv->rawData[0] != 0x00)
                    {
                        Queue_A_Char(local, priv->rawData[0]);
                    }
                }
                break;

            case 1:
                if ((priv->rawData[1] & 0x80) == 0)
                {
                    priv->curByte++;
                }
                else
                {
                    priv->rawData[0] = priv->rawData[1];
                    priv->curByte = 1;
                }
                break;

            case 2:
                if ((priv->rawData[2] & 0x80) == 0)
                {
                    priv->curByte++;
                }
                else
                {
                    priv->rawData[0] = priv->rawData[2];
                    priv->curByte = 1;
                }
                break;

            case 3:
                if ((priv->rawData[3] & 0x80) == 0)
                {
                    priv->curByte++;
                }
                else
                {
                    priv->rawData[0] = priv->rawData[3];
                    priv->curByte = 1;
                }
                break;

            case 4:
                if ((priv->rawData[4] & 0x80) == 0)
                {
                    MReportCoord(local);
                    priv->curByte = 0;
                }
                else
                {
                    priv->rawData[0] = priv->rawData[4];
                    priv->curByte = 1;
                }
                break;
        }
    }
}


static void
xf86IdealtekReadInput(InputInfoPtr	local)
{
    unsigned char                 InBuff[IdT_BUFFER_SIZE];
    int                           num_bytes;

    if (ReadInput != 1)
    {

        SYSCALL(num_bytes = read(local->fd,
                                 InBuff,
                                 IdT_BUFFER_SIZE));

        MDecodeData(local, InBuff, num_bytes);
    }
}


/* these functions are missing from Xfree 3.3 */
#ifdef XFREE86_V3
static int xf86BlockSIGIO(void)
{
    return 0;
}
static void xf86UnblockSIGIO(int val) {}
#endif


/*
** xf86IdealtekControlProc
** This is called for each device control that is defined at init time.
** It currently isn't use, but I plan to make on/off available as
** integer device controls.
*/
static void
xf86IdealtekControlProc(DeviceIntPtr       device, PtrCtrl *ctrl)
{
    DBG(2, ErrorF("xf86IdealtekControlProc\n"));
}

/* Read the configuration file or revert to default (identity) cfg */
static int xf86IdealtekReadCalib(IdealtekDevicePtr priv, int activate)
{
    int err = 1;
    FILE *f;

    f = fopen(priv->idtConfig, "r");
    if (f)
    {
        if (fscanf(f, "%d %d", &priv->idtBPnt[0], &priv->idtBPnt[1]) == 2)
        {
            err = 0;
        }
        else
        {
            err = 2;
        }

        xf86Msg(X_CONFIG, "***BASE POINT Data: %d %d\n",
                priv->idtBPnt[0],
                priv->idtBPnt[1]);

        if (fscanf(f, "%d %d %d %d %d %d", &priv->idtCalib[0][0], &priv->idtCalib[0][1],
                   &priv->idtCalib[1][0], &priv->idtCalib[1][1],
                   &priv->idtCalib[2][0], &priv->idtCalib[2][1]) == 6)
        {
            err = err + 0;
        }
        else
        {
            err = err + 4;
        }
        fclose(f);
    }

    if (err)
    {
        ErrorF(MSGID "Calibration data absent or invalid, using defaults\n");
    }

    if (err)
    {
        /* LED_UNCALIBRATE passes through here, to avoid duplication */
        priv->idtBPnt[0] = 3968;
        priv->idtBPnt[1] = 128;
        priv->idtCalib[0][0] = 3968;
        priv->idtCalib[0][1] = 128;
        priv->idtCalib[1][0] = 128;
        priv->idtCalib[1][1] = 128;
        priv->idtCalib[2][0] = 3968;
        priv->idtCalib[2][1] = 3968;
    }
    xf86Msg(X_CONFIG, "***Calibration Data: %d %d %d %d %d %d\n",
            priv->idtCalib[0][0],
            priv->idtCalib[0][1],
            priv->idtCalib[1][0],
            priv->idtCalib[1][1],
            priv->idtCalib[2][0],
            priv->idtCalib[2][1]);
    return 0;
}


/*
 * This feedback function is used to get commands from a client application
 */
static void
xf86IdealtekLeds(DeviceIntPtr dev, LedCtrl *ctrl)
{
    InputInfoPtr      local = (InputInfoPtr)dev->public.devicePrivate;
    IdealtekDevicePtr      priv = (IdealtekDevicePtr)local->private;
    int cmd;

    if (!priv->idtLeds)
    {
        /* fist time */
        priv->idtLeds = ctrl;
        ctrl->led_mask = ~0;
    }

    DBG(9, ErrorF(MSGID "ledfeedback  %x %x\n",
                  (unsigned int)ctrl->led_values, (unsigned int)ctrl->led_mask));
    cmd = ctrl->led_values & ctrl->led_mask;

    if (cmd & IDEALTEKLED_UNCALIBRATE)
    {
        /* remove calibration, used before making a new calibration */
        xf86IdealtekReadCalib(priv, 1);
        if (ReadInput != 0)
        {
            ReadInput = 0;
        }
    }
    if (cmd & IDEALTEKLED_RECALIBRATE)
    {
        /* xf86IdealtekProc( (DeviceIntPtr)local, DEVICE_ON); */
        if (local->fd < 0)
        {
            return;
        }
        tcflush(local->fd, TCIFLUSH); /* flush pending input */
        /* double-enable is fine, so we can do it always */
        AddEnabledDevice(local->fd);
        xf86IdealtekReadCalib(priv, 1);
        if (ReadInput != 0)
        {
            ReadInput = 0;
        }
        /* ptr->public.on = TRUE; */
        /*xf86IdealtekReadCalib(priv, 1);*/
    }
    if (cmd & IDEALTEKLED_OFF)
    {
        if (local->fd >= 0)
            /*   RemoveEnabledDevice(local->fd); */
        {
            ReadInput = 1;
        }
    }

    if (cmd & IDEALTEKLED_BUTTON1)
    {
        /*priv->idtButton = 0;*/
    }
    if (cmd & IDEALTEKLED_BUTTON2)
    {
        /*priv->idtButton = 1;*/
    }
    if (cmd & IDEALTEKLED_BUTTON3)
    {
        /*priv->idtButton = 2;*/
    }
    if (cmd & IDEALTEKLED_BUTTON2ONCE)
    {
        /*priv->idtButton = -1;*/
    }
    if (cmd & IDEALTEKLED_BUTTON3ONCE)
    {
        /*priv->idtButton = -2;*/
    }
}


/*
 ***************************************************************************
 * xf86IdealtekOpen
 * Open and initialize the panel, as well as probe for any needed data.
 ***************************************************************************
 */
#ifdef XFREE86_V4
#define WAIT(t)                                                 \
    err = xf86WaitForInput(-1, ((t) * 1000));                   \
    if (err == -1) {                                            \
        ErrorF("Idealtek select error : %s\n", strerror(errno));   \
        return !Success;                                        \
    }
#else
#define WAIT(t)                                                 \
    timeout.tv_sec = 0;                                         \
    timeout.tv_usec = (t) * 1000;                               \
    SYSCALL(err = select(0, NULL, NULL, NULL, &timeout));       \
    if (err == -1) {                                            \
        ErrorF("Idealtek select error : %s\n", strerror(errno));   \
        return !Success;                                        \
    }
#endif




int Beep(int freq, int dur)
{
    int  console_fd = -1;
    int  theFreq;
    int  result;
    int  err;

    theFreq = CLOCK_TICK_RATE / freq;

    SYSCALL(console_fd = open("/dev/console", O_WRONLY, 0));
    if (console_fd == -1)
    {
        return -1;
    }
    SYSCALL(result = ioctl(console_fd, KIOCSOUND, (char *)theFreq));
    WAIT(dur);
    theFreq = 0;
    SYSCALL(result = ioctl(console_fd, KIOCSOUND, (char *)theFreq));
    SYSCALL(close(console_fd));
    return -1;
}

#if 0
/*******************************************************************************************
**   Send some protocol to device and get response
********************************************************************************************/
static Bool
xf86IdTSendPacket(unsigned char	*packet,
                  int		len,
                  int		fd)
{
    int	result;

    SYSCALL(result = write(fd, packet, len));
    if (result != len)
    {
        xf86Msg(X_CONFIG, "System error while sending to IdealTouch touchscreen.\n");
        return !Success;
    }
    else
    {
        return Success;
    }
}
#endif

#if 0
static int
xf86IdTWaitReply(unsigned char	*reply,
                 int           Len,
                 int		fd)
{
    int             num_bytes, rLen, i;
    unsigned char   RspBuf[128];
    int             err;

    WAIT(300); /*wait for 300 ms*/
    /*xf86WaitForInput(fd, 200*1000);*/
    SYSCALL(num_bytes = read(fd, (char *)RspBuf, 128));
    rLen = 0;

    for (i = 0; i < num_bytes; i++)
    {
        if (RspBuf[i] != 0x00)
        {
            reply[rLen++] = RspBuf[i];
        }
        if (rLen >= Len)
        {
            break;
        }
    }

    return rLen;
}
#endif

static Bool
xf86IdealtekOpen(InputInfoPtr local)
{
#ifdef XFREE86_V3
    struct termios      termios_tty;
    int                 err;
#endif
	 
    IdealtekDevicePtr   priv = (IdealtekDevicePtr)local->private;

    DBG(1, ErrorF("opening %s (calibration file is \"%s\"\n", priv->idtDevice, priv->idtConfig));
    xf86IdealtekReadCalib(priv, 1);
    /* Is it a serial port or something else? */
    if (priv->idtType == TYPE_UNKNOWN)
    {
        if (strstr(priv->idtDevice, "tty"))
        {
            priv->idtType = TYPE_SERIAL;
        }
        else if (strstr(priv->idtDevice, "idtk"))
        {
            priv->idtType = TYPE_USB;
        }
        else
        {
            return !Success;
        }
    }

#ifdef XFREE86_V4
    if (priv->idtType == TYPE_SERIAL)
    {
        local->fd = xf86OpenSerial(local->options);
    }
    else
#endif
        SYSCALL(local->fd = open(priv->idtDevice, O_RDWR | O_NDELAY));

    if (local->fd == -1)
    {
        Error(priv->idtDevice);
        return !Success;
    }
    DBG(2, ErrorF("%s opened as fd %d\n", priv->idtDevice, local->fd));


#ifdef XFREE86_V3 /* 3.3 needs termios handling */
    if (priv->idtType != TYPE_SERIAL)
    {
        return Success;
    }

    /* If serial set the serial options */
    SYSCALL(err = tcgetattr(local->fd, &termios_tty));
    if (err == -1)
    {
        Error("Idealtek touch screen tcgetattr");
        return !Success;
    }
    termios_tty.c_iflag = IXOFF;
    termios_tty.c_lflag = 0;
    termios_tty.c_cflag = CS8 | CREAD | CLOCAL;
    if (priv->idtBaud == 19200)
    {
        termios_tty.c_cflag |= B19200;
    }
    else
    {
        termios_tty.c_cflag |= B9600;
    }

    /* I wonder what these all do, anyway */
    termios_tty.c_cc[VINTR] = 0;
    termios_tty.c_cc[VQUIT] = 0;
    termios_tty.c_cc[VERASE] = 0;
#ifdef VWERASE
    termios_tty.c_cc[VWERASE] = 0;
#endif
#ifdef VREPRINT
    termios_tty.c_cc[VREPRINT] = 0;
#endif
    termios_tty.c_cc[VKILL] = 0;
    termios_tty.c_cc[VEOF] = 0;
    termios_tty.c_cc[VEOL] = 0;
#ifdef VEOL2
    termios_tty.c_cc[VEOL2] = 0;
#endif
    termios_tty.c_cc[VSUSP] = 0;
#ifdef VDISCARD
    termios_tty.c_cc[VDISCARD] = 0;
#endif
#ifdef VLNEXT
    termios_tty.c_cc[VLNEXT] = 0;
#endif

    termios_tty.c_cc[VMIN] = 1 ;
    termios_tty.c_cc[VTIME] = 10 ;

    err = tcsetattr(local->fd, TCSANOW, &termios_tty);
    if (err == -1)
    {
        Error("Idealtek touch screen tcsetattr TCSANOW");
        return !Success;
    }

#endif /* XFREE86_V3 */


    DBG(1, ErrorF("initializing Idealtek touch screen\n"));

    /* Hmm... close it, so it doens't say anything before we're ready */
    /* FIXME */

    /* Clear any pending input */
    tcflush(local->fd, TCIFLUSH);
    if (xf86Verbose)
    {
        ErrorF("%s Idealtek touch screen\n", XCONFIG_PROBED);
    }

    /* FIXME: is there something to write-and-read here? */

    return Success;
}

/*
** xf86IdealtekOpenDevice
** Opens and initializes the device driver stuff
*/
static int
xf86IdealtekOpenDevice(DeviceIntPtr ptr)

{
    InputInfoPtr      local = (InputInfoPtr)ptr->public.devicePrivate;
    
    if (xf86IdealtekOpen(local) != Success)
    {
        if (local->fd >= 0)
        {
            SYSCALL(close(local->fd));
        }
        local->fd = -1;
        
        return !Success;
    }    

    /* Initialize the axes */
    InitValuatorAxisStruct(ptr, 0, /* X */
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
                           XIGetKnownProperty(AXIS_LABEL_PROP_ABS_X),
#endif
                           0, IDEALTEK_MAXCOORD, /* min, max val */
                           500000, /* resolution */
                           0, 500000, Absolute); /* min, max_res */
    InitValuatorAxisStruct(ptr, 1, /* Y */
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
                           XIGetKnownProperty(AXIS_LABEL_PROP_ABS_Y),
#endif
                           0, IDEALTEK_MAXCOORD, /* min, max val */
                           500000, /* resolution */
                           0, 500000, Absolute); /* min, max_res */

    /* Register led feedback */
    if (InitLedFeedbackClassDeviceStruct(ptr, xf86IdealtekLeds) == FALSE)
    {
        DBG(0, ErrorF("Idealtek: can't initialize the feedback structure\n"));
    }

    return (local->fd != -1);
}

/* ################################################# */

/*
** xf86IdealtekClose
** It...  Uh...  Closes the physical device?
*/
static void
xf86IdealtekClose(InputInfoPtr local)
{
    if (local->fd >= 0)
    {
        SYSCALL(close(local->fd));
    }
    local->fd = -1;
}

/*
** xf86IdealtekProc
** Handle requests to do stuff to the driver.
** In order to allow for calibration, the function is called (with DEVICE_OFF
** and DEVICE_ON commands) also by xf86IdealtekChangeControl.
*/
static int
xf86IdealtekProc(DeviceIntPtr ptr, int what)
{
    CARD8               map[25]; /* 25? */
    int                 nbaxes;
    int                 nbbuttons;
    int                 loop;
    
    InputInfoPtr      local = (InputInfoPtr)ptr->public.devicePrivate;
    IdealtekDevicePtr   priv = local->private;
    
    unsigned char       RespBuf[12], CmdBuf[8];
    int                 Times;
    Atom btn_labels[TPK_MAX_BUTTONS] = {0};
    Atom axis_labels[MAX_VALUATORS] = {0};

    int                 input_count = 0;
    InBuffer   USBCmdBuf;
    OutBuffer  USBRespBuf;
    int        Len;

    DBG(2, ErrorF("BEGIN xf86IdealtekProc dev=0x%x priv=0x%x what=%d\n", (unsigned int)ptr, (unsigned int)priv, (unsigned int)what));

    switch (what)
    {
        case DEVICE_INIT:
            DBG(1, ErrorF("xf86IdealtekProc ptr=0x%x what=INIT\n", (unsigned int)ptr));

            nbaxes = 2;                        /* X, Y */
            nbbuttons = TPK_MAX_BUTTONS; /* three, one way or the other... */

            for (loop = 1; loop <= nbbuttons; loop++)
            {
                map[loop] = loop;
            }

            if (InitButtonClassDeviceStruct(ptr,
                                            nbbuttons,
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
                                            btn_labels,
#endif
                                            map) == FALSE)
            {
                ErrorF("unable to allocate Button class device\n");
                return !Success;
            }

            if (InitFocusClassDeviceStruct(ptr) == FALSE)
            {
                ErrorF("unable to init Focus class device\n");
                return !Success;
            }

            /*
             * Use default (?) feedback stuff.
             * I'll use device feedback controls to change parameters at
             * run time.
             */
            if (InitPtrFeedbackClassDeviceStruct(ptr,
                                                 xf86IdealtekControlProc) == FALSE)
            {
                ErrorF("unable to init ptr feedback\n");
                return !Success;
            }

            if (InitProximityClassDeviceStruct(ptr) == FALSE)
            {
                ErrorF("unable to init proximity class device\n");
                return !Success;
            }

            if (InitValuatorClassDeviceStruct(ptr,
                                              nbaxes,
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
                                              axis_labels,
#endif
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) < 3
                                              xf86GetMotionEvents,
#endif
                                              GetMotionHistorySize(),
                                              Absolute)
                == FALSE)
            {
                ErrorF("unable to allocate Valuator class device\n");
                return !Success;
            }
            /* allocate the motion history buffer if needed */
            xf86MotionHistoryAllocate(local);

#ifdef XFREE86_V3
            AssignTypeAndName(ptr, local->atom, local->name);
#endif
            /* open the device to gather informations */
            xf86IdealtekOpenDevice(ptr);
            break;

        case DEVICE_ON:
            DBG(1, ErrorF("xf86IdealtekProc ptr=0x%x what=ON\n", (unsigned int)ptr));

            if ((local->fd < 0))
            {
                return !Success;
            }
            /*xf86FlushInput(local->fd);*/

            /*fd_set EnableDevices;
            fd_set AllSockets;*/

            /*FD_SET(local->fd,&EnableDevices);
            FD_SET(local->fd,&AllSockets);*/


            if (priv->idtType == TYPE_SERIAL)
            {
                xf86Msg(X_CONFIG, "IdealTouch interface type is Serial\n");
            }
            else  if (priv->idtType == TYPE_USB)
            {
                xf86Msg(X_CONFIG, "IdealTouch interface type is USB\n");
            }
            else
            {
                return !Success;
            }

            if (priv->idtType == TYPE_SERIAL)
            {
                memset(CmdBuf, 0, sizeof(CmdBuf));
                CmdBuf[0] = 0x01;
                CmdBuf[1] = 'M';
                CmdBuf[2] = 'S';
                CmdBuf[3] = 0x0D;
            }
            else  if (priv->idtType == TYPE_USB)
            {
                memset(USBCmdBuf.Buf, 0, sizeof(USBCmdBuf.Buf));
                USBCmdBuf.Buf[0] = 0x01;
                USBCmdBuf.Buf[1] = 'M';
                USBCmdBuf.Buf[2] = 'S';
                USBCmdBuf.Buf[3] = 0x0D;
                USBCmdBuf.Len = 4;
            }
            Times = 0;

        SEND_MS_AGAIN:
            xf86Msg(X_CONFIG, "Send MS\n");
            Times++;
            tcflush(local->fd, TCIFLUSH);  /* Clear any pending input */
            if (priv->idtType == TYPE_SERIAL)
            {
                write(local->fd, CmdBuf, 4);
            }
            else  if (priv->idtType == TYPE_USB)
            {
                ioctl(local->fd, IOCTL_IDEALTOUCH_WRITEBUFFER, &USBCmdBuf);
            }

            input_count = wait_for_fd(local->fd, 2000000);

            if (priv->idtType == TYPE_SERIAL)
            {
                if (input_count <= 0)
                {
                    return !Success;
                }
            }
            else
            {
                if (input_count < 0)
                {
                    return !Success;
                }
            }
            if (priv->idtType == TYPE_SERIAL)
            {
                memset(RespBuf, 0, sizeof(RespBuf));
                read(local->fd, RespBuf, sizeof(RespBuf));
                if (RespBuf[0] == 0x01 && RespBuf[1] == 0x30 && RespBuf[2] == 0x0d)
                {
                    Times = 5;
                }
            }
            else  if (priv->idtType == TYPE_USB)
            {
                memset(USBRespBuf.Buf, 0, sizeof(USBRespBuf.Buf));
                ioctl(local->fd, IOCTL_IDEALTOUCH_READLENGTH, &Len);
                USBRespBuf.Len = Len;
                ioctl(local->fd, IOCTL_IDEALTOUCH_READBUFFER, &USBRespBuf);
                if (USBRespBuf.Buf[0] == 0x01 && USBRespBuf.Buf[1] == 0x30 && USBRespBuf.Buf[2] == 0x0d)
                {
                    Times = 5;
                }
            }

            if (Times <= 3)
            {
                goto SEND_MS_AGAIN;
            }
            else if (Times == 4)
            {
                return !Success;
            }
            if (priv->idtType == TYPE_SERIAL)
            {
                memset(CmdBuf, 0, sizeof(CmdBuf));
                CmdBuf[0] = 0x01;
                CmdBuf[1] = 'U';
                CmdBuf[2] = 'u';
                CmdBuf[3] = 0x0D;
            }
            else  if (priv->idtType == TYPE_USB)
            {
                memset(USBCmdBuf.Buf, 0, sizeof(USBCmdBuf.Buf));
                USBCmdBuf.Buf[0] = 0x01;
                USBCmdBuf.Buf[1] = 'U';
                USBCmdBuf.Buf[2] = 'u';
                USBCmdBuf.Buf[3] = 0x0D;
                USBCmdBuf.Len = 4;
            }
            Times = 0;

        SEND_Uu_AGAIN:
            xf86Msg(X_CONFIG, "Send Uu\n");
            Times++;
            tcflush(local->fd, TCIFLUSH);  /* Clear any pending input */
            if (priv->idtType == TYPE_SERIAL)
            {
                write(local->fd, CmdBuf, 4);
            }
            else  if (priv->idtType == TYPE_USB)
            {
                ioctl(local->fd, IOCTL_IDEALTOUCH_WRITEBUFFER, &USBCmdBuf);
            }

            input_count = wait_for_fd(local->fd, 2000000);
            if (priv->idtType == TYPE_SERIAL)
            {
                if (input_count <= 0)
                {
                    return !Success;
                }
            }
            else
            {
                if (input_count < 0)
                {
                    return !Success;
                }
            }
            if (priv->idtType == TYPE_SERIAL)
            {
                memset(RespBuf, 0, sizeof(RespBuf));
                read(local->fd, RespBuf, sizeof(RespBuf));
                if (RespBuf[0] == 0x49 && RespBuf[1] == 0x64 && RespBuf[2] == 0x65 && RespBuf[3] == 0x61 && RespBuf[4] == 0x6C)
                {
                    Times = 5;
                }
                xf86Msg(X_CONFIG, "Uu Response = 0x%02X(%c) 0x%02X(%c) 0x%02X(%c) 0x%02X(%c) 0x%02X(%c)\n", RespBuf[0], RespBuf[0], RespBuf[1], RespBuf[1], RespBuf[2], RespBuf[2], RespBuf[3], RespBuf[3], RespBuf[4], RespBuf[4]);
            }
            else  if (priv->idtType == TYPE_USB)
            {
                memset(USBRespBuf.Buf, 0, sizeof(USBRespBuf.Buf));
                ioctl(local->fd, IOCTL_IDEALTOUCH_READLENGTH, &Len);
                USBRespBuf.Len = Len;
                ioctl(local->fd, IOCTL_IDEALTOUCH_READBUFFER, &USBRespBuf);
                if (USBRespBuf.Buf[0] == 0x49 && USBRespBuf.Buf[1] == 0x64 && USBRespBuf.Buf[2] == 0x65 && USBRespBuf.Buf[3] == 0x61 && USBRespBuf.Buf[4] == 0x6C)
                {
                    Times = 5;
                }
                xf86Msg(X_CONFIG, "Uu Response = 0x%02X(%c) 0x%02X(%c) 0x%02X(%c) 0x%02X(%c) 0x%02X(%c)\n", USBRespBuf.Buf[0], USBRespBuf.Buf[0], USBRespBuf.Buf[1], USBRespBuf.Buf[1], USBRespBuf.Buf[2], USBRespBuf.Buf[2], USBRespBuf.Buf[3], USBRespBuf.Buf[3], USBRespBuf.Buf[4], USBRespBuf.Buf[4]);
            }

            if (Times <= 3)
            {
                goto SEND_Uu_AGAIN;
            }
            else if (Times == 4)
            {
                return !Success;
            }
            xf86Msg(X_CONFIG, "TPK Touch device turn on OK!\n");
            tcflush(local->fd, TCIFLUSH);  /* flush pending input */
            AddEnabledDevice(local->fd);
            ptr->public.on = TRUE;

            break;

        case DEVICE_OFF:
            DBG(1, ErrorF("xf86IdealtekProc  ptr=0x%x what=OFF\n", (unsigned int)ptr));
            if (local->fd >= 0)
            {
                xf86RemoveEnabledDevice(local);
                xf86IdealtekClose(local);
            }
            ptr->public.on = FALSE;
            break;

        case DEVICE_CLOSE:
            if (local->fd >= 0)
            {            
                xf86IdealtekClose(local);
            }
            DBG(1, ErrorF("xf86IdealtekProc  ptr=0x%x what=CLOSE\n", (unsigned int)ptr));
            ptr->public.on = FALSE;
            break;

        default:
            ErrorF("unsupported mode=%d\n", what);
            return !Success;
            break;
    }
    DBG(2, ErrorF("END   xf86IdealtekProc Success what=%d dev=0x%x priv=0x%x\n",
                  (unsigned int)what, (unsigned int)ptr, (unsigned int)priv));
    return Success;
}




/*
** xf86IdealtekSwitchMode
** Switches the mode. Only absolute is allowed.
*/
static int
xf86IdealtekSwitchMode(ClientPtr client, DeviceIntPtr dev, int mode)
{
    /*DBG(3, ErrorF("xf86IdealtekSwitchMode dev=0x%x mode=%d\n", dev, mode));*/

    switch (mode)
    {
        case Absolute:
            break;

        default:
            /*DBG(1, ErrorF("xf86IdealtekSwitchMode dev=0x%x invalid mode=%d\n",
                   dev, mode));*/
            return BadMatch;
    }
    return Success;
}


/*
** xf86IdealtekAllocate
** Allocates the device structures for the Idealtek Touch Screen.
*/
static int
xf86IdealtekAllocate(InputInfoPtr pInfo)
{

    IdealtekDevicePtr priv = (IdealtekDevicePtr)malloc(sizeof(IdealtekDeviceRec));
    if (!priv)
    {
        DBG(3, ErrorF("xf86IdealtekAllocate failed to allocate priv\n"));
        goto LocalDevicePtr_fail;
    }
    
    pInfo->name = XI_NAME;
    pInfo->type_name = "Idealtek Touch Screen";
    pInfo->flags = 0; /*XI86_NO_OPEN_ON_INIT;*/
#ifdef XFREE86_V3
    pInfo->device_config = xf86IdealtekConfig;
#endif
    pInfo->device_control = xf86IdealtekProc;
    pInfo->read_input = xf86IdealtekReadInput;
    pInfo->switch_mode = xf86IdealtekSwitchMode;
    /* reverse_conversion is only used by relative devices (for warp events) */
    pInfo->fd = -1;
    pInfo->private = priv;
    priv->idtDevice = "";         /* device file name */
    priv->idtConfig = IDEALTEK_DEFAULT_CFGFILE;
    priv->idtType = TYPE_UNKNOWN;
    priv->idtBaud = 9600;
    priv->flags = FLAG_WAS_UP;    /* first event is button-down */
    priv->leftdown = 0;
    priv->rightdown = 0;
    priv->curByte = 0;
    priv->RspState = 0;
    priv->MBytes = 0;
    priv->screen_no = 0;
    priv->protocol = 0; /*IDEALTEK00*/
    priv->swap_y = 0;
    priv->rotation = 0;
    priv->RightButtonON = 1;
    memset(priv->RespData, 0, 128);        /* Command Response */
    priv->In_Index = 0;             /* Data In Index */
    priv->Out_Index = 0;            /* Data Out Index */
    priv->TwinXResolution = 0;
    priv->TwinYResolution = 0;
    priv->TwinXOrigin = 0;
    priv->TwinYOrigin = 0;
    
    return Success;
    
LocalDevicePtr_fail:

    if (priv) free(priv);
    
    return BadValue;
}


/*
** IDEALTEKTS device association
** Device section name and allocation function.
*/

#ifdef XFREE86_V3
#ifdef DYNAMIC_MODULE
/*
 ***************************************************************************
 * init_module
 * Entry point for dynamic module.
 ***************************************************************************
 */
int
#ifndef DLSYM_BUG
init_module(unsigned long server_version)
#else
init_xf86Idealtek(unsigned long server_version)
#endif
{
    xf86AddDeviceAssoc(&idealtek_assoc);

    if (server_version != XF86_VERSION_CURRENT)
    {
        xf86Msg(X_CONFIG, "Warning: IdealtekTS module compiled for version%s %ld\n", XF86_VERSION_CURRENT, server_version);
        /*ErrorF("Warning: IdealtekTS module compiled for version%s\n",
               XF86_VERSION);*/
        return 0;
    }
    else
    {
        return 1;
    }
}
#endif /* DYNAMIC_MODULE */

#else /* XFREE86_V4 */

/*
 * xf86IdtUninit --
 *
 * called when the driver is unloaded.
 */
static void
xf86IdtUninit(InputDriverPtr    drv,
              InputInfoPtr    local,
              int flags)
{
    IdealtekDevicePtr      priv = (IdealtekDevicePtr) local->private;

    DBG(1, ErrorF("xf86IdtUninit\n"));
    xf86IdealtekProc(local->dev, DEVICE_OFF);
    free(priv);
    local->private = NULL;    
}

/*
 * xf86IdtInit --
 *
 * called when the module subsection is found in XF86Config
 */
static int
xf86IdtInit(InputDriverPtr      drv,
            InputInfoPtr		pInfo,
            int                 flags)
{
    IdealtekDevicePtr priv = NULL;
    char		*s;

    xf86Msg(X_CONFIG, "Linux Touch Driver. v1.0.0.8. 2011/05/31\n");
    
    idtDrv = drv; /* used by xf86IdealtekAllocate() */    
    
    if (xf86IdealtekAllocate(pInfo) != Success)
    {
        xf86Msg(X_CONFIG, "%s: Can't allocate touchscreen data\n",
                pInfo->name);
        goto SetupProc_fail;
    }
    priv = (IdealtekDevicePtr)(pInfo->private);    
    /* Device name is mandatory */
    priv->idtDevice = xf86FindOptionValue(pInfo->options, "Device");
    if (!priv->idtDevice)
    {
        xf86Msg(X_ERROR, "%s: Device not specified\n",
                pInfo->name);
        goto SetupProc_fail;
    }
    priv->idtType = TYPE_UNKNOWN;
    xf86ProcessCommonOptions(pInfo, pInfo->options);
    priv->screen_no = xf86SetIntOption(pInfo->options, "ScreenNo", 0);
    xf86Msg(X_CONFIG, "IdealTouch associated screen: %d\n", priv->screen_no);

    priv->swap_y = xf86SetIntOption(pInfo->options, "SwapY", 0);
    if (priv->swap_y)
    {
        xf86Msg(X_CONFIG, "IdealTouch %s device will work with Y axes swapped\n",
                pInfo->name);
    }

    priv->rotation = xf86SetIntOption(pInfo->options, "Rotation", 0);
    xf86Msg(X_CONFIG, "Rotation : %d\n", priv->rotation);
    priv->RightButtonON = xf86SetIntOption(pInfo->options, "RightButtonON ", 1);
    xf86Msg(X_CONFIG, "RightButtonON : %d\n", priv->RightButtonON);
    debug_level = xf86SetIntOption(pInfo->options, "DebugLevel", 0);

    priv->Upsound = xf86SetIntOption(pInfo->options, "UpSound", 0);
    xf86Msg(X_CONFIG, "Upsound : %d\n", priv->Upsound);

    priv->Dnsound = xf86SetIntOption(pInfo->options, "DownSound", 0);
    xf86Msg(X_CONFIG, "DownSound : %d\n", priv->Dnsound);

    s = xf86FindOptionValue(pInfo->options, "XRandrSetting");
    if (s)
    {
        int a, b, c, d;
        if ((sscanf(s, "%dx%d+%d+%d", &a, &b, &c, &d) != 4) ||
            (a <= 0) || (b <= 0) || (c < 0) || (d < 0))
        {
            xf86Msg(X_CONFIG, "XRandrSetting is wrong!!\n");
            priv->TwinXResolution = 0;
            priv->TwinYResolution = 0;
            priv->TwinXOrigin = 0;
            priv->TwinYOrigin = 0;
        }
        else
        {
            priv->TwinXResolution = a;
            priv->TwinYResolution = b;
            priv->TwinXOrigin = c;
            priv->TwinYOrigin = d;
            xf86Msg(X_CONFIG, "XRandrSetting: Resolution= %dx%d OriginX=%d,OriginY=%d\n",
                    priv->TwinXResolution, priv->TwinYResolution, priv->TwinXOrigin, priv->TwinYOrigin);
        }
    }

	xf86Msg(X_CONFIG, "%s: xf86IdtInit() finished.\n", pInfo->name);
  
    return Success;

SetupProc_fail:

    xf86Msg(X_CONFIG, "idealtek::SetupProc_fail\n");
    
    if (priv) free(priv);
    
    return BadValue;
}

#ifdef XFree86LOADER
static
#endif
InputDriverRec IDEALTEK =
{
    1,                          /* driver version */
    "xfdrvtouch",                    /* driver name */
    NULL,                       /* identify */
    xf86IdtInit,                /* pre-init */
    xf86IdtUninit,              /* un-init */
    NULL,                       /* module */
    default_options 
};

/*
 ***************************************************************************
 *
 * Dynamic loading functions
 *
 ***************************************************************************
 */
#ifdef XFree86LOADER

/*
 * xf86IdtUnplug --
 *
 * called when the module subsection is found in XF86Config
 */
static void
xf86IdtUnplug(pointer   p)
{
    DBG(1, ErrorF("xf86IdtUnplug\n"));
}

/*
 * xf86IdtPlug --
 *
 * called when the module subsection is found in XF86Config
 */
static pointer
xf86IdtPlug(pointer     module,
            pointer     options,
            int         *errmaj,
            int         *errmin)
{
    DBG(1, ErrorF("xf86IdtPlug\n"));

    xf86AddInputDriver(&IDEALTEK, module, 0);

    return module;
}

static XF86ModuleVersionInfo xf86IdtVersionRec =
{
    "xfdrvtouch",
    "idealTEK",
    MODINFOSTRING1,
    MODINFOSTRING2,
    XORG_VERSION_CURRENT,
    1, 2, 0,  /* VERSION */
    ABI_CLASS_XINPUT,
    ABI_XINPUT_VERSION,
    MOD_CLASS_XINPUT,
    {0, 0, 0, 0}                /* signature, to be patched into the file by */
    /* a tool */
};


_X_EXPORT XF86ModuleData xfdrvtouchModuleData =
{
    &xf86IdtVersionRec,
    xf86IdtPlug,
    xf86IdtUnplug
};

#endif /* XFree86LOADER */
#endif /* XFREE86_V4 */

/* end of xfdrvtouch.c */
