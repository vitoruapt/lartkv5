/** @file
 * @brief My version of conio.h include file that some portions of code provided by SensAble require.
 
 All credits go to SensAble.
 ****************************************************************************

Copyright (c) 2005 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com
                                          
Module Name:

    conio.h

Description: 

    Console functionality required by example code.

*****************************************************************************
 * @author pedro_cruz
 * @version 1.0
 * @date 3 May 2012
 *@{
*/

#ifndef __CONIO_H_
#define __CONIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

static struct termios term_attribs, term_attribs_old;

static void restore_term(void)
{
    if(tcsetattr(STDIN_FILENO, TCSAFLUSH, &term_attribs_old) < 0)
    {
        perror("tcsetattr: ");
        exit(-1);
    }
}

// #ifdef _cplusplus
// extern "C" {
// #endif // _cplusplus

/**
* @brief waits for user input on keyboard

This functions holds program execution until user presses any key.
* @return 0 if sucessful | errorcode if not
*/
int _kbhit();

/**
* @brief gets character from keyboard input

This functions holds program execution until user presses any key.
* @return input char if sucessful | errorcode if not
*/
int getch();

// #ifdef _cplusplus
// }
// #endif // _cplusplus

#endif // __CONIO_H

/******************************************************************************/
/**
 *@}
*/