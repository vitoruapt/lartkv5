/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
 * \file
 * \brief Conio windows functions source file
 */

#include <phua_haptic/conio.h>

int _kbhit()
{
    static int initialized;
	
    fd_set rfds;
    struct timeval tv;
    int retcode;
	
    if(!initialized)
    {
        if(tcgetattr(STDIN_FILENO, &term_attribs) < 0)
        {
            perror("tcgetattr: ");
            exit(-1);
        }

        term_attribs_old = term_attribs;

        if(atexit(restore_term))
        {
            perror("atexit: ");
            exit(-1);
        }

        term_attribs.c_lflag &= ~(ECHO | ICANON | ISIG | IEXTEN);
        term_attribs.c_iflag &= ~(IXON | BRKINT | INPCK | ICRNL | ISTRIP);
        term_attribs.c_cflag &= ~(CSIZE | PARENB);
        term_attribs.c_cflag |= CS8;
        term_attribs.c_cc[VTIME] = 0;
        term_attribs.c_cc[VMIN] = 0;

        if(tcsetattr(STDIN_FILENO, TCSANOW, &term_attribs) < 0)
        {
            perror("tcsetattr: ");
            exit(-1);
        }

        initialized = 1;
    }	

    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    memset(&tv, 0, sizeof(tv));
    
    retcode = select(1, &rfds, NULL, NULL, &tv);
    if(retcode == -1 && errno == EINTR)
    {
        return 0;
    }
    else if(retcode < 0)
    {
        perror("select: ");
        exit(-1);
    }
    else if(FD_ISSET(STDIN_FILENO, &rfds))
    {
        return retcode;
    }
	
    return 0;
}

int getch()
{
    fd_set rfds;
    int retcode;

    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    
    retcode = select(1, &rfds, NULL, NULL, NULL);
    if(retcode == -1 && errno == EINTR)
    {
        return 0;
    }
    else if(retcode < 0)
    {
        perror("select: ");
        exit(-1);
    }

    return getchar();
}
