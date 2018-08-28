#include "bhand_misc.h"

#include <stdio.h>
#include <string.h>

#ifdef LINUX
	#include <termios.h>
#endif


bool isAlpha2(const char c)
{
	return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

bool isspacechar(const char c)
{
	// isspace from cctype can have annoying debug assertion failures so another verison is defined here
	return c == ' ' || c == '\r' || c == '\n' || c == '\t';
}

bool searchstrn(char *dest, const char *src, int max)
{
	// Search up to max characters for source string in destination memory
	int matched = 0;
	int srclen = strlen(src);
	while (max > 0)
	{
		if (dest[matched] == src[matched])
		{
			matched++;
			if (matched == srclen)
				return true;
		}
		else
		{
			matched = 0;
			dest++;
			max--;
		}
	}
	return false;
}

int countWords(const char *str)
{
	int nResults = 0;
	bool first = false;
	while (1)
	{
		const char c = *(str++);
		if (c == 0)
			return nResults;

		if (c != ' ')
		{
			if (!first)
			{
				nResults++;
				first = true;
			}
		}
		else
		{
			first = false;
		}
	}
	return nResults;
}

///////////////////////////////////////////////////////////////////////////////
// OS Wrapper Functions
///////////////////////////////////////////////////////////////////////////////

double TimeInSeconds()
{
#ifdef LINUX
	struct timeval tv;
	gettimeofday(&tv, 0);
	return tv.tv_sec + (double)(tv.tv_usec) * 1e-6;
#else
	return (double)GetTickCount() * 1e-3;
#endif
}


#ifdef LINUX

//struct termios _old_tio; // Global saved termios settings for linux

int UnbufferedInputStart()
{
// TODO: put this back in?
	/* The following written by Olivier Mehani <shtrom-kb@ssji.net>
	* from http://shtrom.ssji.net/skb/getc.html */
//	struct termios new_tio;
	/* get the terminal settings for stdin */
//	tcgetattr(STDIN_FILENO, &_old_tio);
	/* we want to keep the old setting to restore them at the end */
//	new_tio = _old_tio;
	/* disable canonical mode (buffered i/o) and local echo */
//	new_tio.c_lflag &= (~ICANON & ~ECHO);
	/* minimum of number input read. */
//	new_tio.c_cc[VMIN] = 1;
	/* set the new settings immediately */
//	tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
	return 0;
}

int UnbufferedInputEnd()
{
// TODO: put this back in?
    /* restore the former settings */
//	tcsetattr(STDIN_FILENO, TCSANOW, &_old_tio);
    return 0;
}

#else

int UnbufferedInputStart() { return 0; }
int UnbufferedInputEnd() { return 0; }

#endif

#ifdef LINUX
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

int _getch()
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}
#endif


int UnbufferedGetChar()
{
   if (_kbhit())
      return _getch();
   else
      return EOF;
}

/*#ifdef LINUX
   struct timeval tv;
   fd_set read_set;
   tv.tv_sec = 0;
   tv.tv_usec = 0;
   FD_ZERO(&read_set);
   FD_SET(STDIN_FILENO, &read_set);
   select(STDIN_FILENO+1, &read_set, NULL, NULL, &tv);
   if (FD_ISSET(STDIN_FILENO, &read_set))
      return getchar();
   else
      return EOF;
#else*/
//#endif
