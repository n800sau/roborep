// gcc -I/usr/include/fastcgi -lfcgi testfastcgi.c -o test.fastcgi
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <alloca.h>
#include <fcgiapp.h>
#include <uriparser/Uri.h>

#include "CMUcam4.h"
#include "CMUcom4.h"

#define LISTENSOCK_FILENO 0
#define LISTENSOCK_FLAGS 0

CMUcam4 cam(CMUCOM4_SERIAL);

int main(int argc, char** argv) 
{
  openlog("testfastcgi", LOG_CONS|LOG_NDELAY, LOG_USER);
//  cam.begin();
  FCGX_Stream *in, *out, *err;
  FCGX_ParamArray envp;

  while (FCGX_Accept(&in, &out, &err, &envp) >= 0) 
  {
    FCGX_FPrintF(out,"Content-type: text/plain\r\n\r\n");      
    
    char *pathInfo = FCGX_GetParam("PATH_INFO",envp);
    
	FCGX_FPrintF(out,"REQUEST_URI=%s\n", FCGX_GetParam("REQUEST_URI",envp));

        const char *query_string = FCGX_GetParam("QUERY_STRING", envp);
	FCGX_FPrintF(out,"QUERY=%s\n", query_string);

	 UriUriA uri;
        UriQueryListA * queryList;
        int itemCount;
        if (uriDissectQueryMallocA(&queryList, &itemCount, uri.query.first, uri.query.afterLast) != URI_SUCCESS) {
			FCGX_FPrintF(out,"QUERY PARSING FAILED\n");
			
        }
        uriFreeQueryListA(queryList);




  }


//	cam.end();
  return 0;
}
