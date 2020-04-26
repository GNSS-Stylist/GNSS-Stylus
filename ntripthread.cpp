/*
    Original implementation: NTRIP client for POSIX.
    $Id: ntripclient.c,v 1.51 2009/09/11 09:49:19 stoecker Exp $
    Copyright (C) 2003-2008 by Dirk Stöcker <soft@dstoecker.de>

    ntripthread.cpp (modifications for GNSS-Stylus)
    Copyright (C) 2019 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
 * This NTRIP-client-implementation is based on the "NTRIP client for POSIX" by Dirk Stöcker.
 * Modifications were mostly encapsulating it to a c++ class, removing of the serial interface
 * and replacing it with QT's signal-based mechanism and overriding the output functions
 * (printf, fprintf, fwrite). Also some statics were moved into the class to make the
 * thread restartable.
 *
 * Not too pretty but works...
 *
 * Note: Does not work with u-center (version 19.06) if authentication is in use.
 * Authentication works with SNIP.
*/

#ifdef _WIN32
#define WINDOWSVERSION
#endif

#include "ntripthread.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>
#include <memory>

#ifdef WINDOWSVERSION
  #include <winsock.h>
  typedef SOCKET sockettype;
  typedef u_long in_addr_t;
  typedef int socklen_t;
#else
  typedef int sockettype;
  #include <signal.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <arpa/inet.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netdb.h>

  #define closesocket(sock)       close(sock)
  #define ALARMTIME   (2*60)
#endif

//#ifndef COMPILEDATE
//#define COMPILEDATE " built " __DATE__
//#endif

/* The string, which is send as agent in HTTP request */
#define AGENTSTRING "NTRIP GNSS-Stylus"
#define TIME_RESOLUTION 125

#define MAXDATASIZE 1000 /* max number of bytes we can get at once */

#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wdate-time"                    // "expansion of date or time macro is not reproducible" "note: expanded from macro 'COMPILEDATE'"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wsign-conversion"              // "implicit conversion changes signedness: 'int' to 'char'"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"         // "this statement may fall through"
//#pragma GCC diagnostic ignored "-Wconditional-uninitialized"    // "variable 'p' may be uninitialized when used here"
#pragma GCC diagnostic ignored "-Wconversion"                   // "implicit conversion loses integer precision: 'long' to 'u_short' (aka 'unsigned short')"
#pragma GCC diagnostic ignored "-Wsign-compare"                 // "comparison of integers of different signs: 'sockettype' (aka 'unsigned int') and 'int'"
#pragma GCC diagnostic ignored "-Wcast-align"                   // "cast from 'char *' to 'struct in_addr *' increases required alignment from 1 to 4"
//#pragma GCC diagnostic ignored "-Wpointer-sign"                 // "passing 'socklen_t *' (aka 'unsigned int *') to parameter of type 'int *' converts between pointers to integer types with different sign" "winsock.h:283:82: note: passing argument to parameter 'namelen' here"
#pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"

/* Version (/revision) */
/* TODO: Get rid of hard-coded version */
static const char revisionstr[] = "1.0.0";
//static const char datestr[]     = "2019/08/21";

/* option parsing */
#ifdef NO_LONG_OPTS
#define LONG_OPT(a)
#else
#define LONG_OPT(a) a

static const struct option optsInit[] = {
{ "bitrate",    no_argument,       nullptr, 'b'},
//{ "data",       required_argument, nullptr, 'd'}, /* compatibility */
{ "mountpoint", required_argument, nullptr, 'm'},
{ "initudp",    no_argument,       nullptr, 'I'},
{ "udpport",    required_argument, nullptr, 'P'},
{ "server",     required_argument, nullptr, 's'},
{ "password",   required_argument, nullptr, 'p'},
{ "port",       required_argument, nullptr, 'r'},
{ "proxyport",  required_argument, nullptr, 'R'},
{ "proxyhost",  required_argument, nullptr, 'S'},
{ "user",       required_argument, nullptr, 'u'},
{ "nmea",       required_argument, nullptr, 'n'},
{ "mode",       required_argument, nullptr, 'M'},
{ "serlogfile", required_argument, nullptr, 'l'},
{ "help",       no_argument,       nullptr, 'h'},
{nullptr,0,nullptr,0}};

#endif
#define ARGOPT "m:bhp:r:s:u:n:S:R:M:IP:l:"
//Original: #define ARGOPT "-d:m:bhp:r:s:u:n:S:R:M:IP:D:B:T:C:Y:A:l:"

const char *NTRIPThread::encodeurl(const char *req)
{
  const char *h = "0123456789abcdef";
  char *urlenc = encodeUrlBuffer;
  char *bufend = encodeUrlBuffer + sizeof(encodeUrlBuffer) - 3;

  while(*req && urlenc < bufend)
  {
    if(isalnum(*req)
    || *req == '-' || *req == '_' || *req == '.')
      *urlenc++ = *req++;
    else
    {
      *urlenc++ = '%';
      *urlenc++ = h[*req >> 4];
      *urlenc++ = h[*req & 0x0f];
      req++;
    }
  }
  *urlenc = 0;
  return encodeUrlBuffer;
}

const char *NTRIPThread::geturl(const char *url, struct Args *args)
{
  const char *h = "0123456789abcdef";

  getUrlBufferPtr = getUrlBuffer;
  getUrlBufferEndPtr = getUrlBuffer+sizeof(getUrlBuffer);

  if(strncmp("ntrip:", url, 6))
    return "URL must start with 'ntrip:'.";
  url += 6; /* skip ntrip: */

  if(*url != '@' && *url != '/')
  {
    /* scan for mountpoint */
    args->data = getUrlBufferPtr;
    if(*url != '?')
    {
       while(*url && *url != '@' &&  *url != ';' && *url != '/' && getUrlBufferPtr != getUrlBufferEndPtr)
         *(getUrlBufferPtr++) = *(url++);
    }
    else
    {
       while(*url && *url != '@' &&  *url != '/' && getUrlBufferPtr != getUrlBufferEndPtr)
       {
          if(isalnum(*url) || *url == '-' || *url == '_' || *url == '.')
            *getUrlBufferPtr++ = *url++;
          else
          {
            *getUrlBufferPtr++ = '%';
            *getUrlBufferPtr++ = h[*url >> 4];
            *getUrlBufferPtr++ = h[*url & 0x0f];
            url++;
          }
       }
    }
    if(getUrlBufferPtr == args->data)
      return "Mountpoint required.";
    else if(getUrlBufferPtr >= getUrlBufferEndPtr-1)
      return "Parsing buffer too short.";
    *(getUrlBufferPtr++) = 0;
  }

  if(*url == '/') /* username and password */
  {
    ++url;
    args->user = getUrlBufferPtr;
    while(*url && *url != '@' && *url != ';' && *url != ':' && getUrlBufferPtr != getUrlBufferEndPtr)
      *(getUrlBufferPtr++) = *(url++);
    if(getUrlBufferPtr == args->user)
      return "Username cannot be empty.";
    else if(getUrlBufferPtr >= getUrlBufferEndPtr-1)
      return "Parsing buffer too short.";
    *(getUrlBufferPtr++) = 0;

    if(*url == ':') ++url;

    args->password = getUrlBufferPtr;
    while(*url && *url != '@' && *url != ';' && getUrlBufferPtr != getUrlBufferEndPtr)
      *(getUrlBufferPtr++) = *(url++);
    if(getUrlBufferPtr == args->password)
      return "Password cannot be empty.";
    else if(getUrlBufferPtr >= getUrlBufferEndPtr-1)
      return "Parsing buffer too short.";
    *(getUrlBufferPtr++) = 0;
  }

  if(*url == '@') /* server */
  {
    ++url;
    if(*url != '@' && *url != ':')
    {
      args->server = getUrlBufferPtr;
      while(*url && *url != '@' && *url != ':' && *url != ';' && getUrlBufferPtr != getUrlBufferEndPtr)
        *(getUrlBufferPtr++) = *(url++);
      if(getUrlBufferPtr == args->server)
        return "Servername cannot be empty.";
      else if(getUrlBufferPtr >= getUrlBufferEndPtr-1)
        return "Parsing buffer too short.";
      *(getUrlBufferPtr++) = 0;
    }

    if(*url == ':')
    {
      ++url;
      args->port = getUrlBufferPtr;
      while(*url && *url != '@' && *url != ';' && getUrlBufferPtr != getUrlBufferEndPtr)
        *(getUrlBufferPtr++) = *(url++);
      if(getUrlBufferPtr == args->port)
        return "Port cannot be empty.";
      else if(getUrlBufferPtr >= getUrlBufferEndPtr-1)
        return "Parsing buffer too short.";
      *(getUrlBufferPtr++) = 0;
    }

    if(*url == '@') /* proxy */
    {
      ++url;
      args->proxyhost = getUrlBufferPtr;
      while(*url && *url != ':' && *url != ';' && getUrlBufferPtr != getUrlBufferEndPtr)
        *(getUrlBufferPtr++) = *(url++);
      if(getUrlBufferPtr == args->proxyhost)
        return "Proxy servername cannot be empty.";
      else if(getUrlBufferPtr >= getUrlBufferEndPtr-1)
        return "Parsing buffer too short.";
      *(getUrlBufferPtr++) = 0;

      if(*url == ':')
      {
        ++url;
        args->proxyport = getUrlBufferPtr;
        while(*url && *url != ';' && getUrlBufferPtr != getUrlBufferEndPtr)
          *(getUrlBufferPtr++) = *(url++);
        if(getUrlBufferPtr == args->proxyport)
          return "Proxy port cannot be empty.";
        else if(getUrlBufferPtr >= getUrlBufferEndPtr-1)
          return "Parsing buffer too short.";
        *(getUrlBufferPtr++) = 0;
      }
    }
  }
  if(*url == ';') /* NMEA */
  {
    args->nmea = ++url;
    while(*url)
      ++url;
  }

  return *url ? "Garbage at end of server string." : nullptr;
}

int NTRIPThread::getargs(const int argc, const char* const* argv, struct Args* const args)
{
  int res = 1;
  int getoptr;
  int help = 0;

  args->server = "www.euref-ip.net";
  args->port = "2101";
  args->user = "";
  args->password = "";
  args->nmea = nullptr;
  args->data = nullptr;
  args->bitrate = 0;
  args->proxyhost = nullptr;
  args->proxyport = "2101";
  args->mode = AUTO;
  args->initudp = 0;
  args->udpport = 0;
  args->serlogfile = nullptr;
  help = 0;

  do
  {
#ifdef NO_LONG_OPTS
    switch((getoptr = getopt(argc, argv, ARGOPT)))
#else
    switch((getoptr = getopt_long(argc, (char**)argv, ARGOPT, opts, nullptr)))
#endif
    {
    case 's': args->server = optarg; break;
    case 'u': args->user = optarg; break;
    case 'p': args->password = optarg; break;
    case 'm':
      if(optarg && *optarg == '?')
        args->data = encodeurl(optarg);
      else
        args->data = optarg;
      break;
    case 'l': args->serlogfile = optarg; break;
    case 'I': args->initudp = 1; break;
    case 'P': args->udpport = strtol(optarg, nullptr, 10); break;
    case 'n': args->nmea = optarg; break;
    case 'b': args->bitrate = 1; break;
    case 'h': help=1; break;
    case 'r': args->port = optarg; break;
    case 'S': args->proxyhost = optarg; break;
    case 'R': args->proxyport = optarg; break;
    case 'M':
      args->mode = 0;
      if (!strcmp(optarg,"n") || !strcmp(optarg,"ntrip1"))
        args->mode = NTRIP1;
      else if(!strcmp(optarg,"h") || !strcmp(optarg,"http"))
        args->mode = HTTP;
      else if(!strcmp(optarg,"r") || !strcmp(optarg,"rtsp"))
        args->mode = RTSP;
      else if(!strcmp(optarg,"u") || !strcmp(optarg,"udp"))
        args->mode = UDP;
      else if(!strcmp(optarg,"a") || !strcmp(optarg,"auto"))
        args->mode = AUTO;
      else args->mode = atoi(optarg);
      if((args->mode == 0) || (args->mode >= END))
      {
        fprintf(stderr, "Mode %s unknown\n", optarg);
        res = 0;
      }
      break;
    case 1:
      {
        const char *err;
        if((err = geturl(optarg, args)))
        {
          fprintf(stderr, "%s\n\n", err);
          res = 0;
        }
      }
      break;
    case -1: break;
    }
  } while(getoptr != -1 && res);

  if(!res || help)
  {
    printf("Usage:\n-s server -u user ...\n"
    " -m " LONG_OPT("--mountpoint ") "the requested data set or sourcetable filtering criteria\n"
    " -s " LONG_OPT("--server     ") "the server name or address\n"
    " -p " LONG_OPT("--password   ") "the login password\n"
    " -r " LONG_OPT("--port       ") "the server port number (default 2101)\n"
    " -u " LONG_OPT("--user       ") "the user name\n"
    " -M " LONG_OPT("--mode       ") "mode for data request\n"
    "     Valid modes are:\n"
    "     1, h, http     NTRIP Version 2.0 Caster in TCP/IP mode\n"
    "     2, r, rtsp     NTRIP Version 2.0 Caster in RTSP/RTP mode\n"
    "     3, n, ntrip1   NTRIP Version 1.0 Caster\n"
    "     4, a, auto     automatic detection (default)\n"
    "     5, u, udp      NTRIP Version 2.0 Caster in UDP mode\n"
    "or using an URL:\nntrip:mountpoint[/user[:password]][@[server][:port][@proxyhost[:proxyport]]][;nmea]\n"
    "\nExpert options:\n"
    " -n " LONG_OPT("--nmea       ") "NMEA string for sending to server\n"
    " -b " LONG_OPT("--bitrate    ") "output bitrate\n"
    " -I " LONG_OPT("--initudp    ") "send initial UDP packet for firewall handling\n"
    " -P " LONG_OPT("--udpport    ") "set the local UDP port\n"
    " -S " LONG_OPT("--proxyhost  ") "proxy name or address\n"
    " -R " LONG_OPT("--proxyport  ") "proxy port, optional (default 2101)\n"
    );
    exit(1);
  }
  return res;
}

static const char encodingTable [64] = {
  'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
  'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
  'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
  'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
int NTRIPThread::encode(char *buf, int size, const char *user, const char *pwd)
{
  unsigned char inbuf[3];
  char *out = buf;
  int i, sep = 0, fill = 0, bytes = 0;

  while(*user || *pwd)
  {
    i = 0;
    while(i < 3 && *user) inbuf[i++] = static_cast<unsigned char>(*(user++));
    if(i < 3 && !sep)    {inbuf[i++] = ':'; ++sep; }
    while(i < 3 && *pwd)  inbuf[i++] = static_cast<unsigned char>(*(pwd++));
    while(i < 3)         {inbuf[i++] = 0; ++fill; }
    if(out-buf < size-1)
      *(out++) = encodingTable[(inbuf [0] & 0xFC) >> 2];
    if(out-buf < size-1)
      *(out++) = encodingTable[((inbuf [0] & 0x03) << 4)
               | ((inbuf [1] & 0xF0) >> 4)];
    if(out-buf < size-1)
    {
      if(fill == 2)
        *(out++) = '=';
      else
        *(out++) = encodingTable[((inbuf [1] & 0x0F) << 2)
                 | ((inbuf [2] & 0xC0) >> 6)];
    }
    if(out-buf < size-1)
    {
      if(fill >= 1)
        *(out++) = '=';
      else
        *(out++) = encodingTable[inbuf [2] & 0x3F];
    }
    bytes += 4;
  }
  if(out-buf < size)
    *out = 0;
  return bytes;
}

int NTRIPThread::main(const int argc, const char* const* const argv)
{
    struct Args args;

    setbuf(stdout, nullptr);
    setbuf(stdin, nullptr);
    setbuf(stderr, nullptr);
#ifndef WINDOWSVERSION
//    signal(SIGALRM,sighandler_alarm);
//    signal(SIGINT,sighandler_int);
//    alarm(ALARMTIME);
#else
    WSADATA wsaData;
    if(WSAStartup(MAKEWORD(1,1),&wsaData))
    {
        fprintf(stderr, "Could not init network access.\n");
        return 20;
    }
#endif

    if(getargs(argc, argv, &args))
    {
        FILE *ser = nullptr;
        int sleeptime = 0;
        if(args.serlogfile)
        {
            if(!(ser = fopen(args.serlogfile, "a+")))
            {
                fprintf(stderr, "Could not open serial logfile.\n");
                return 20;
            }
        }
        do
        {
            int error = 0;
            sockettype sockfd = 0;
            int numbytes;
            char buf[MAXDATASIZE];
            struct sockaddr_in their_addr; /* connector's address information */
            struct hostent *he = nullptr;
            struct servent *se;
            const char *server, *port, *proxyserver = nullptr;
            char proxyport[6];
            char *b;
            long i = 0;
            if(sleeptime)
            {
#ifdef WINDOWSVERSION
                Sleep(static_cast<DWORD>(sleeptime*1000));
#else
                sleep(sleeptime);
#endif
                sleeptime += 2;
            }
            else
            {
                sleeptime = 1;
            }
#ifndef WINDOWSVERSION
            alarm(ALARMTIME);
#endif
            if(args.proxyhost)
            {
                int p = 0;
                if((i = strtol(args.port, &b, 10)) && (!b || !*b))
                    p = i;
                else if(!(se = getservbyname(args.port, nullptr)))
                {
                    fprintf(stderr, "Can't resolve port %s.", args.port);
                    stop = 1;
                }
                else
                {
                    p = ntohs(static_cast<u_short>(se->s_port));
                }
                if(!stop && !error)
                {
                    snprintf(proxyport, sizeof(proxyport), "%d", p);
                    port = args.proxyport;
                    proxyserver = args.server;
                    server = args.proxyhost;
                }
            }
            else
            {
                server = args.server;
                port = args.port;
            }
            if(!stop && !error)
            {
                memset(&their_addr, 0, sizeof(struct sockaddr_in));
                if((i = strtol(port, &b, 10)) && (!b || !*b))
                    their_addr.sin_port = static_cast<u_short>(htons(i));
                else if(!(se = getservbyname(port, nullptr)))
                {
                    fprintf(stderr, "Can't resolve port %s.", port);
                    stop = 1;
                }
                else
                {
                    their_addr.sin_port = static_cast<u_short>(se->s_port);
                }
                if(!stop && !error)
                {
                    if(!(he=gethostbyname(server)))
                    {
                        fprintf(stderr, "Server name lookup failed for '%s'.\n", server);
                        error = 1;
                    }
                    else if((sockfd = socket(AF_INET, (args.mode == static_cast<int>(UDP) ? SOCK_DGRAM :
                                                       SOCK_STREAM), 0)) == -1)
                    {
                        myperror("socket");
                        error = 1;
                    }
                    else
                    {
                        their_addr.sin_family = AF_INET;
                        their_addr.sin_addr = *((struct in_addr *)he->h_addr);
                    }
                }
            }
            if(!stop && !error)
            {
                if(args.mode == UDP)
                {
                    unsigned int session;
                    int tim, seq, init;
                    char rtpbuf[1526];
                    int i=12, j;

                    init = time(0);
                    srand(init);
                    session = rand();
                    tim = rand();
                    seq = rand();

                    rtpbuf[0] = (2<<6);
                    /* padding, extension, csrc are empty */
                    rtpbuf[1] = 97;
                    /* marker is empty */
                    rtpbuf[2] = (seq>>8)&0xFF;
                    rtpbuf[3] = (seq)&0xFF;
                    rtpbuf[4] = (tim>>24)&0xFF;
                    rtpbuf[5] = (tim>>16)&0xFF;
                    rtpbuf[6] = (tim>>8)&0xFF;
                    rtpbuf[7] = (tim)&0xFF;
                    /* sequence and timestamp are empty */
                    rtpbuf[8] = (session>>24)&0xFF;
                    rtpbuf[9] = (session>>16)&0xFF;
                    rtpbuf[10] = (session>>8)&0xFF;
                    rtpbuf[11] = (session)&0xFF;
                    ++seq;

                    j = snprintf(rtpbuf+i, sizeof(rtpbuf)-i-40, /* leave some space for login */
                                 "GET /%s HTTP/1.1\r\n"
                                 "Host: %s\r\n"
                                 "Ntrip-Version: Ntrip/2.0\r\n"
                                 "User-Agent: %s/%s\r\n"
                                 "%s%s%s"
                                 "Connection: close%s",
                                 args.data ? args.data : "", args.server, AGENTSTRING, revisionstr,
                                 args.nmea ? "Ntrip-GGA: " : "", args.nmea ? args.nmea : "",
                                 args.nmea ? "\r\n" : "",
                                 (*args.user || *args.password) ? "\r\nAuthorization: Basic " : "");
                    i += j;
                    if(i > (int)sizeof(rtpbuf)-40 || j < 0) /* second check for old glibc */
                    {
                        fprintf(stderr, "Requested data too long\n");
                        stop = 1;
                    }
                    else
                    {
                        i += encode(rtpbuf+i, sizeof(rtpbuf)-i-4, args.user, args.password);
                        if(i > (int)sizeof(rtpbuf)-4)
                        {
                            fprintf(stderr, "Username and/or password too long\n");
                            stop = 1;
                        }
                        else
                        {
                            struct sockaddr_in local;
                            socklen_t len;

                            rtpbuf[i++] = '\r';
                            rtpbuf[i++] = '\n';
                            rtpbuf[i++] = '\r';
                            rtpbuf[i++] = '\n';


                            /* fill structure with local address information for UDP */
                            memset(&local, 0, sizeof(local));
                            local.sin_family = AF_INET;
                            local.sin_port = htons(args.udpport);
                            local.sin_addr.s_addr = htonl(INADDR_ANY);
                            len = sizeof(local);

                            /* bind() in order to get a random RTP client_port */
                            if((bind(sockfd, (struct sockaddr *)&local, len)) < 0)
                            {
                                myperror("bind");
                                error = 1;
                            }
                            else if(::connect(sockfd, (struct sockaddr *)&their_addr,
                                            sizeof(struct sockaddr)) == -1)
                            {
                                myperror("connect");
                                error = 1;
                            }
                            else if(send(sockfd, rtpbuf, i, 0) != i)
                            {
                                myperror("Could not send UDP packet");
                                stop = 1;
                            }
                            else
                            {
                                if((numbytes=recv(sockfd, rtpbuf, sizeof(rtpbuf)-1, 0)) > 0)
                                {
                                    int sn = 0x10000, ts=0;
                                    /* we don't expect message longer than 1513, so we cut the last
                    byte for security reasons to prevent buffer overrun */
                                    rtpbuf[numbytes] = 0;
                                    if(numbytes > 17+12 &&
                                            (!strncmp(rtpbuf+12, "HTTP/1.1 200 OK\r\n", 17) ||
                                             !strncmp(rtpbuf+12, "HTTP/1.0 200 OK\r\n", 17)))
                                    {
                                        const char *sessioncheck = "session: ";
                                        const char *datacheck = "Content-Type: gnss/data\r\n";
                                        const char *sourcetablecheck = "Content-Type: gnss/sourcetable\r\n";
                                        const char *contentlengthcheck = "Content-Length: ";
                                        const char *httpresponseend = "\r\n\r\n";
                                        int contentlength = 0, httpresponselength = 0;
                                        /* datacheck */
                                        int l = strlen(datacheck)-1;
                                        int j=0;
                                        for(i = 12; j != l && i < numbytes-l; ++i)
                                        {
                                            for(j = 0; j < l && rtpbuf[i+j] == datacheck[j]; ++j)
                                                ;
                                        }
                                        if(i != numbytes-l)
                                        {
                                            /* check for Session */
                                            l = strlen(sessioncheck)-1;
                                            j=0;
                                            for(i = 12; j != l && i < numbytes-l; ++i)
                                            {
                                                for(j = 0; j < l && tolower(rtpbuf[i+j]) == sessioncheck[j]; ++j)
                                                    ;
                                            }
                                            if(i != numbytes-l) /* found a session number */
                                            {
                                                i+=l;
                                                session = 0;
                                                while(i < numbytes && rtpbuf[i] >= '0' && rtpbuf[i] <= '9')
                                                    session = session * 10 + rtpbuf[i++]-'0';
                                                if(rtpbuf[i] != '\r')
                                                {
                                                    fprintf(stderr, "Could not extract session number\n");
                                                    stop = 1;
                                                }
                                            }
                                        }
                                        else
                                        {
                                            /* sourcetablecheck */
                                            l = strlen(sourcetablecheck)-1;
                                            j=0;
                                            for(i = 12; j != l && i < numbytes-l; ++i)
                                            {
                                                for(j = 0; j < l && rtpbuf[i+j] == sourcetablecheck[j]; ++j)
                                                    ;
                                            }
                                            if(i == numbytes-l)
                                            {
                                                fprintf(stderr, "No 'Content-Type: gnss/data' or"
                                                                " 'Content-Type: gnss/sourcetable' found\n");
                                                error = 1;
                                            }
                                            else
                                            {
                                                /* check for http response end */
                                                l = strlen(httpresponseend)-1;
                                                j=0;
                                                for(i = 12; j != l && i < numbytes-l; ++i)
                                                {
                                                    for(j = 0; j < l && rtpbuf[i+j] == httpresponseend[j]; ++j)
                                                        ;
                                                }
                                                if(i != numbytes-l) /* found http response end */
                                                {
                                                    httpresponselength = i+3-12;
                                                }
                                                /* check for content length */
                                                l = strlen(contentlengthcheck)-1;
                                                j=0;
                                                for(i = 12; j != l && i < numbytes-l; ++i)
                                                {
                                                    for(j = 0; j < l && rtpbuf[i+j] == contentlengthcheck[j]; ++j)
                                                        ;
                                                }
                                                if(i != numbytes-l) /* found content length */
                                                {
                                                    i+=l;
                                                    contentlength = 0;
                                                    while(i < numbytes && rtpbuf[i] >= '0' && rtpbuf[i] <= '9')
                                                        contentlength = contentlength * 10 + rtpbuf[i++]-'0';
                                                    if(rtpbuf[i] == '\r')
                                                    {
                                                        contentlength += httpresponselength;
                                                        do
                                                        {
                                                            fwrite(rtpbuf+12, (size_t)numbytes-12, 1, stdout);
                                                            if((contentlength -= (numbytes-12)) == 0)
                                                            {
                                                                stop = 1;
                                                            }
                                                            else
                                                            {
                                                                numbytes = recv(sockfd, rtpbuf, sizeof(rtpbuf), 0);
                                                            }
                                                        }while((numbytes >12) && (!stop));
                                                    }
                                                    else
                                                    {
                                                        fprintf(stderr, "Could not extract content length\n");
                                                        stop = 1;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        int k;
                                        fprintf_error_buffered("Could not get the requested data: ");
                                        for(k = 12; k < numbytes && rtpbuf[k] != '\n' && rtpbuf[k] != '\r'; ++k)
                                        {
                                            fprintf_error_buffered("%c", isprint(rtpbuf[k]) ? rtpbuf[k] : '.');
                                        }
                                        fprintf_error_buffered_flush();
                                        error = 1;
                                    }
                                    while(!stop && !error)
                                    {
                                        struct timeval tv = {1,0};
                                        fd_set fdr;
                                        fd_set fde;

                                        FD_ZERO(&fdr);
                                        FD_ZERO(&fde);
                                        FD_SET(sockfd, &fdr);
                                        FD_SET(sockfd, &fde);
                                        if(select(sockfd+1,&fdr,0,&fde,&tv) < 0)
                                        {
                                            fprintf(stderr, "Select problem.\n");
                                            error = 1;
                                            continue;
                                        }
                                        i = recv(sockfd, rtpbuf, sizeof(rtpbuf), 0);
#ifndef WINDOWSVERSION
                                        alarm(ALARMTIME);
#endif
                                        if(i >= 12 && (unsigned char)rtpbuf[0] == (2 << 6)
                                                && rtpbuf[1] >= 96 && rtpbuf[1] <= 98)
                                        {
                                            time_t ct;
                                            int u,v;
                                            unsigned int w;
                                            u = ((unsigned char)rtpbuf[2]<<8)+(unsigned char)rtpbuf[3];
                                            v = ((unsigned char)rtpbuf[4]<<24)+((unsigned char)rtpbuf[5]<<16)
                                                    +((unsigned char)rtpbuf[6]<<8)+(unsigned char)rtpbuf[7];
                                            w = ((unsigned char)rtpbuf[8]<<24)+((unsigned char)rtpbuf[9]<<16)
                                                    +((unsigned char)rtpbuf[10]<<8)+(unsigned char)rtpbuf[11];

                                            if(sn == 0x10000) {sn = u-1;ts=v-1;}
                                            else if(u < -30000 && sn > 30000) sn -= 0xFFFF;
                                            if(session != w || ts > v)
                                            {
                                                fprintf(stderr, "Illegal UDP data received.\n");
                                                continue;
                                            }
                                            else if(u > sn) /* don't show out-of-order packets */
                                            {
                                                if(rtpbuf[1] == 98)
                                                {
                                                    fprintf(stderr, "Connection closed.\n");
                                                    error = 1;
                                                    continue;
                                                }
                                                else if((rtpbuf[1] == 96)  && (i>12))
                                                {
                                                    fwrite(rtpbuf+12, (size_t)i-12, 1, stdout);
                                                }
                                            }
                                            sn = u; ts = v;

                                            /* Keep Alive */
                                            ct = time(0);
                                            if(ct-init > 15)
                                            {
                                                tim += (ct-init)*1000000/TIME_RESOLUTION;
                                                rtpbuf[0] = (2<<6);
                                                /* padding, extension, csrc are empty */
                                                rtpbuf[1] = 96;
                                                /* marker is empty */
                                                rtpbuf[2] = (seq>>8)&0xFF;
                                                rtpbuf[3] = (seq)&0xFF;
                                                rtpbuf[4] = (tim>>24)&0xFF;
                                                rtpbuf[5] = (tim>>16)&0xFF;
                                                rtpbuf[6] = (tim>>8)&0xFF;
                                                rtpbuf[7] = (tim)&0xFF;
                                                /* sequence and timestamp are empty */
                                                rtpbuf[8] = (session>>24)&0xFF;
                                                rtpbuf[9] = (session>>16)&0xFF;
                                                rtpbuf[10] = (session>>8)&0xFF;
                                                rtpbuf[11] = (session)&0xFF;
                                                ++seq;
                                                init = ct;

                                                if(send(sockfd, rtpbuf, 12, 0) != 12)
                                                {
                                                    myperror("send");
                                                    error = 1;
                                                }
                                            }
                                        }
                                        else if(i >= 0)
                                        {
                                            fprintf(stderr, "Illegal UDP header.\n");
                                            continue;
                                        }
                                    }
                                }
                                /* send connection close always to allow nice session closing */
                                tim += (time(0)-init)*1000000/TIME_RESOLUTION;
                                rtpbuf[0] = (2<<6);
                                /* padding, extension, csrc are empty */
                                rtpbuf[1] = 98;
                                /* marker is empty */
                                rtpbuf[2] = (seq>>8)&0xFF;
                                rtpbuf[3] = (seq)&0xFF;
                                rtpbuf[4] = (tim>>24)&0xFF;
                                rtpbuf[5] = (tim>>16)&0xFF;
                                rtpbuf[6] = (tim>>8)&0xFF;
                                rtpbuf[7] = (tim)&0xFF;
                                /* sequence and timestamp are empty */
                                rtpbuf[8] = (session>>24)&0xFF;
                                rtpbuf[9] = (session>>16)&0xFF;
                                rtpbuf[10] = (session>>8)&0xFF;
                                rtpbuf[11] = (session)&0xFF;

                                send(sockfd, rtpbuf, 12, 0); /* cleanup */
                            }
                        }
                    }
                }
                else if(args.data && *args.data != '%' && args.mode == RTSP)
                {
                    struct sockaddr_in local;
                    sockettype sockudp = 0;
                    int localport = 0;
                    int cseq = 1;
                    socklen_t len;

                    if((sockudp = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
                    {
                        myperror("socket");
                        error = 1;
                    }
                    if(!stop && !error)
                    {
                        /* fill structure with local address information for UDP */
                        memset(&local, 0, sizeof(local));
                        local.sin_family = AF_INET;
                        local.sin_port = htons(args.udpport);
                        local.sin_addr.s_addr = htonl(INADDR_ANY);
                        len = sizeof(local);
                        /* bind() in order to get a random RTP client_port */
                        if((bind(sockudp, (struct sockaddr *)&local, len)) < 0)
                        {
                            myperror("bind");
                            error = 1;
                        }
                        else if((::getsockname(sockudp, (struct sockaddr*)&local, &len)) == -1)
                        {
                            myperror("local access failed");
                            error = 1;
                        }
                        else if(::connect(sockfd, (struct sockaddr *)&their_addr,
                                        sizeof(struct sockaddr)) == -1)
                        {
                            myperror("connect");
                            error = 1;
                        }
                        localport = ntohs(local.sin_port);
                    }
                    if(!stop && !error)
                    {
                        i=snprintf(buf, MAXDATASIZE-40, /* leave some space for login */
                                   "SETUP rtsp://%s%s%s/%s RTSP/1.0\r\n"
                                   "CSeq: %d\r\n"
                                   "Ntrip-Version: Ntrip/2.0\r\n"
                                   "Ntrip-Component: Ntripclient\r\n"
                                   "User-Agent: %s/%s\r\n"
                                   "%s%s%s"
                                   "Transport: RTP/GNSS;unicast;client_port=%u%s",
                                   args.server, proxyserver ? ":" : "", proxyserver ? args.port : "",
                                   args.data, cseq++, AGENTSTRING, revisionstr,
                                   args.nmea ? "Ntrip-GGA: " : "", args.nmea ? args.nmea : "",
                                   args.nmea ? "\r\n" : "",
                                   localport,
                                   (*args.user || *args.password) ? "\r\nAuthorization: Basic " : "");
                        if(i > MAXDATASIZE-40 || i < 0) /* second check for old glibc */
                        {
                            fprintf(stderr, "Requested data too long\n");
                            stop = 1;
                        }
                        i += encode(buf+i, MAXDATASIZE-i-4, args.user, args.password);
                        if(i > MAXDATASIZE-4)
                        {
                            fprintf(stderr, "Username and/or password too long\n");
                            stop = 1;
                        }
                        buf[i++] = '\r';
                        buf[i++] = '\n';
                        buf[i++] = '\r';
                        buf[i++] = '\n';
                    }
                    if(!stop && !error)
                    {
                        if(send(sockfd, buf, (size_t)i, 0) != i)
                        {
                            myperror("send");
                            error = 1;
                        }
                        else if((numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) == -1)
                        {
                            myperror("recv");
                            error = 1;
                        }
                        else if(numbytes >= 17 && !strncmp(buf, "RTSP/1.0 200 OK\r\n", 17))
                        {
                            int serverport = 0, session = 0;
                            const char *portcheck = "server_port=";
                            const char *sessioncheck = "session: ";
                            int l = strlen(portcheck)-1;
                            int j=0;
                            for(i = 0; j != l && i < numbytes-l; ++i)
                            {
                                for(j = 0; j < l && tolower(buf[i+j]) == portcheck[j]; ++j)
                                    ;
                            }
                            if(i == numbytes-l)
                            {
                                fprintf(stderr, "No server port number found\n");
                                stop = 1;
                            }
                            else
                            {
                                i+=l;
                                while(i < numbytes && buf[i] >= '0' && buf[i] <= '9')
                                    serverport = serverport * 10 + buf[i++]-'0';
                                if(buf[i] != '\r' && buf[i] != ';')
                                {
                                    fprintf(stderr, "Could not extract server port\n");
                                    stop = 1;
                                }
                            }
                            if(!stop && !error)
                            {
                                l = strlen(sessioncheck)-1;
                                j=0;
                                for(i = 0; j != l && i < numbytes-l; ++i)
                                {
                                    for(j = 0; j < l && tolower(buf[i+j]) == sessioncheck[j]; ++j)
                                        ;
                                }
                                if(i == numbytes-l)
                                {
                                    fprintf(stderr, "No session number found\n");
                                    stop = 1;
                                }
                                else
                                {
                                    i+=l;
                                    while(i < numbytes && buf[i] >= '0' && buf[i] <= '9')
                                        session = session * 10 + buf[i++]-'0';
                                    if(buf[i] != '\r')
                                    {
                                        fprintf(stderr, "Could not extract session number\n");
                                        stop = 1;
                                    }
                                }
                            }
                            if(!stop && !error && args.initudp)
                            {
                                printf("Sending initial UDP packet\n");
                                struct sockaddr_in casterRTP;
                                char rtpbuffer[12];
                                int i;
                                rtpbuffer[0] = (2<<6);
                                /* padding, extension, csrc are empty */
                                rtpbuffer[1] = 96;
                                /* marker is empty */
                                rtpbuffer[2] = 0;
                                rtpbuffer[3] = 0;
                                rtpbuffer[4] = 0;
                                rtpbuffer[5] = 0;
                                rtpbuffer[6] = 0;
                                rtpbuffer[7] = 0;
                                /* sequence and timestamp are empty */
                                rtpbuffer[8] = (session>>24)&0xFF;
                                rtpbuffer[9] = (session>>16)&0xFF;
                                rtpbuffer[10] = (session>>8)&0xFF;
                                rtpbuffer[11] = (session)&0xFF;
                                /* fill structure with caster address information for UDP */
                                memset(&casterRTP, 0, sizeof(casterRTP));
                                casterRTP.sin_family = AF_INET;
                                casterRTP.sin_port   = htons(serverport);
                                casterRTP.sin_addr   = *((struct in_addr *)he->h_addr);

                                if((i = sendto(sockudp, rtpbuffer, 12, 0,
                                               (struct sockaddr *) &casterRTP, sizeof(casterRTP))) != 12)
                                    myperror("WARNING: could not send initial UDP packet");
                            }
                            if(!stop && !error)
                            {
                                i = snprintf(buf, MAXDATASIZE,
                                             "PLAY rtsp://%s%s%s/%s RTSP/1.0\r\n"
                                             "CSeq: %d\r\n"
                                             "Session: %u\r\n"
                                             "\r\n",
                                             args.server, proxyserver ? ":" : "", proxyserver ? args.port : "",
                                             args.data, cseq++, session);

                                if(i > MAXDATASIZE || i < 0) /* second check for old glibc */
                                {
                                    fprintf(stderr, "Requested data too long\n");
                                    stop=1;
                                }
                                else if(send(sockfd, buf, (size_t)i, 0) != i)
                                {
                                    myperror("send");
                                    error = 1;
                                }
                                else if((numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) != -1)
                                {
                                    if(numbytes >= 17 && !strncmp(buf, "RTSP/1.0 200 OK\r\n", 17))
                                    {
                                        int ts = 0, sn = 0;
                                        time_t init = 0;
                                        struct sockaddr_in addrRTP;
#ifdef WINDOWSVERSION
                                        u_long blockmode = 1;
                                        if(ioctlsocket(sockudp, FIONBIO, &blockmode)
                                                || ioctlsocket(sockfd, FIONBIO, &blockmode))
#else /* WINDOWSVERSION */
                                        if(fcntl(sockfd, F_SETFL, O_NONBLOCK) < 0
                                                || fcntl(sockudp, F_SETFL, O_NONBLOCK) < 0)
#endif /* WINDOWSVERSION */
                                        {
                                            fprintf(stderr, "Could not set nonblocking mode\n");
                                            error = 1;
                                        }

                                        /* fill structure with caster address information for UDP */
                                        memset(&addrRTP, 0, sizeof(addrRTP));
                                        addrRTP.sin_family = AF_INET;
                                        addrRTP.sin_port   = htons(serverport);
                                        their_addr.sin_addr = *((struct in_addr *)he->h_addr);
                                        len = sizeof(addrRTP);

                                        while(!stop && !error)
                                        {
                                            char rtpbuffer[1526];
                                            struct timeval tv = {1,0};
                                            fd_set fdr;
                                            fd_set fde;
                                            int r;

                                            FD_ZERO(&fdr);
                                            FD_ZERO(&fde);
                                            FD_SET(sockudp, &fdr);
                                            FD_SET(sockfd, &fdr);
                                            FD_SET(sockudp, &fde);
                                            FD_SET(sockfd, &fde);
                                            if(select((sockudp>sockfd?sockudp:sockfd)+1,
                                                      &fdr,0,&fde,&tv) < 0)
                                            {
                                                fprintf(stderr, "Select problem.\n");
                                                error = 1;
                                                continue;
                                            }
                                            i = ::recvfrom(sockudp, rtpbuffer, sizeof(rtpbuffer), 0,
                                                         (struct sockaddr*) &addrRTP, &len);
#ifndef WINDOWSVERSION
                                            alarm(ALARMTIME);
#endif
                                            if(i >= 12+1 && (unsigned char)rtpbuffer[0] == (2 << 6) && rtpbuffer[1] == 0x60)
                                            {
                                                int u,v,w;
                                                u = ((unsigned char)rtpbuffer[2]<<8)+(unsigned char)rtpbuffer[3];
                                                v = ((unsigned char)rtpbuffer[4]<<24)+((unsigned char)rtpbuffer[5]<<16)
                                                        +((unsigned char)rtpbuffer[6]<<8)+(unsigned char)rtpbuffer[7];
                                                w = ((unsigned char)rtpbuffer[8]<<24)+((unsigned char)rtpbuffer[9]<<16)
                                                        +((unsigned char)rtpbuffer[10]<<8)+(unsigned char)rtpbuffer[11];

                                                if(init)
                                                {
                                                    time_t ct;
                                                    if(u < -30000 && sn > 30000) sn -= 0xFFFF;
                                                    if(session != w || ts > v)
                                                    {
                                                        fprintf(stderr, "Illegal UDP data received.\n");
                                                        continue;
                                                    }
                                                    else if(u > sn) /* don't show out-of-order packets */
                                                        fwrite(rtpbuffer+12, (size_t)i-12, 1, stdout);
                                                    ct = time(0);
                                                    if(ct-init > 15)
                                                    {
                                                        i = snprintf(buf, MAXDATASIZE,
                                                                     "GET_PARAMETER rtsp://%s%s%s/%s RTSP/1.0\r\n"
                                                                     "CSeq: %d\r\n"
                                                                     "Session: %u\r\n"
                                                                     "\r\n",
                                                                     args.server, proxyserver ? ":" : "", proxyserver
                                                                     ? args.port : "", args.data, cseq++, session);
                                                        if(i > MAXDATASIZE || i < 0)
                                                        {
                                                            fprintf(stderr, "Requested data too long\n");
                                                            stop = 1;
                                                        }
                                                        else if(send(sockfd, buf, (size_t)i, 0) != i)
                                                        {
                                                            myperror("send");
                                                            error = 1;
                                                        }
                                                        init = ct;
                                                    }
                                                }
                                                else
                                                {
                                                    init = time(0);
                                                }
                                                sn = u; ts = v;
                                            }
                                            else if(i >= 0)
                                            {
                                                fprintf(stderr, "Illegal UDP header.\n");
                                                continue;
                                            }
                                            /* ignore RTSP server replies */
                                            if((r=recv(sockfd, buf, MAXDATASIZE-1, 0)) < 0)
                                            {
#ifdef WINDOWSVERSION
                                                if(WSAGetLastError() != WSAEWOULDBLOCK)
#else /* WINDOWSVERSION */
                                                if(errno != EAGAIN)
#endif /* WINDOWSVERSION */
                                                {
                                                    fprintf(stderr, "Control connection closed\n");
                                                    error = 1;
                                                }
                                            }
                                            else if(!r)
                                            {
                                                fprintf(stderr, "Control connection read error\n");
                                                error = 1;
                                            }
                                        }
                                    }
                                    i = snprintf(buf, MAXDATASIZE,
                                                 "TEARDOWN rtsp://%s%s%s/%s RTSP/1.0\r\n"
                                                 "CSeq: %d\r\n"
                                                 "Session: %u\r\n"
                                                 "\r\n",
                                                 args.server, proxyserver ? ":" : "", proxyserver ? args.port : "",
                                                 args.data, cseq++, session);

                                    if(i > MAXDATASIZE || i < 0) /* second check for old glibc */
                                    {
                                        fprintf(stderr, "Requested data too long\n");
                                        stop = 1;
                                    }
                                    else if(send(sockfd, buf, (size_t)i, 0) != i)
                                    {
                                        myperror("send");
                                        error = 1;
                                    }
                                }
                                else
                                {
                                    fprintf(stderr, "Could not start data stream.\n");
                                    error = 1;
                                }
                            }
                        }
                        else
                        {
                            fprintf(stderr, "Could not setup initial control connection.\n");
                            error = 1;
                        }
                        if(sockudp)
                            closesocket(sockudp);
                    }
                }
                else
                {
                    if(::connect(sockfd, (struct sockaddr *)&their_addr,
                               sizeof(struct sockaddr)) == -1)
                    {
                        myperror("connect");
                        error = 1;
                    }
                    if(!stop && !error)
                    {
                        if(!args.data)
                        {
                            i = snprintf(buf, MAXDATASIZE,
                                         "GET %s%s%s%s/ HTTP/1.1\r\n"
                                         "Host: %s\r\n%s"
                                         "User-Agent: %s/%s\r\n"
                                         "Connection: close\r\n"
                                         "\r\n"
                                         , proxyserver ? "http://" : "", proxyserver ? proxyserver : "",
                                         proxyserver ? ":" : "", proxyserver ? proxyport : "",
                                         args.server, args.mode == NTRIP1 ? "" : "Ntrip-Version: Ntrip/2.0\r\n",
                                         AGENTSTRING, revisionstr);
                        }
                        else
                        {
                            const char *nmeahead = (args.nmea && args.mode == HTTP) ? args.nmea : 0;

                            i=snprintf(buf, MAXDATASIZE-40, /* leave some space for login */
                                       "GET %s%s%s%s/%s HTTP/1.1\r\n"
                                       "Host: %s\r\n%s"
                                       "User-Agent: %s/%s\r\n"
                                       "%s%s%s"
                                       "Connection: close%s"
                                       , proxyserver ? "http://" : "", proxyserver ? proxyserver : "",
                                       proxyserver ? ":" : "", proxyserver ? proxyport : "",
                                       args.data, args.server,
                                       args.mode == NTRIP1 ? "" : "Ntrip-Version: Ntrip/2.0\r\n",
                                       AGENTSTRING, revisionstr,
                                       nmeahead ? "Ntrip-GGA: " : "", nmeahead ? nmeahead : "",
                                       nmeahead ? "\r\n" : "",
                                       (*args.user || *args.password) ? "\r\nAuthorization: Basic " : "");
                            if(i > MAXDATASIZE-40 || i < 0) /* second check for old glibc */
                            {
                                fprintf(stderr, "Requested data too long\n");
                                stop = 1;
                            }
                            else
                            {
                                i += encode(buf+i, MAXDATASIZE-i-4, args.user, args.password);
                                if(i > MAXDATASIZE-4)
                                {
                                    fprintf(stderr, "Username and/or password too long\n");
                                    stop = 1;
                                }
                                else
                                {
                                    buf[i++] = '\r';
                                    buf[i++] = '\n';
                                    buf[i++] = '\r';
                                    buf[i++] = '\n';
                                    if(args.nmea && !nmeahead)
                                    {
                                        int j = snprintf(buf+i, MAXDATASIZE-i, "%s\r\n", args.nmea);
                                        if(j >= 0 && j < MAXDATASIZE-i)
                                            i += j;
                                        else
                                        {
                                            fprintf(stderr, "NMEA string too long\n");
                                            stop = 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if(!stop && !error)
                    {
                        if(send(sockfd, buf, (size_t)i, 0) != i)
                        {
                            myperror("send");
                            error = 1;
                        }
                        else if(args.data && *args.data != '%')
                        {
                            int k = 0;
                            int chunkymode = 0;
                            int starttime = time(0);
                            int lastout = starttime;
                            int totalbytes = 0;
                            int chunksize = 0;

                            while(!stop && !error &&
                                  (numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) > 0)
                            {
#ifndef WINDOWSVERSION
                                alarm(ALARMTIME);
#endif
                                if(!k)
                                {
                                    buf[numbytes] = 0; /* latest end mark for strstr */
                                    if( numbytes > 17 &&
                                            !strstr(buf, "ICY 200 OK")  &&  /* case 'proxy & ntrip 1.0 caster' */
                                            (!strncmp(buf, "HTTP/1.1 200 OK\r\n", 17) ||
                                             !strncmp(buf, "HTTP/1.0 200 OK\r\n", 17)) )
                                    {
                                        const char *datacheck = "Content-Type: gnss/data\r\n";
                                        const char *chunkycheck = "Transfer-Encoding: chunked\r\n";
                                        int l = strlen(datacheck)-1;
                                        int j=0;
                                        for(i = 0; j != l && i < numbytes-l; ++i)
                                        {
                                            for(j = 0; j < l && buf[i+j] == datacheck[j]; ++j)
                                                ;
                                        }
                                        if(i == numbytes-l)
                                        {
                                            fprintf(stderr, "No 'Content-Type: gnss/data' found\n");
                                            error = 1;
                                        }
                                        l = strlen(chunkycheck)-1;
                                        j=0;
                                        for(i = 0; j != l && i < numbytes-l; ++i)
                                        {
                                            for(j = 0; j < l && buf[i+j] == chunkycheck[j]; ++j)
                                                ;
                                        }
                                        if(i < numbytes-l)
                                            chunkymode = 1;
                                    }
                                    else if(!strstr(buf, "ICY 200 OK"))
                                    {
                                        fprintf_error_buffered("Could not get the requested data: ");
                                        for(k = 0; k < numbytes && buf[k] != '\n' && buf[k] != '\r'; ++k)
                                        {
                                            fprintf_error_buffered("%c", isprint(buf[k]) ? buf[k] : '.');
                                        }
                                        fprintf_error_buffered_flush();
                                        error = 1;
                                    }
                                    else if(args.mode != NTRIP1)
                                    {
                                        fprintf(stderr, "NTRIP version 2 HTTP connection failed%s.\n",
                                                args.mode == AUTO ? ", falling back to NTRIP1" : "");
                                        if(args.mode == HTTP)
                                            stop = 1;
                                    }
                                    k = 1;
                                    if(args.mode == NTRIP1)
                                        continue; /* skip old headers for NTRIP1 */
                                    else
                                    {
                                        char *ep = strstr(buf, "\r\n\r\n");
                                        if(!ep || ep+4 == buf+numbytes)
                                            continue;
                                        ep += 4;
                                        memmove(buf, ep, numbytes-(ep-buf));
                                        numbytes -= (ep-buf);
                                    }
                                }
                                sleeptime = 0;
                                if(chunkymode)
                                {
                                    int cstop = 0;
                                    int pos = 0;
                                    while(!stop && !cstop && !error && pos < numbytes)
                                    {
                                        switch(chunkymode)
                                        {
                                        case 1: /* reading number starts */
                                            chunksize = 0;
                                            ++chunkymode; /* no break */
                                        case 2: /* during reading number */
                                            i = buf[pos++];
                                            if(i >= '0' && i <= '9') chunksize = chunksize*16+i-'0';
                                            else if(i >= 'a' && i <= 'f') chunksize = chunksize*16+i-'a'+10;
                                            else if(i >= 'A' && i <= 'F') chunksize = chunksize*16+i-'A'+10;
                                            else if(i == '\r') ++chunkymode;
                                            else if(i == ';') chunkymode = 5;
                                            else cstop = 1;
                                            break;
                                        case 3: /* scanning for return */
                                            if(buf[pos++] == '\n') chunkymode = chunksize ? 4 : 1;
                                            else cstop = 1;
                                            break;
                                        case 4: /* output data */
                                            i = numbytes-pos;
                                            if(i > chunksize) i = chunksize;

                                            if (1)
                                            {
                                                int ofs = 0;
                                                while(i > ofs && !cstop && !stop && !error)
                                                {
                                                    // Original: int j = SerialWrite(&sx, buf+pos+ofs, i-ofs);

                                                    int j = rtcmDataWrite(buf+pos+ofs, i-ofs);

                                                    if(j < 0)
                                                    {
                                                        fprintf(stderr, "Could not access serial device\n");
                                                        stop = 1;
                                                    }
                                                    else
                                                        ofs += j;
                                                }
                                            }

                                            totalbytes += i;
                                            chunksize -= i;
                                            pos += i;
                                            if(!chunksize)
                                                chunkymode = 1;
                                            break;
                                        case 5:
                                            if(i == '\r') chunkymode = 3;
                                            break;
                                        }
                                    }
                                    if(cstop)
                                    {
                                        fprintf(stderr, "Error in chunky transfer encoding\n");
                                        error = 1;
                                    }
                                }
                                else
                                {
                                    totalbytes += numbytes;
                                    if (1)
                                    {
                                        int ofs = 0;
                                        while(numbytes > ofs && !stop)
                                        {
                                            int i = rtcmDataWrite(buf+ofs, numbytes-ofs);
                                            // Original: int i = SerialWrite(&sx, buf+ofs, numbytes-ofs);
                                            if(i < 0)
                                            {
                                                fprintf(stderr, "Could not access serial device\n");
                                                stop = 1;
                                            }
                                            else
                                                ofs += i;
                                        }
                                    }
                                }
                                fflush(stdout);
                                if(totalbytes < 0) /* overflow */
                                {
                                    totalbytes = 0;
                                    starttime = time(0);
                                    lastout = starttime;
                                }

                                if(args.bitrate)
                                {
                                    int t = time(0);
                                    if(t > lastout + 60)
                                    {
                                        lastout = t;
                                        printf("Bitrate is %dbyte/s (%d seconds accumulated).\n",
                                                totalbytes/(t-starttime), t-starttime);
                                    }
                                }
                            }
                        }
                        else
                        {
                            sleeptime = 0;
                            while(!stop && (numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) > 0)
                            {
#ifndef WINDOWSVERSION
                                alarm(ALARMTIME);
#endif
                                fwrite(buf, (size_t)numbytes, 1, stdout);
                            }
                        }
                    }
                }
            }
            if(sockfd)
                closesocket(sockfd);

#ifdef WINDOWSVERSION
                Sleep((DWORD)(1000));
#else
                sleep(1);
#endif

        } while(args.data && *args.data != '%' && !stop);

        if(ser)
            fclose(ser);
    }
    return 0;
}


#pragma GCC diagnostic pop

int NTRIPThread::fprintf (FILE *stream, const char *format, ...)
{
    va_list argList;
    va_start(argList, format);

    int retval;

    if ((stream == stderr) || (stream == stdout))
    {
        QString outString = QString::vasprintf(format, argList);
        retval = outString.length();

        if (stream == stderr)
        {
            emit errorMessage("\n" + outString.trimmed());
        }
        else if (stream == stdout)
        {
            emit infoMessage("\n" + outString.trimmed());
        }
    }
    else
    {
        retval = vfprintf(stream, format, argList);
    }

    va_end(argList);

    return retval;
}

int NTRIPThread::fprintf_error_buffered(const char *__format, ...)
{
    va_list argList;
    va_start(argList, __format);

    int retval;

    QString outString = QString::vasprintf(__format, argList);
    retval = outString.length();

    fprintf_ErrorBuffer += outString;

    va_end(argList);

    return retval;
}

void NTRIPThread::fprintf_error_buffered_flush(void)
{
    emit errorMessage("\n" + fprintf_ErrorBuffer.trimmed());
    fprintf_ErrorBuffer.clear();
}


int NTRIPThread::printf (const char *__format, ...)
{
    va_list argList;
    va_start(argList, __format);

    QString outString = QString::vasprintf(__format, argList);

    va_end(argList);

    emit infoMessage("\n" + outString.trimmed());

    return outString.length();
}

size_t NTRIPThread::fwrite(const void *ptr, size_t size, size_t nitems, FILE *stream)
{
    size_t retval;

    if ((stream == stdout) || (stream == stderr))
    {
        auto stringBuffer = std::make_unique<char[]>(size * nitems + 1);

        memcpy(&stringBuffer[0], ptr, size * nitems);

        stringBuffer[size * nitems] = 0;

        if (stream == stdout)
        {
            emit infoMessage("\n" + QString(&stringBuffer[0]).trimmed());
        }
        else if (stream == stderr)
        {
            emit errorMessage("\n" + QString(&stringBuffer[0]).trimmed());
        }

        retval = nitems;
    }
    else
    {
        retval = ::fwrite(ptr, size, nitems, stream);
    }

    return retval;
}


NTRIPThread::NTRIPThread(const QString& command)
{
    paramList = command.split(' ', QString::SkipEmptyParts);
    stop=false;
}

void NTRIPThread::run()
{
    const char* programName = "GNSS-Stylus";

    auto byteArrays = std::make_unique<QByteArray[]>(paramList.size() + 1);

    byteArrays[0] = programName;

    for (int i = 0; i < paramList.size(); i++)
    {
        byteArrays[i + 1] = paramList[i].toLocal8Bit().data();
    }

    auto paramStrings = std::make_unique<const char*[]>(paramList.size() + 1);

    for (int i = 0; i < (paramList.size() + 1); i++)
    {
        paramStrings[i] = byteArrays[i].data();
    }

    for (unsigned int i = 0; i < sizeof(optsInit) / sizeof(optsInit[0]); i++)
    {
        opts[i] = optsInit[i];
    }

    optind = 1;

    stop = false;

    main(paramList.size() + 1, &paramStrings[0]);

    printf("Execution of NTRIP thread ended.");

    emit threadEnded();
}

int NTRIPThread::rtcmDataWrite(const char *buffer, size_t size)
{
    QByteArray rtcmData;
    rtcmData.append(buffer, size);

    emit dataReceived(rtcmData);

    return size;
}

void NTRIPThread::requestTerminate(void)
{
    stop = true;
}

NTRIPThread::~NTRIPThread()
{
    stop = true;
    this->wait(5000);
    this->terminate();
    this->wait(5000);
}

void NTRIPThread::myperror(const char *s)
{
#ifdef WINDOWSVERSION
  fprintf(stderr, "%s: %d\n", s, WSAGetLastError());
#else
  perror(s);
#endif
}
