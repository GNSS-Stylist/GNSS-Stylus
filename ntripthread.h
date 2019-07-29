/*
    ntripthread.h (part of GNSS-Stylus)
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

#ifndef NTRIPTHREAD_H
#define NTRIPTHREAD_H

#include <QObject>
#include <QThread>
#include <getopt.h>

#define MAXNUMOFCOMMANDLINEOPTIONS (20)

class NTRIPThread : public QThread
{
    Q_OBJECT
private:

    struct Args
    {
      const char *server;
      const char *port;
      const char *user;
      const char *proxyhost;
      const char *proxyport;
      const char *password;
      const char *nmea;
      const char *data;
      int         bitrate;
      int         mode;

      int         udpport;
      int         initudp;
      const char *serlogfile;
    };

    enum MODE { HTTP = 1, RTSP = 2, NTRIP1 = 3, AUTO = 4, UDP = 5, END };

    const char *encodeurl(const char *req);
    const char *geturl(const char *url, struct Args *args);
    int getargs(const int argc, const char* const* argv, struct Args* const args);
    int encode(char *buf, int size, const char *user, const char *pwd);
    int main(const int argc, const char* const* const argv);
    int rtcmDataWrite(const char *buffer, size_t size);
    void myperror(const char *s);
    int fprintf (FILE *__stream, const char *__format, ...);
    int printf (const char *__format, ...);
    size_t fwrite(const void *ptr, size_t size, size_t nitems, FILE *stream);

    bool stop;
    QStringList paramList;  //!< Command line parameters
    struct option opts[MAXNUMOFCOMMANDLINEOPTIONS];
    char encodeUrlBuffer[128];
    char getUrlBuffer[1000];
    char *getUrlBufferPtr;
    char *getUrlBufferEndPtr;

public:
    /**
     * @brief Constructor
     * @param command "Command line" for the NTRIP client
     */
    NTRIPThread(const QString& command);
    void requestTerminate(void);    //!< Requests thread to terminate
    ~NTRIPThread() override;

    void run() override;            //!< Thread code

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message
    int dataReceived(const QByteArray&);   //!< Signal that is emitted when data is received.

    void threadEnded(void);
};

#endif // NTRIPTHREAD_H
