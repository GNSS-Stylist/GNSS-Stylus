// This is only a little "glue"-file to make it possible to compile this on
// different OSses. Not too pretty, but should work.
// I threw socket version out of this, since most probably it won't be needed.

// Not currently (22-nov-2020) tested on Mac or linux!

#if defined(_WIN32)
//#include "win32/net_socket.cpp"
#include "win32/net_serial.cpp"
#include "win32/timer.cpp"
#elif defined(_MACOS)
//#include "macOS/net_socket.cpp"
#include "macOS/net_serial.cpp"
#include "macOS/timer.cpp"
#elif defined(__GNUC__)
//#include "linux/net_socket.cpp"
#include "linux/net_serial.cpp"
#include "linux/timer.cpp"
#else
#error no serial/timer implemention found for this platform.
#endif

