
#ifndef _LOGGER_H__
#define _LOGGER_H__

#include <iostream>

extern "C" {
#include "mcbsp.h"
}

class Logger {
public:
    enum LoggingMode {
        UNDEF = 0,
        DEV_LEAD_ONLY = 1,
        DEV_ALL = 2,
        SUPPRESS = 4
    };

    Logger ()
        : mode_ (DEV_LEAD_ONLY | DEV_ALL)
    {
    }

    std::ostream& get_stream()
    {   return std::cout;
    }

    bool is_allowed (LoggingMode mode)
    {   return mode & mode_;
    }

private:
    int mode_;
};

#ifndef NO_LOG
#define LOG_LEAD(M)   do { if (Logger().is_allowed(Logger::DEV_LEAD_ONLY))   (Logger().get_stream() << M << "\n"); } while (false)

#define LOG_LEAD(M)    do {   \
                              if (Logger ().is_allowed (Logger::DEV_LEAD_ONLY) && \
                                  0 == bsp_pid ()) \
                              { Logger ().get_stream () << M << std::endl << std::flush; \
                              } \
                          } while (false)

#define LOG_ALL(M)    do {   \
                             if (Logger ().is_allowed (Logger::DEV_ALL)) \
                             { Logger ().get_stream () << "[P" << bsp_pid () << "] " << M << std::endl << std::flush; \
                             } \
                         } while (false)
#else
#define LOG_ERROR(M)
#define LOG_INFO(M)
#define LOG_WARNING(M)
#endif

#endif // _LOGGER_H__
