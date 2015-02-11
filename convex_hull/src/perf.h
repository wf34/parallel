
#ifndef PERF_H__
#define PERF_H__

#include <string>

extern "C" {
#include "mcbsp.h"
}

#include "logging.h"

class Perf {
public:
    Perf (const std::string& section_name)
        : time_start_ (bsp_time ())
        , section_name_ (section_name)
    {}

    ~Perf ()
    {   double time_end = bsp_time ();
        LOG_LEAD (section_name_ << " " << (time_end - time_start_));
    }
private:
    double time_start_;
    std::string section_name_;
};

#endif //PERF_H__
