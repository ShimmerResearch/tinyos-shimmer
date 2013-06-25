#ifndef LOGGING_BROADCAST_H
#define LOGGING_BROADCAST_H 

#include "AM.h"

typedef nx_struct start_logging_msg {
    nx_uint32_t trial_id;
  nx_bool start_stop;
} start_logging_msg_t;

enum{
    AM_LOGGING_BROADCAST_MSG = 0x7F
};

#endif
