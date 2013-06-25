#ifndef TIME_BROADCAST_H
#define TIME_BROADCAST_H 

#include "AM.h"

typedef nx_struct time_broadcast_msg {
    nx_uint32_t trial_id;
  nx_uint32_t timestamp;
} time_broadcast_msg_t;

enum{
    AM_TIME_BROADCAST_MSG = 0x7F
};

#endif
