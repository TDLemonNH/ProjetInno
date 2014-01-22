#include "utils.hpp"

uint64_t getTimeStamp() 
{
    struct timeval tv;
    gettimeofday(&tv,0);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}
