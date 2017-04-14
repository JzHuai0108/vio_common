#include "vio/timegrabber.h"
void testTimeGrabber(std::string timeFile)
{
    TimeGrabber tg(timeFile);
    double timestamp = tg.readTimestamp(0);
    std::cout << "timestamp for 0 "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(1799);
    std::cout << "timestamp for 1799 "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(1800);
    std::cout << "timestamp for 1800 "<< timestamp<< std::endl;
}
