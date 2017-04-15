#include "vio/timegrabber.h"
void testTimeGrabber(std::string timeFile)
{
    vio::TimeGrabber tg(timeFile);
    double timestamp = tg.readTimestamp(0);
    std::cout << "expected timestamp for 0 frame 0 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(1799);
    std::cout << "expected timestamp for 1799 frame 1.864921e+02 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(1800);
    std::cout << "expected timestamp for 1800 frame 1.865959e+02 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(4540);
    std::cout << "expected timestamp for 4540 frame 4.705816e+02 and timestamp read from file "<< timestamp<< std::endl;
    timestamp = tg.readTimestamp(4541);
    std::cout << "expected timestamp for 4541 frame NULL and timestamp read from file "<< timestamp<< std::endl;

}
int main(){
	testTimeGrabber("../test/times_kitti_seq00.txt");
	return 0;
}
