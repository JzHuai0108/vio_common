
#include "vio/timegrabber.h"
#include "vio/eigen_utils.h"

//#include <gpstk-2.5.linux.x86_64/SystemTime.hpp>
//#include <gpstk-2.5.linux.x86_64/CommonTime.hpp>
//#include <gpstk-2.5.linux.x86_64/CivilTime.hpp>
//#include <gpstk-2.5.linux.x86_64/YDSTime.hpp>
//#include <gpstk-2.5.linux.x86_64/GPSWeekSecond.hpp>
//#include <gpstk-2.5.linux.x86_64/Position.hpp>

//using namespace gpstk;
using namespace std;

namespace vio{

TimeGrabber::TimeGrabber():
        last_line_index(-1), last_line_time(-1){
    }
    TimeGrabber::TimeGrabber(const string time_file_name):time_file(time_file_name),
        time_stream(time_file_name.c_str()),
        last_line_index(-1), last_line_time(-1){
        if(!time_stream.is_open())
        {
             std::cout << "Error opening timestamp file!"<<endl;
        }
    }
   
    TimeGrabber::~TimeGrabber(){
        time_stream.close();
    }
    bool TimeGrabber::init(const string time_file_name){
        time_file=time_file_name;
        time_stream.open(time_file_name.c_str());
        last_line_index=-1;
        last_line_time=-1;
        if(!time_stream.is_open())
        {
                 std::cout << "Error opening timestamp file!"<<endl;
                 return false;
        }
        return true;
    }
    // this reading function only works for KITTI timestamp files
    double TimeGrabber::readTimestamp(int line_number)
    {
        string tempStr;
        double precursor(-1);
        if(last_line_index>line_number){
            cerr<<"Retracing to read timestamps is unsupported!"<<endl;
            return -1;
        }
        if(last_line_index==line_number)
            return last_line_time;
        while(last_line_index<line_number){
            time_stream>>precursor;
            if(time_stream.fail())
                break;
            getline(time_stream, tempStr);       //remove the remaining part, this works even when it is empty
            ++last_line_index;
        }
        if(last_line_index<line_number)
        {
            cerr<<"Failed to find this line in time file!"<<endl;
            return -1;
        }
        last_line_time=precursor;
        return last_line_time;
    }
    //extract time and return left image filename, tailored for Malaga urban dataset
    // in this function frame_number is 0 for the first two lines in the /*_IMAGE.txt file
double TimeGrabber::extractTimestamp(int frame_number)
{
    string tempStr;
    double precursor(-1);
    if(last_line_index>frame_number){
        cerr<<"Retracing to read timestamps is unsupported!"<<endl;
        return -1;
    }
    if(last_line_index==frame_number)
    {
        return last_line_time;
    }
    while(last_line_index<frame_number){
        getline(time_stream, tempStr);
        if(time_stream.fail())
            break;
        last_left_image_name=tempStr;
        getline(time_stream, tempStr);       //also read in the right image name
        precursor=atof(tempStr.substr(12, 17).c_str());
        ++last_line_index;
    }
    if(last_line_index<frame_number)
    {
        cerr<<"Failed to find this line in time file!"<<endl;
        return -1;
    }
    last_line_time=precursor;
    return last_line_time;
}
}
