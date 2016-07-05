#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include "ftdi.h"
#include <time.h>
#include <string.h>
#include <atomic>  
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include "params.h"
#include <chrono>
#include <thread>
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>  
#include "GridEyeReader.h"
#include "XMPPWrapper.h"
#include "OccupancyCounter.h"
using namespace cv;
using namespace std;

static int exitRequested = 0;
static int frameNumber = 0;
static int frameN = 0;
static int people_inside = 0;
int retval ;
int person_count_last = people_inside;
GridEyeReader sensorReader;
XMPPWrapper XMPPWrapper_;
OccupancyCounter OccupancyCounter_;
/*
*    SIGINT handler, so we can gracefully exit when the user hits ctrl-C.
*/
void print_time(){
    time_t     now;
    struct tm  ts;
    char       buf[80];
    time(&now);
    // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
    ts = *localtime(&now);
    strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
    //cout << buf <<" " << frameN << endl;
}
static void sigintHandler(int signum){
	print_time();
	sensorReader.~GridEyeReader();
    exitRequested = 1;
}
void read_from_file(string file_name,VideoWriter outputVideo){
    std::ifstream infile(file_name);
    int** frame = NULL;
    int frameN = 0;
    vector<Person> people;
    auto begin = chrono::high_resolution_clock::now();    
    while(true){
    	frameN++;
        
        auto end = chrono::high_resolution_clock::now();
        auto dur = end - begin;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
        cout << "ms passed is " << ms << endl;
    	frame = sensorReader.read_frame(&infile);
        begin = chrono::high_resolution_clock::now();
    	if (frame == NULL){
    		cout << "Something is wrong with input file " << input_file_name << endl;
    		return;
    	}
    	if (frameN < START_FRAME)
    		continue; 
    	else if (frameN > END_FRAME)
    		return;
    	Mat im = OccupancyCounter_.convert_to_Mat(frame);
        
    	Mat extended_im = OccupancyCounter_.resize_frame(im,frameN);
    	if(blobDetection)
    		OccupancyCounter_.blob_detect(extended_im);
    	if(contourDetection)
    		OccupancyCounter_.process_frame(extended_im,frameN,people,outputVideo);
    	//OccupancyCounter_.show_scaled(im);
       	if(ENABLE_XMPP_REPORTING){
       		if(people_inside != person_count_last){
       			cout << "sending info " << people_inside << person_count_last << endl;
		    	XMPPWrapper_.occupancyChange(people_inside, people_inside - person_count_last);
		    }
		}
		person_count_last = people_inside;
    }
}
void update_people_count_handler(int param){
	if(ENABLE_XMPP_REPORTING)
       XMPPWrapper_.update_people_count(param,people_inside);
}
void reset_occupancy_count(unsigned int initial_wait_time, atomic<bool>& timer_keeps_running){
	if(ENABLE_XMPP_REPORTING)
        XMPPWrapper_.reset_occupancy_count(initial_wait_time, timer_keeps_running,people_inside);
}
int compute_remaining_time_of_today(){

	struct timeval tv;
	struct tm *tm;

	int current_hour, current_min, current_sec;
	gettimeofday(&tv, NULL);

	if ((tm = localtime(&tv.tv_sec)) != NULL) {
		current_hour = tm->tm_hour; //24 hr format
		current_min = tm->tm_min;
		current_sec = tm->tm_sec;
	}
	else
	{
		cout << "Could not compute local time for timer based restart." << endl;
		return 0;
	}
	int remaining_time = (24 - current_hour - 1)*60*60 + (60 - current_min - 1)*60 + (60 - current_sec -1) + 1;  //in seconds, assuming we will reset at 12:00 AM
	return remaining_time;
}
int main(int argc, char **argv){
	VideoWriter outputVideo;
	Size S = Size(extended_resolution,extended_resolution);
	outputVideo.open("test.avi",  CV_FOURCC('M','J','P','G'), 10, S, true);
    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " << endl;
        return 0;
    }
    unsigned char buffer[1024];
    /* Initiate a timer for resetting people count at midnight */
    atomic<bool> timer_keeps_running {true} ;
	if(reset_count_at_midnight) {
		int initial_wait_time = compute_remaining_time_of_today();
		thread( reset_occupancy_count, initial_wait_time, std::ref(timer_keeps_running) ).detach() ; 
	}
	
	XMPPWrapper_.report_updated_people_count(-200, people_inside, people_inside - person_count_last); //logging + reporting initial count
    if(ENABLE_ACTUATION)
    {
    	signal(SIGUSR1, update_people_count_handler);
    }
    if(LIVE == false){
		cout << "reading from file" << endl;
		read_from_file(input_file_name,outputVideo);
		return 0;
	}
	//GridEyeReader sensorReader;
  	signal(SIGINT, sigintHandler);
 	map<int,int**> frames;
    print_time();
    vector<Person> people;
    sensorReader.Init();
    auto begin = chrono::high_resolution_clock::now();    
    while (!exitRequested)
    {
    	auto end = chrono::high_resolution_clock::now();
        auto dur = end - begin;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
<<<<<<< HEAD
        //cout << "ms passed is " << ms << endl;
        //usleep(std::max(0,(int)(100 - ms)) * 1000);
        if(ms > 100)
            cout << "time passed " << ms << endl;
        int f = sensorReader.ReadFrame(buffer);
        //cout << "sensere reader returns " << f <<"bytes" << endl;
        begin = chrono::high_resolution_clock::now();
       if( f > 130 )
       {
            
            //cout << "sensor reads  " << f << "bytes "<<endl;
=======
        if(ms > 100){
			cout << "ms passed is " << ms <<"at frame "<<frameN<< endl;
		}else{
			//usleep(std::max(0,(int)(100 - ms)) * 1000);
		}
        int f = sensorReader.ReadFrame(buffer);
        //cout << "sensere reader returns " << f <<"bytes" << endl;
        begin = chrono::high_resolution_clock::now();
        if (f<0){
            cout << "Something is wrong." <<f <<" bytes read" <<endl;
            usleep(1 * 1000000);
        }
        else if( f > 130 )
        {
            cout << "read from sensor " << f << endl;
>>>>>>> 4b6c7f33ae6c87453f0641f7af3ba8cc1f581446
            frames = sensorReader.interpret_data(buffer, f);
            for(map<int,int**>::iterator it=frames.begin();it!=frames.end();it++){
				frameN++;
				Mat im = OccupancyCounter_.convert_to_Mat(it->second);
				Mat extended_im = OccupancyCounter_.resize_frame(im,it->first);
				if(blobDetection)
					OccupancyCounter_.blob_detect(extended_im);
				if(contourDetection)
					OccupancyCounter_.process_frame(extended_im,it->first,people,outputVideo);
            }
            frames.clear();
        }
        if(ENABLE_XMPP_REPORTING)
            if(people_inside != person_count_last)
		      XMPPWrapper_.occupancyChange(people_inside, people_inside - person_count_last);
		person_count_last = people_inside;
    }
    signal(SIGINT, SIG_DFL);
    retval =  EXIT_SUCCESS;
    
    sensorReader.~GridEyeReader();
    return retval;
}
