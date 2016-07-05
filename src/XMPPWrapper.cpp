/**@file GridEyeReader.cpp
 *
 * @date June 30, 2016
 * @author Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 */

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
#include "XMPPWrapper.h"
using namespace std;
using namespace cv;

std::string XMPPWrapper::getParentInfo(){
    if(!LIVE)
        return "_BOSCH_OFFICE_MAIN";
    else {
        return "_BOSCH_OFFICE_MAIN"; //for the unknown nodes
    }
}
void XMPPWrapper::report_updated_people_count(int indicator_count, int new_count, int offset) {
    //if(log_occ_estimation) {
    //  const char *timestamp = create_timestamp();
    //  log_file << timestamp << " " << indicator_count << "\n"; //dataset will indicate here is where the FORK started running or had a forced restart
    //  log_file << timestamp << " " << new_count << "\n"; //reporting the initial count 
    //  delete[] timestamp;
    //}
    #ifdef ENABLE_XMPP_REPORTING
        if(enable_publishing_indicator_count)
            xmpp->occupancyChange(indicator_count, 0);   //dataset will indicate here is where the FORK started running or had a forced restart or received a new count over the network
        // offset is 0
        xmpp->occupancyChange(new_count, offset); //reporting the initial count
    #endif
}
void XMPPWrapper::update_people_count(int param,int people_inside){
    
    if(param == SIGUSR1)
    {
        int new_count = xmpp->get_actuation_count();
        cout <<"In FORK_RT, updated count: " <<  new_count << endl;
        actuation_request_count++;
        if((actuation_request_count > ignore_actuation_request_count) && (new_count > -450)){
            int offset = new_count - people_inside;
            people_inside = new_count;
            report_updated_people_count(-300, new_count, offset);  //-300 means it was reset over the network
        }
    }
}
void XMPPWrapper::reset_occupancy_count( unsigned int initial_wait_time, atomic<bool>& timer_keeps_running, int people_inside){

    // First, wait until midnight
    const auto initial_wait = chrono::seconds(initial_wait_time);
    this_thread::sleep_for(initial_wait) ;

    //Then at midnight, reset count and wait 1 more day
    const auto next_interval = chrono::seconds(24*60*60);//delay for the next day, in seconds
    while( timer_keeps_running )
    {
        int offset = -people_inside;
        people_inside = 0; //We may need to use mutex in the future. Assuming that at 12:00 AM we will not run into a race condition.
        report_updated_people_count(-200, people_inside, offset);

        //wait until the next 24 hours     
        this_thread::sleep_for(next_interval) ;
    }
}
XMPPWrapper::XMPPWrapper(){

    this->xmpp = new XMPPInterface(XMPP_JID, XMPP_PASS, XMPP_NODE, XMPP_PARENT_NODE, enable_distributed_coordination, ENABLE_ACTUATION, XMPP_VERBOSITY); 
    this-> actuation_request_count = 0;
    this-> enable_distributed_coordination= true;
    this-> ignore_actuation_request_count = 3;
}
void XMPPWrapper::occupancyChange(int people_inside, int change){
    this->xmpp->occupancyChange(people_inside, change);
}
     
    
	