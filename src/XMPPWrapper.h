/**@file GridEyeReader.h
 *
 * @date June 30, 2016
 * @author Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 */

#ifndef XMPPWrapper_H_
#define XMPPWrapper_H_
using namespace std;
#include "XMPPInterface.h"
#define XMPP_JID  "fork1@sensor.andrew.cmu.edu"
#define XMPP_PASS "boschtop75"
#define XMPP_NODE "FORK018003251847" //getSerialNo()).c_str() )
#define XMPP_PARENT_NODE  (("FORK" + getParentInfo()).c_str())  //This is the node ID, where the offset of occupancy change will be published
#define XMPP_VERBOSITY 3
class XMPPWrapper{
public:
	XMPPWrapper();
	void reset_occupancy_count( unsigned int initial_wait_time, atomic<bool>& timer_keeps_running,int people_inside);
	void report_updated_people_count(int indicator_count, int new_count, int offset);
	void update_people_count(int param,int people_inside);
	void occupancyChange(int people_inside, int change);
	//virtual ~XMPPWrapper();
private:
	XMPPInterface *xmpp;
	int actuation_request_count;
	bool enable_distributed_coordination;
	int ignore_actuation_request_count;
	std::string getParentInfo();
	
	
};

#endif /* XMPPWrapper_H_ */
