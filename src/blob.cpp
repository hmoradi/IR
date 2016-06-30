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
using namespace cv;
using namespace std;
char temp;
#define pause() (cin >> temp)

//#define ENABLE_XMPP_REPORTING

#ifdef ENABLE_XMPP_REPORTING
#include "XMPPInterface.h"
/* XMPP Settings */
#define XMPP_JID  "fork1@sensor.andrew.cmu.edu"
#define XMPP_PASS "boschtop75"
#define XMPP_NODE ("FORK018003251847") //getSerialNo()).c_str() )
#define XMPP_PARENT_NODE ( ("FORK" + getParentInfo()).c_str() ) //This is the node ID, where the offset of occupancy change will be published
#define XMPP_VERBOSITY 3
#endif
#ifdef ENABLE_XMPP_REPORTING
	XMPPInterface *xmpp;
#endif

int actuation_request_count = 0;
struct ftdi_context *ftdi;
unsigned char residue[1024] ;
static int exitRequested = 0;
static int frameNumber = 0;
static int frameN = 0;
static int people_inside = 0;
enum status {UNKNOWN, ENTERED,  LEFT};
enum direction {UNKNOWNDIR, LtoR , RtoL};
bool enable_distributed_coordination= true;
int ignore_actuation_request_count = 3;
bool enable_actuation = true;
int person_count_last = people_inside;
struct Body {
	int ID_;
	Rect rect_;
	int T ;
	float confidence;
};
struct bodyCompare {
	bool operator() (const Body& lhs, const Body& rhs) const
	{
		return lhs.T < rhs.T;
	}
	
};

struct Person {
	map<int,Body> trajectory;
	status status_;
	direction direction_;
	int ID_;
	int last_location;
	int last_frame_seen;
	Body last_body_seen;
	int trajectory_size;
};

std::string getParentInfo(){
	if(!LIVE)
		return "_BOSCH_OFFICE_MAIN";
	else {
		return "_BOSCH_OFFICE_MAIN"; //for the unknown nodes
	}
}
void report_updated_people_count(int indicator_count, int new_count, int offset) //indicator_count: -200 (forced reset/restart), -300(actuation over the network)
// offset indicates (new count - old count). It can be negative.
{
	//if(log_occ_estimation) {
	//	const char *timestamp = create_timestamp();
	//	log_file << timestamp << " " << indicator_count << "\n"; //dataset will indicate here is where the FORK started running or had a forced restart
	//	log_file << timestamp << " " << new_count << "\n"; //reporting the initial count 
	//	delete[] timestamp;
	//}
	#ifdef ENABLE_XMPP_REPORTING
		if(enable_publishing_indicator_count)
	    	xmpp->occupancyChange(indicator_count, 0);   //dataset will indicate here is where the FORK started running or had a forced restart or received a new count over the network
	    // offset is 0
	    xmpp->occupancyChange(new_count, offset); //reporting the initial count
	#endif
}
void update_people_count_handler(int param){
	#ifdef ENABLE_XMPP_REPORTING
		if(param == SIGUSR1)
		{
			int new_count = xmpp->get_actuation_count();
			cout <<"In FORK_RT, updated count: " <<  new_count << endl;
			
			actuation_request_count++;
			
		//	if(new_count == -500) //start data collection
		//	{
		//		store_depth_images = true;
		//		process_images = false;
		//	}
			
		//	if(new_count == -600)	//stop data collection
		//	{
		//		store_depth_images = false;
		//		process_images = true;
		//	}
			
			if((actuation_request_count > ignore_actuation_request_count) && (new_count > -450)){
				int offset = new_count - people_inside;
				people_inside = new_count;
				report_updated_people_count(-300, new_count, offset);  //-300 means it was reset over the network
			}
		}
	#endif
}
void reset_occupancy_count( unsigned int initial_wait_time, atomic<bool>& timer_keeps_running)
{

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

float calc_match_weight (Person person_ , Body body){
	map<int,Body>::reverse_iterator last_location = person_.trajectory.rbegin();
	double distance =  norm(last_location->second.rect_.tl()-body.rect_.tl());
	float size = abs(last_location->second.rect_.area() - body.rect_.area()) / last_location->second.rect_.area(); 
	float T = abs(last_location->second.T - body.T) / last_location->second.T;
	float dir = 0;
	if(person_.direction_ == LtoR){
		if (last_location->second.rect_.x + (last_location->second.rect_.width / 2) <= body.rect_.x + (body.rect_.width / 2))
			dir = -1;
		else 
			dir = 1;
	}
	else if(person_.direction_ == RtoL){
		if (last_location->second.rect_.x + (last_location->second.rect_.width / 2) >= body.rect_.x + (body.rect_.width / 2))
			dir = -1;
		else 
			dir = 1;
	}
	return   7.0 / max((distance + 3*size + 2*T+dir),0.001) ;
}
float avg_(int start, int end, vector<int> input_){
	//cout << "averageing vector with size "<< input_.size() << "starting from " << start << "ending at" << end << endl;
	float sum = 0;
	int count = 0;
	for(int i=start;i<end;i++){
		sum+= input_.at(i);
		count += 1;
	}
	return sum / max(count , 1);
}
void update_people_status(vector<Person>& people, int frameN){
	int left_person = 0;
	for(Person& person_:people){
		if(person_.status_ == LEFT){
			left_person++;
			continue;
		}
		//cout << "at frame "<<frameN << "updateding status for " << people.size() - left_person<< "people" << endl;
		if(person_.status_ == UNKNOWN){
			person_.status_ = ENTERED;
		}
		else if (person_.status_ == ENTERED){
			if(frameN - person_.last_frame_seen >= DETECTION_DELAY){ //Old Person
				person_.status_= LEFT;
				vector<int> locations ;
				vector<int> sorted_locations;
			
				float frame_confidence = 0;
				float frame_T = 0;
				int min_x = extended_resolution;
				int min_t = 0;
				int max_x = 0;
				int max_t = 0;
				for(auto item :person_.trajectory){
					locations.push_back(item.second.rect_.x+ (item.second.rect_.width / 2));
					sorted_locations.push_back(item.second.rect_.x+ (item.second.rect_.width / 2));
				
					frame_confidence += item.second.confidence;
					frame_T += item.second.T;
					min_t = min(min_t,item.first);
					max_t = max(max_t,item.first);
					//cout << "person id " <<person_.ID_ << " trajectory frame "<< item.first << endl;
				}
				sort(sorted_locations.begin(),sorted_locations.end());
				max_x = sorted_locations.at(sorted_locations.size()-1);
				min_x = sorted_locations.at(0);
				float speed = (float)(max_x - min_x)/(float)(max(max_t-min_t,1));
				float avg_confidence = frame_confidence /(float)locations.size();
				float avg_T = frame_T / (float)(locations.size());
				if(locations.size() >= TRAJECTORY_LENGTH and speed != 0){
					cout << "avg confidence is " << avg_confidence <<" locatoin size "<<locations.size() << " spped "<< speed << endl;
					
					float first_half_average = avg_(0, locations.size()/2,locations);    
					float second_half_average = avg_(((locations.size()/2)) , locations.size(),locations);
					if(first_half_average < second_half_average){
						cout << "&&&&&&&&&&&&&&&&&&&&left to right" << endl;
						if(OPOSITE_DIRECTION)
							people_inside ++;
						else
							people_inside --;
					}
					else{
						cout << "$$$$$$$$$$$$$$$$$$$right to left " << endl;
						if(OPOSITE_DIRECTION)
							people_inside --;
						else
							people_inside ++;
					}
					cout << "person temperature is " << avg_T << endl;
					cout << "frame " << frameN << endl;
					//pause();
				}else if(speed > 10 and speed < 120){
					if(person_.direction_ == LtoR)
						cout << "speed move from left to right"<< endl;
					else if(person_.direction_ == RtoL)
						cout << "speed move from right to left" << endl;
					else
						cout << "speed move without direction detected "<< endl;

				}else if(avg_confidence > .8){
					if(person_.direction_ == LtoR)
						cout << "confidence move from left to right"<< endl;
					else if(person_.direction_ == RtoL)
						cout << "confidence move from right to left" << endl;
					else
						cout << "confidence move without direction detected "<< endl;					
				}
			}	
		}
		else {
			if (person_.last_body_seen.rect_.x < person_.last_location and (person_.direction_ == RtoL or person_.direction_ == UNKNOWNDIR) ){
				person_.direction_ = RtoL;
			}
			else if(person_.last_body_seen.rect_.x < person_.last_location and (person_.direction_ == LtoR or person_.direction_ == UNKNOWNDIR)){
				person_.direction_ = LtoR;
			}else{
				person_.direction_ = UNKNOWNDIR;
			}
			person_.last_location = person_.last_body_seen.rect_.x;
		}	
	}	
}
void update_people_info2(map<int,Body> body_confidence,vector<Person>& people, int frameN){
	//cout << "updating people info at frame "<< frameN << endl;
	int num_of_bodies = 0;
	int body_ID = 0;
	int person_ID = 0;
	vector<Body> bodies ;
	vector<Person> persons;
	for(auto body_item : body_confidence){
		num_of_bodies ++;
		body_item.second.ID_ = body_ID;
		body_ID++;
		//cout << "body confidence is " << body_item.second.confidence << endl;
		bodies.push_back(body_item.second);
	}
	int num_of_persons =0;
	for(Person& person:people){
		if(person.status_ == LEFT)
			continue;
		person.ID_ = person_ID;
		persons.push_back(person);
		person_ID++;
		num_of_persons++;
	}
	

	people.clear();
	float** pairs_matrix = new float*[num_of_bodies];
	for(int i=0;i < num_of_bodies;i++)
		pairs_matrix[i] = new float[num_of_persons];
	
	//cout << "num of bodies " << num_of_bodies << " num of persons " << num_of_persons << endl;
	float match_weight = 0;
	for(Body& body_:bodies){
		for(Person& person_ :persons){
			match_weight = 0;
			map<int,Body>::iterator peopleIT = person_.trajectory.end();
			if(person_.status_ != LEFT){
				match_weight = calc_match_weight(person_,body_);
				//cout << "match weight for " << person_.ID_ << " and " << body_.ID_ << "is "<< match_weight <<endl;
			}
			pairs_matrix[body_.ID_][person_.ID_] = match_weight;
		}
	}
	int num_of_matched_pairs = 0;
	map<int,int> matched ;
	while(true){
		int max_row = -1;
		int max_col = -1;
		float max_value = 0;
		for (int i=0;i<num_of_bodies;i++){
			for(int j=0;j<num_of_persons;j++){
				if(pairs_matrix[i][j] > max_value){
					max_value = pairs_matrix[i][j];
					max_row = i;
					max_col = j;
				}
			}
		}
		if(max_row != -1 and max_col != -1){
			persons.at(max_col).trajectory[frameN] = bodies.at(max_row);
			persons.at(max_col).last_frame_seen = frameN;
			persons.at(max_col).last_body_seen = bodies.at(max_row);
			people.push_back(persons.at(max_col));
			matched[max_row] = max_col;
			//cout << "person " << max_col << "matched with body " << max_row << endl;
			for(int k=0;k<num_of_persons;k++)
				pairs_matrix[max_row][k] = -1;
			for(int k=0;k<num_of_bodies;k++)
				pairs_matrix[k][max_col] = -1;
			num_of_matched_pairs ++;	
		}else{
			break;
		}
		
	}
	for(Body& body_ : bodies){
		if(matched.find(body_.ID_) == matched.end()){
			Person new_one ;
			new_one.trajectory[frameN]= body_;
			new_one.status_ = UNKNOWN;
			new_one.direction_ = UNKNOWNDIR;
			new_one.ID_ = person_ID;
			person_ID++;
			people.push_back(new_one);
			//cout << "new person created " << endl;
		}
	}
	bool found = false;
	for(Person& person_:persons){
		found = false;
		for(auto item:matched){
			if(item.second == person_.ID_){
				found = true;
			}
		}
		if(!found){
			people.push_back(person_);
		}
	}
	//if(num_of_bodies> 1)
	//	pause();
}

//prints system time 
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
/*
 * sigintHandler --
 *
 *    SIGINT handler, so we can gracefully exit when the user hits ctrl-C.
 */
static void sigintHandler(int signum){
	print_time();
	unsigned char buf2[1];
	buf2[0] = '~';
	int f = ftdi_write_data(ftdi, buf2, 1); // Asking PIC24F04KA200 microcontroller to stop sending data
	if( f < 0)
		printf("Error in writing data: ~ .\n");
    exitRequested = 1;
}

//Extracts serial packets from raw data received from serial port
int find_packet_head(unsigned char *buf, int index, int len){
    //Each serail packet starts with ***
    int count = 0;
    int i=0;
    for(i=index; i< len; i++){
        if(buf[i]=='*'){
            count++;
            if(count ==3){
                return i -2;
            }
        }else{
            count = 0;
        }
    }
    return i;
}

//Copey source array to destinatino array starting from start_index for lenght of len
void array_copy(unsigned char* dest, unsigned char* source,int start_index, int len){
    
    for (int i=0;i<len;i++){
        dest[i] = source[start_index+i];
    }
}

//If there is not complete packet, buffer it at residue buffer 
void add_to_residue(unsigned char* buf, int index){
    int last_data_index =0;
    while(residue[last_data_index]!= 0){
        last_data_index++;
    }
    int i=0;
    while(i < index){
        residue[last_data_index+i] = buf[index];
        i++;
    }
}

//Prints content of serial packet received from FTDI module
int** print_packet(unsigned char* buf,int index){
    frameNumber++;
	int** frame = new int*[8];
    for (int i=0;i<8;i++)
        frame[i] = new int[8];
	//long int epoch= print_time();    
    int thermistor_data = ((int)buf[index + 4]<<8) | buf[index +3]; //Next 2 characters [index: 3, 4] are thermistor bytes	
    int k = index + 5; 
    for(int i = 0; i < 8; i++) {
        for(int j = 0; j < 8; j++) {
            int high_byte = buf[k+1];
            unsigned char low_byte = buf[k];
            int temperature_reading = (high_byte << 8) | low_byte;
            //IR_array_data[i][j] = temperature_reading;
            frame[i][j] = min(max( temperature_reading / 4 , 0),40);
             
            k+=2;
        }
    }   
    return frame;
}

//interpret raw data
map<int,int**> interpret_data(unsigned char *buf, int len){
	int** frame;
	map<int,int**> frames;
	int index = find_packet_head(buf,0,len);
	//cout << "1 " << index << endl;
	if(index != 0){
        add_to_residue(buf,index);
        frame = print_packet(residue,0);
        frames[frameNumber] = frame;
        memset(residue,0,sizeof residue);
    }else{
        if(residue[0] != 0){
           frame = print_packet(residue,0);
           frames[frameNumber] = frame;
           memset(residue,0,sizeof residue);
        }
    }
    while(index < len){
        if(len - index < 131 ){
            array_copy(residue,buf,index ,len-index);
            return frames;
        }
        frame = print_packet(buf,index);
        frames[frameNumber] = frame;
        index+= PacketLength;
        index = find_packet_head(buf,index,len);
        //cout << "2 " << index << endl; 
    }
    return frames;	
}

int** read_frame(ifstream* infile){
    int** frame = new int*[8];
    for (int i=0;i<8;i++)
        frame[i] = new int[8];
    int epoch,frameN,Ta;
    int index = 0;
    
    while(*infile >> epoch >> frameN >> Ta >> frame[index][0] >> frame[index][1] >> frame[index][2] >> frame[index][3] >> frame[index][4] >> frame[index][5] >> frame[index][6] >> frame[index][7]){
    	
    	index ++;
    	if (index >=8){
    		return frame;
    	}
    }
    return NULL;
}

Mat convert_to_Mat(int** frame){
   try{
   	Mat image = Mat::zeros(8,8,CV_8UC1);
   for(int i=0;i<8;i++)
      for (int j=0;j<8;j++)
      	image.at<uchar>(i,j) = frame[i][j];	
   		return image;
   }catch(Exception &e){
   		cerr << e.what() << endl;
   } 
}

Mat resize_frame(Mat im, int frameN){
	Size size(extended_resolution,extended_resolution);
	Mat result;
	resize(im,result,size,INTER_CUBIC);
	double min, max;
	Point minL,maxL;
	minMaxLoc(result,&min,&max, &minL,&maxL);
	//cout << frameN << " " << min << " " << max << endl; 
	return result;
}

void show_image(Mat im,VideoWriter outputVideo){
   
   frameN ++;
   if(!showImage)
   	   return;
   Mat im_color;
   Mat display;
   double min,max;
   minMaxIdx(im,&min,&max);
   Mat scaled_image;
   Point textOrg(110,25);
   int fontFace = CV_FONT_HERSHEY_TRIPLEX;
   double fontScale = 1;
   int thickness = 1; 
   if(scaleImage){
   	    convertScaleAbs(im,scaled_image,255 / 26);
   	    applyColorMap(scaled_image,im_color,COLORMAP_JET);
   	    flip(im_color,display,0);
   	    putText(display, to_string(people_inside), textOrg, fontFace,fontScale,Scalar::all(40),thickness,8  );
   	    namedWindow("key points- scaled",CV_WINDOW_AUTOSIZE);

   	    if(SAVE_VIDEO)
   	    	outputVideo.write(display);
   	    imshow("key points- scaled",display);
   	    
   }else{
        applyColorMap(im,im_color,COLORMAP_JET);
		namedWindow("key points - raw image",CV_WINDOW_AUTOSIZE);
   	    imshow("key points - raw image",im_color);
   }
   waitKey(WAIT_TIME);
}
void show_scaled (Mat im){
	Mat scaled= Mat::zeros(256,256,CV_8UC1);
	for(int i=0;i<256;i++){
		for(int j=0;j<256;j++){
			scaled.at<uchar>(i,j)=im.at<uchar>(i/32,j/32);
		}
	}
	Mat scaled_image;
	Mat im_color;
	Mat display;
	convertScaleAbs(scaled,scaled_image,255 / 26);
    applyColorMap(scaled_image,im_color,COLORMAP_JET);
    flip(im_color,display,0);
	namedWindow("raw input",CV_WINDOW_AUTOSIZE);
	imshow("raw input",display);

}
void blob_detect(Mat im){
    SimpleBlobDetector::Params params;
 	
    //Change thresholds
    params.minThreshold = blobMinThreshold;
    //params.maxThreshold = blobMaxThreshold;
    params.thresholdStep = blobThresholdStep;
    //params.minDistBetweenBlobs =minDistBetweenBlobs;
    
    // Filter by Area.
    params.filterByArea = blobFilterByArea;
    params.minArea = blobMinArea;
    params.maxArea = blobMaxArea;
    
    // Filter by Circularity
    params.filterByCircularity = blobFilterByCircularity;
    params.minCircularity = blobMinCircularity;
    params.maxCircularity = blobMaxCircularity;
     
    // Filter by Convexity
    params.filterByConvexity = blobFilterByConvexity;
    params.minConvexity = blobMinConvexity;
    params.maxConvexity = blobMaxConvexity;
    
    // Filter by Inertia
    params.filterByInertia = blobFilterByInertia;
    params.minInertiaRatio = blobMinInertiaRatio;
    params.maxInertiaRatio = blobMaxInertiaRatio;
    
    //Filter by Color 
    params.filterByColor = blobFilterByColor;
    //params.blobColor = 2;

    #if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
      SimpleBlobDetector detector(params);
    #else
      Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    #endif
 
    // Detect blobs.
    std::vector<KeyPoint> keypoints;
    detector.detect( im, keypoints);
    //cout << keypoints.size() << endl;
    if(keypoints.size() >= 0){
        for (int i=0;i<keypoints.size();i++){
            cout << keypoints[i].pt.x << " " << keypoints[i].pt.y << " " << keypoints[i].size << endl; 
        }

        Mat im_with_key_points;
        //drawKeypionts(im,keypoints,im_with_key_points,Scalar(0,0,255),DrawMatchesFlags::DRAW_RTCH_KEYPOINTS);
        drawKeypoints(im,keypoints,im_with_key_points,Scalar(0,0,15),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //show_image(im_with_key_points);
    }
}

void find_body(Rect candidate_rect, map<int,vector<Rect>> contour_map, int thresh , int level,map<int,vector<Rect>>& body_map){
  
  int maxx = (RECTMAXAREA*extended_resolution*extended_resolution)/100;
  int minx = (RECTMINAREA*extended_resolution*extended_resolution)/1000;
  minx = 5;
  if (candidate_rect.area() >= maxx or candidate_rect.area() < minx)
        return;
  vector<Rect> rects = contour_map[thresh-1];
  //cout << "find body is called at thresh " << thresh << "and there are " << rects.size() << "rects in lower layer"<<endl;
  //cout << "rect " << candidate_rect.x << " " << candidate_rect.area() << "at thresh " << thresh << "is trying to match with " << rects.size() <<endl; 
  for (Rect rect : rects  ){
    if (rect.area() >= maxx or rect.area() < minx)
        continue;
    Rect intersect = candidate_rect & rect;
    float percentage = (float)intersect.area() / (float)min(candidate_rect.area(),rect.area());
    //cout << "cand rect x " << candidate_rect.x << "rect is " << rect.x << endl;
    if (percentage >= 0.7){
      //cout << "matched with rect " << rect.x << " " << rect.area() << endl;
      if(level == 1){
        bool inserted = false;
        for(Rect rect_temp : body_map[thresh-1]){
        	if (rect_temp == rect)
        		inserted = true;
        }
        if(!inserted)
        	body_map[thresh-1].push_back(rect);
      }
      else{
        //cout << "rect " << rect.x << "sent to find_body at thresh " << thresh -1 << endl;
        find_body(rect,contour_map,thresh -1,level + 1,body_map);  
      } 
    }
  }
  rects.clear();
  return;
}

map<int,Body> calc_confidence(map<int,map<int,vector<Rect>>> body_parts, vector<Rect> bodies){
  map<int,Body> body_confidence;
  int temperature;
  for(map<int,map<int,vector<Rect>>>::iterator it = body_parts.begin();it != body_parts.end();it++){
    int body_location = it->first;
    Rect body ;
    for(Rect rect:bodies){
      if ((rect.x + rect.width /2) == body_location){
        body = rect;
        break;
      }
    }
    float confidence = 0;
    map<int,vector<Rect>> parts = it->second;
    for(map<int,vector<Rect>>::iterator inner_it = parts.begin();inner_it!= parts.end();inner_it++){
        Rect result;
        temperature = inner_it->first;
        for(Rect rect:inner_it->second){
          result |= rect;
        }
        Rect coverage = result & body;
        float coverage_conf = (float)coverage.area() / (float)(body| coverage).area();
        float temp_conf = 1.0 -((float)abs(25 - temperature) / 10.0);
        float height_conf = (float)result.height / (float)extended_resolution;
        float width_conf = (float)result.width / (float)extended_resolution;
        float size_conf = (float)(body.area()) / (float)(extended_resolution*extended_resolution);
        confidence += (float)(7*temp_conf + coverage_conf + height_conf+size_conf)/10.0 ;
    }
    //cout << "new confidence is "  << confidence << endl;
    if(confidence >= BODY_THRESHOLD){
		Body body_ ;
		body_.rect_ = body;
		body_.T = temperature;
		body_.confidence = confidence;
      	body_confidence[body_location] = body_;
	}
  }
  return body_confidence;
}

map<int, Body> find_matching_contours(map<int,vector<Rect>> contour_map , vector<Rect> bodies){
  map<int,map<int,vector<Rect>>> body_parts ;
  for (map<int,vector<Rect>>:: iterator it = contour_map.begin();it != contour_map.end();it++){
    for(Rect rect : it->second){
      for(Rect body : bodies){
        if((body & rect) == rect){
          if(body_parts.find(body.x + body.width /2) == body_parts.end())
              body_parts[body.x + body.width/2] = map<int,vector<Rect>>();
          body_parts[body.x + body.width/2][it->first].push_back(rect);
        }
      }
    }
  }
  map<int,Body> body_confidence = calc_confidence(body_parts,bodies);
  //cout << "body confidence size " << body_confidence.size() << endl;
  return body_confidence;
}
bool match_to_body(Rect a, Rect b){
	//int min_x = max(a.x,b.x);
	//int max_x = min(a.x+a.width,b.x+b.width);
	int min_x = min (a.x, b.x);
	int max_x = max (a.x + a.width,b.x+ b.width);
	int distance = max_x - min_x ;
	//if((float)(max_x - min_x) / (float)(min(a.width,b.width)) > .7)
	//cout << "rect a " << a.x <<" "<< a.width << "rect b" << b.x <<" "<< b.width << endl;
	//cout << "dist is " << distance << " and " << max(a.width,b.width) << endl;
	if(abs(distance - max(a.width,b.width)) < 5)
		return true;
	return false;
}

map<int,vector<Rect>> counter_map_extract(Mat im){
	map<int,vector<Rect>> contour_map;
	double maxValue = 100 ;
	double min,max;
	minMaxIdx(im,&min,&max);
  	vector<vector<Point>> contours;
  	vector<Vec4i> hierarchy;
  	for (double thresh = min;thresh <= max;thresh++){
  		Mat thresh_result;
  		threshold(im,thresh_result,thresh,maxValue,THRESH_BINARY);
      	findContours( thresh_result, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      	/// Approximate contours to polygons + get bounding rects and circles
      	vector<vector<Point> > contours_poly( contours.size() );
      	vector<Rect> boundRect( contours.size() );
      	
	    for( int i = 0; i < contours.size(); i++ ){
	        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
	         boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	    }
      	contour_map[thresh] = boundRect;
      	contours.clear();
      	contours.shrink_to_fit();
      	hierarchy.clear();
      	hierarchy.shrink_to_fit();
      	boundRect.clear();
      	boundRect.shrink_to_fit();  
      	contours_poly.clear();
      	contours_poly.shrink_to_fit();
      	thresh_result.release();
  	}
  	return contour_map;
}
vector<Rect> extract_bodies(map<int,vector<Rect>>& body_map){
 	vector<Rect> bodies ;
    for (map<int,vector<Rect>>::iterator it = body_map.begin();it !=body_map.end(); it++){
        bodies.insert(bodies.end(),it->second.begin(),it->second.end());
    }
    //cout << "potential body rects before merging "<< bodies.size() << endl;
    bool resized = false;
    while(true){
    	resized = false;
    	for (int i=0;i<bodies.size();i++){
    		for (int j=i+1;j<bodies.size();j++){
    			if (match_to_body(bodies.at(i),bodies.at(j))){
    				bodies.push_back(bodies.at(i) | bodies.at(j));
    				bodies.erase(bodies.begin()+i);
    				bodies.erase(bodies.begin()+j-1);
    				resized = true;
    			}
    		}
    	}
    	if(!resized)
    		break;
    }
    //cout << "potential body rects after merging "<< bodies.size() << endl;
    return bodies;
}
void contour_detector(Mat im, int frameN,vector<Person>& people,VideoWriter outputVideo){
	Mat org_im;
  	org_im = im.clone();
  	RNG rng(12345);
  	double min,max;
	minMaxIdx(im,&min,&max);
	if(max - min >=5){
		map<int,vector<Rect>> contour_map = counter_map_extract(im);
		map<int,vector<Rect>> layerd_countors;
		for (int index = max;index >= max - INTEREST_THRESH;index--){
			vector<Rect> max_rects = contour_map[index];
			//cout << "##### at index " << index << "with " << max_rects.size() << "body nomenies " << endl;
			for (Rect rect : max_rects){
				//cout << "calling find body in thersh " << index <<"with rect "<<rect.x << endl;
				find_body(rect,contour_map,index,0,layerd_countors);
			}
			max_rects.clear();
		}
		if(layerd_countors.size()> 0){
			vector<Rect> body_rects = extract_bodies(layerd_countors);
			//cout << "potential body rects " << body_rects.size() << endl;
			map<int,Body> body_confidence = find_matching_contours(contour_map,body_rects);
			if(body_confidence.size()>0){
		    	for( int i = 0; i< body_rects.size(); i++ ){
			    	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			      	rectangle( org_im, body_rects[i].tl(), body_rects[i].br(), 25, 2, 8, 0 );
		    	}
		    	update_people_info2(body_confidence , people ,frameN);
				//pause();
			}
		}
	}
	
	update_people_status(people,frameN);
	show_image(org_im,outputVideo);
	//pause();
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

void read_from_file(string file_name,VideoWriter outputVideo){
    std::ifstream infile(file_name);
    int** frame = NULL;
    int frameN = 0;
    char quite = 'n';
    vector<Person> people;
    while(true){
		frameN++;
		//cout << "processing frame " << frameN << endl;
       frame = read_frame(&infile);
       if (frame == NULL){
            cout << "Something is wrong with input file " << input_file_name << endl;
            return;	
       }
       if (frameN < START_FRAME)
       		continue; 
       else if (frameN > END_FRAME)
       		return;
       Mat im = convert_to_Mat(frame);
       Mat extended_im = resize_frame(im,frameN);
       if(blobDetection)
       		blob_detect(extended_im);
       if(contourDetection)
       		contour_detector(extended_im,frameN,people,outputVideo);
       	show_scaled(im);
       	 #ifdef ENABLE_XMPP_REPORTING
       		if(people_inside != person_count_last){
       			cout << "sending info " << people_inside << person_count_last << endl;
		        xmpp->occupancyChange(people_inside, people_inside - person_count_last);	
       		}
       		
		#endif
		person_count_last = people_inside;
    }
}

int main(int argc, char **argv){
    VideoWriter outputVideo;
   Size S = Size(extended_resolution,extended_resolution);
   //int ex = VideoWriter::fourcc('P','I','M','1');
   outputVideo.open("test.avi",  CV_FOURCC('M','J','P','G'), 10, S, true);
    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " << endl;
        return 0;
    }
    /* Initiate a timer for resetting people count at midnight */
    atomic<bool> timer_keeps_running {true} ;
	if(reset_count_at_midnight) {
		int initial_wait_time = compute_remaining_time_of_today();
		thread( reset_occupancy_count, initial_wait_time, std::ref(timer_keeps_running) ).detach() ; 
	}
	#ifdef ENABLE_XMPP_REPORTING
    	xmpp = new XMPPInterface(XMPP_JID, XMPP_PASS, XMPP_NODE, XMPP_PARENT_NODE, enable_distributed_coordination, enable_actuation, XMPP_VERBOSITY);
	#endif

  	report_updated_people_count(-200, people_inside, people_inside - person_count_last); //logging + reporting initial count
    
 
    if(enable_actuation)
    {
    	signal(SIGUSR1, update_people_count_handler);
    }
    if(LIVE == false){
		cout << "reading from file" << endl;
		read_from_file(input_file_name,outputVideo);
		return 0;
	}
	
   // libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
  //  libfreenect2::FrameMap frames;
    unsigned char buf[1024];
    int f = 0, i;
    int vid = 0x403;
    int pid = 0x6015;
    int baudrate = 115200;
    int interface = INTERFACE_A; //INTERFACE_ANY;
    int retval = EXIT_FAILURE;
    // Init
    if ((ftdi = ftdi_new()) == 0)
    {
        fprintf(stderr, "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    fprintf(stderr,"Selecting interface.\n");
    // Select interface
    int debug_res = ftdi_set_interface(ftdi, INTERFACE_A);
       
    // Open device 
    f = ftdi_usb_open(ftdi, vid, pid);
    if (f < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        exit(-1);
    }
    fprintf(stderr,"FTDI device is open now.\n");
	fprintf(stderr,"Setting up baudrate.\n");
    // Set baudrate
    f = ftdi_set_baudrate(ftdi, baudrate);
    if (f < 0)
    {
        fprintf(stderr, "unable to set baudrate: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        exit(-1);
    }
	fprintf(stderr,"Baudrate (%d) set up done.\n",baudrate);
    
	fprintf(stderr,"Registering SIGINT handler.\n");
	
    signal(SIGINT, sigintHandler);
    printf("epoch frameN Ta 1 2 3 4 5 6 7 8 9\n");
    map<int,int**> frames;
    print_time();
    vector<Person> people;
    
    while (!exitRequested)
    {
    	
    	unsigned char buf2[1];
    	buf2[0] = '*';
    	f = ftdi_write_data(ftdi, buf2, 1); //Ask PIC24F04KA200 microcontroller to start sending data
        memset(buf, 0, sizeof buf);
        //usleep(1 * 500);
        f = ftdi_read_data(ftdi, buf, sizeof(buf));
        if (f<0){
            fprintf(stderr, "Something is wrong. %d bytes read\n",f);
            usleep(1 * 1000000);
        }
        else if( f > 130 )
        {
			
            //fprintf(stderr, "read %d bytes\n", f);
            frames = interpret_data(buf, f);
            //fprintf(stdout, "\n");
            //cout << "frames size " << frames.size() << endl;
            for(map<int,int**>::iterator it=frames.begin();it!=frames.end();it++){
				Mat im = convert_to_Mat(it->second);
				Mat extended_im = resize_frame(im,it->first);
				if(blobDetection)
					blob_detect(extended_im);
				if(contourDetection)
					contour_detector(extended_im,it->first,people,outputVideo);
				//namedWindow("raw input",CV_WINDOW_AUTOSIZE);
				///imshow("raw input",im);
            }
            frames.clear();
            
            fflush(stderr);
            fflush(stdout);
        }
        #ifdef ENABLE_XMPP_REPORTING
		            xmpp->occupancyChange(people_inside, people_inside - person_count_last);
		#endif
		person_count_last = people_inside;
    }
    
    signal(SIGINT, SIG_DFL);
    retval =  EXIT_SUCCESS;
    ftdi_usb_close(ftdi);
    do_deinit:
    ftdi_free(ftdi);
    return retval;
}
