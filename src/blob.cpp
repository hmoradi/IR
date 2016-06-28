#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include "ftdi.h"
#include <time.h>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include "params.h"

using namespace cv;
using namespace std;
char temp;
#define pause() (cin >> temp)

#define PacketLength 134
struct ftdi_context *ftdi;
unsigned char residue[1024] ;
static int exitRequested = 0;
static int frameNumber = 0;
static int frameN = 0;

enum status {UNKNOWN, ENTERED,  LEFT};
enum direction {UNKNOWNDIR, LtoR , RtoL};

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

float calc_match_weight (Person person_ , Body body){
	map<int,Body>::reverse_iterator last_location = person_.trajectory.rbegin();
	double distance =  norm(last_location->second.rect_.tl()-body.rect_.tl());
	float size = abs(last_location->second.rect_.area() - body.rect_.area()) / last_location->second.rect_.area(); 
	float T = abs(last_location->second.T - body.T) / last_location->second.T;
	return   6.0 / max((distance + 3*size + 2*T),0.001) ;
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
		cout << "updateding status for " << people.size() - left_person<< "people" << endl;
		if(person_.status_ == UNKNOWN){
			//cout << "status is unknown " << endl;
			person_.status_ = ENTERED;
		}
		else if (person_.status_ == ENTERED){
			if(frameN - person_.last_frame_seen >= 10){ //Old Person
				person_.status_= LEFT;
				vector<int> locations ;
				vector<int> sorted_locations;
			
				float frame_confidence = 0;
				int min_x = extended_resolution;
				int min_t = 0;
				int max_x = 0;
				int max_t = 0;
				for(auto item :person_.trajectory){
					locations.push_back(item.second.rect_.x+ (item.second.rect_.width / 2));
					//cout << "location is " << item.second.rect_.x + (item.second.rect_.width / 2) <<endl;;
					//cout << "location area is "<<item.second.rect_.area() << endl;
					sorted_locations.push_back(item.second.rect_.x+ (item.second.rect_.width / 2));
				
					frame_confidence += item.second.confidence;
				
					min_t = min(min_t,item.first);
					max_t = max(max_t,item.first);
					//cout << "person id " <<person_.ID_ << " trajectory frame "<< item.first << endl;
				}
				//person_.last_frame_seen = max_t;
				//person_.last_body_seen = person_.trajectory[max_t];

				sort(sorted_locations.begin(),sorted_locations.end());
				max_x = sorted_locations.at(sorted_locations.size()-1);
				min_x = sorted_locations.at(0);
				float speed = (float)(max_x - min_x)/(float)(max(max_t-min_t,1));
				float avg_confidence = frame_confidence /(float)locations.size();
				//cout << "location size is " << locations.size() << endl;
				//cout << "speed is  " << speed << endl;
				//cout << "avg confidence " << avg_confidence << endl;
				if(locations.size() >= 2 and speed != 0){
					float first_half_average = avg_(0, locations.size()/2,locations);    
					float second_half_average = avg_(((locations.size()/2)) , locations.size(),locations);
					//cout << "first half " << first_half_average << " second half " << second_half_average <<endl;
					if(first_half_average < second_half_average)
						cout << "&&&&&&&&&&&&&&&&&&&&left to right" << endl;
					else
						cout << "$$$$$$$$$$$$$$$$$$$right to left " << endl;
					cout << "person temperature is " << person_.last_body_seen.T << endl;
					pause();
				}else if(speed > 10 and speed < 120){
					if(person_.direction_ == LtoR)
						cout << "speed move from left to right"<< endl;
					else if(person_.direction_ == RtoL)
						cout << "speed move from right to left" << endl;
					else
						cout << "speed move without direction detected "<< endl;

				}else if(avg_confidence > 2){
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
	int num_of_persons = people.size();
	for(Person& person:people){
		person.ID_ = person_ID;
		persons.push_back(person);
		person_ID++;
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
			//cout <<"person "<<max_col << " is matched with body "<<max_row << endl;
			matched[max_row] = max_col;
			for(int k=0;k<num_of_persons;k++)
				pairs_matrix[max_row][k] = -1;
			for(int k=0;k<num_of_bodies;k++)
				pairs_matrix[k][max_col] = -1;
			num_of_matched_pairs ++;	
		}else{
			//cout << "matching is done " << endl;
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
			//cout << "createing new person with id  "<<new_one.ID_ << " at frame "<<frameN<<endl;
			people.push_back(new_one);
		}
	}
	bool found = false;
	//cout << "at frame "<< frameN << " num of matched pairs " << num_of_matched_pairs << endl;
	for(Person& person_:persons){
		found = false;
		//cout << "persion id " << person_.ID_ << endl;
		for(auto item:matched){
			//cout << "matched body " <<item.first << "with person "<<item.second <<endl;
			if(item.second == person_.ID_){
				found = true;
			}
		}
		if(!found){
			people.push_back(person_);
			//cout << "could not match this person to any body "<< person_.ID_ << "at frame "<< frameN << endl;
		}
	}
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
    
    //
    
    
    //for(int i = 0; i < 8; i++){     
    //    if(PRINT_FRAME)
	//		printf("%ld %d %d ",epoch, frameNumber,thermistor_data/16);
    //    for(int j = 0; j < 8; j++){
	//		if(PRINT_FRAME)
	//			printf("%d ", IR_array_data[i][j]/4); //by looking at the datasheet, the LSB stands for .25 degree C.
	//		frame[i][j] = IR_array_data[i][j]/4;
	//	}
    //    if(PRINT_FRAME)
	//		printf("\n");
   // }
    
   
    //
   // fflush(stdout);
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

void show_image(Mat im){
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
   	    convertScaleAbs(im,scaled_image,255 / max);
   	    applyColorMap(scaled_image,im_color,COLORMAP_JET);
   	    flip(im_color,display,0);
   	    putText(display, to_string(frameN), textOrg, fontFace,fontScale,Scalar::all(40),thickness,8  );
   	    namedWindow("key points- scaled",CV_WINDOW_AUTOSIZE);
   	    imshow("key points- scaled",display);
   }else{
        applyColorMap(im,im_color,COLORMAP_JET);
		namedWindow("key points - raw image",CV_WINDOW_AUTOSIZE);
   	    imshow("key points - raw image",im_color);
   }
   waitKey(WAIT_TIME);
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
        show_image(im_with_key_points);
    }
}

void find_body(Rect candidate_rect, map<int,vector<Rect>> contour_map, int thresh , int level,map<int,vector<Rect>>& body_map){
  int maxx = (RECTMAXAREA*extended_resolution*extended_resolution)/100;
  int minx = (RECTMINAREA*extended_resolution*extended_resolution)/1000;
  minx = 5;
  if (candidate_rect.area() >= maxx or candidate_rect.area() < minx)
        return;
  vector<Rect> rects = contour_map[thresh-1];
  for (Rect rect : rects  ){
    if (rect.area() >= maxx or rect.area() < minx)
        continue;
    Rect intersect = candidate_rect & rect;
    if (intersect == candidate_rect){
      if(level == 2){
        if (body_map.find(thresh - 1) == body_map.end()){
          body_map[thresh - 1]= vector<Rect>();
        }
        body_map[thresh-1].push_back(rect);
      }
      else{
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
        float temp_conf = 1.0 / (float)max(1, abs(25 - temperature));
        float height_conf = (float)result.height / (float)extended_resolution;
        float width_conf = (float)result.width / (float)extended_resolution;
        float size_conf = (float)(body.area()) / (float)(extended_resolution*extended_resolution);
        confidence += (float)(2*temp_conf + coverage_conf + height_conf+size_conf- width_conf)/5.0 ;
    }
    //cout << "new confidence is "  << confidence << endl;
    if(confidence >= 1.0){
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
	int min_x = max(a.x,b.x);
	int max_x = min(a.x+a.width,b.x+b.width);
	if((float)(max_x - min_x) / (float)(min(a.width,b.width)) > .7)
		return true;
	return false;
}
void contour_detector(Mat im, int frameN,vector<Person>& people){
  map<int,vector<Rect>> contour_map;
  Mat org_im;
  org_im = im.clone();
  RNG rng(12345);
	//double thresh = 18;
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
      //vector<Point2f>center( contours.size() );
      //vector<float>radius( contours.size() );
      for( int i = 0; i < contours.size(); i++ ){
          approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
          boundRect[i] = boundingRect( Mat(contours_poly[i]) );
          //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
      }
      contour_map[thresh] = boundRect;
      //cout << "adding " << boundRect.size() << "to thresh " << thresh << endl;
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

  //cout << "size of contour map is "<< contour_map.size() << endl;
  map<int,vector<Rect>> body_map;
  for (int index = max;index >= max - INTEREST_THRESH;index--){
      vector<Rect> max_rects = contour_map[index];
      for (Rect rect : max_rects){
        find_body(rect,contour_map,max,0,body_map);
      }
      max_rects.clear();
  }
  if(body_map.size()> 0){

    vector<Rect> bodies ;
    for (map<int,vector<Rect>>::iterator it = body_map.begin();it !=body_map.end(); it++){
        bodies.insert(bodies.end(),it->second.begin(),it->second.end());
    }
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
    map<int,Body> body_confidence = find_matching_contours(contour_map,bodies);
    if(body_confidence.size()>0){
    	for( int i = 0; i< bodies.size(); i++ ){
    		//cout << "body area " << bodies[i].area()<<endl;
      		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      		//drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      		rectangle( org_im, bodies[i].tl(), bodies[i].br(), 25, 2, 8, 0 );
    	}
    	update_people_info2(body_confidence , people ,frameN);
    	//pause();
    }
    update_people_status(people,frameN);
   	show_image(org_im);
   	//pause();
  }
}

void read_from_file(string file_name){
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
       		contour_detector(extended_im,frameN,people);
    }
}

int main(int argc, char **argv){
    if(LIVE == false){
		cout << "reading from file" << endl;
		read_from_file(input_file_name);
		return 0;
	}
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
			
            fprintf(stderr, "read %d bytes\n", f);
            frames = interpret_data(buf, f);
            //fprintf(stdout, "\n");
            //cout << "frames size " << frames.size() << endl;
            for(map<int,int**>::iterator it=frames.begin();it!=frames.end();it++){
				Mat im = convert_to_Mat(it->second);
				Mat extended_im = resize_frame(im,it->first);
				if(blobDetection)
					blob_detect(extended_im);
				if(contourDetection)
					contour_detector(extended_im,it->first,people);
            }
            frames.clear();
            
            fflush(stderr);
            fflush(stdout);
        }
    }
    
    signal(SIGINT, SIG_DFL);
    retval =  EXIT_SUCCESS;
    ftdi_usb_close(ftdi);
    do_deinit:
    ftdi_free(ftdi);
    return retval;
}
