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

#define PacketLength 134
struct ftdi_context *ftdi;
int IR_array_data[8][8];
unsigned char residue[1024] ;
static int exitRequested = 0;
static int frameNumber = 0;
static long int lastTime =0;
static int lastFrame = 0;
static int frameRate = 0;
static int frameN = 0;

enum status {UNKNOWN, ENTERED,  LEFT};
enum direction {UNKNOWNDIR, LtoR , RtoL};

struct Body {
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
};

float calc_match_weight (Person person_ , Body body){
	map<int,Body>::reverse_iterator last_location = person_.trajectory.rbegin();
	double distance =  norm(last_location->second.rect_.tl()-body.rect_.tl());
	float size = abs(last_location->second.rect_.area() - body.rect_.area()) / last_location->second.rect_.area(); 
	float T = abs(last_location->second.T - body.T) / last_location->second.T;
	return (distance + size + T)  / 3.0;
	return 0;
}
void update_people_status(vector<Person>& people, int frameN){
	cout << "updateing status  of " << people.size() << endl;
	for(Person& person:people){
		map<int,Body>::iterator it = person.trajectory.end();
		int last_seen_frame = it->first;
		Body last_seen_body = it->second ;
		if(person.status_ == UNKNOWN){
			cout << "status is unknown " << endl;
			person.status_ = ENTERED;
		}
		else if (person.status_ == ENTERED){
			cout << "person already enreted and last frame seen is "<< last_seen_frame << endl;
			if(frameN - last_seen_frame > 10){
				person.status_ = LEFT;
				if(person.direction_ == LtoR){
					cout << "one move from Left to Right" << endl;;
					char q;
					cin >> q;
				}
				else if(person.direction_ == RtoL){
					cout << "one move from Right to Left " << endl;
					char q;
					cin >> q;
				}else{
					cout << "unknown direction  " << endl;
					char  q;
					cin >> q;
				}
				
			}
			else{
				if (last_seen_body.rect_.x < person.last_location and (person.direction_ == RtoL or person.direction_ == UNKNOWNDIR) ){
					person.direction_ = RtoL;
					
				}
				else if(last_seen_body.rect_.x < person.last_location and (person.direction_ == LtoR or person.direction_ == UNKNOWNDIR)){
					person.direction_ = LtoR;
					
				}else{
					person.direction_ = UNKNOWNDIR;
				}
				person.last_location = last_seen_body.rect_.x;
				cout << "last location is " << person.last_location << endl;
			}	
		}
	}
	for(Person per:people)
		cout << "updated status is " << per.status_ << endl;
}
void update_people_info(map<int,Body> body_confidence , vector<Person>& people , int frameN){
	
	float match_weight ;
	float max_weight = 0;
	
	map<float , Person> weight_values;
	map<Body,map<float,Person>,bodyCompare> pairs;
	map<float,Body> weights_list ;
	
	
	int ID = 0;
	for(Person& perr:people){
		perr.ID_ = ID;
		ID++;
	}
	
	cout << "Frame Number is "<< frameN << endl;
	cout << "number of bodies " << body_confidence.size() << endl;
	for(map<int,Body>::iterator it = body_confidence.begin();it != body_confidence.end();it++){
		max_weight = 0;
		for(Person& person_ :people){
			map<int,Body>::iterator peopleIT = person_.trajectory.end();
			if(person_.status_ == LEFT)
				continue;
			match_weight = calc_match_weight(person_,it->second);
			max_weight = max(match_weight,max_weight);
			weight_values[match_weight] = person_;
			cout << "match weight is " << match_weight << endl;
		}
		pairs[it->second] = weight_values;
		weights_list[max_weight] = it->second;
		weight_values.clear();
	}
	cout << "number of matched pairs "<< weights_list.size() << endl;
	vector<Person> updated_people;
	vector<Person>::iterator peopleIT;
	for(map<float,Body>::reverse_iterator bodyIT = weights_list.rbegin();bodyIT != weights_list.rend();bodyIT++){
		float current_max = bodyIT -> first;
		map<Body,map<float,Person>>::iterator current_bodyIT = pairs.find(bodyIT->second);
		map<float,Person> values = current_bodyIT->second;
		Body current_body = current_bodyIT->first;
		bool found = false;
		bool pushed = false;
		cout << "size of values is " << values.size()<< endl;
		for(map<float,Person>::reverse_iterator itt = values.rbegin();itt != values.rend();itt++){
			for(Person per:updated_people){
				if(per.ID_ == itt->second.ID_){
					found = true;
					break;
				}
			}
			if(!found){
				itt->second.trajectory[frameN] = current_body;
				updated_people.push_back(itt->second);
				pushed = true;
				cout << "pushing person " << itt->second.ID_ <<"at frame " <<frameN<< endl;
				cout << "size of trajectory is "<<itt->second.trajectory.size() << endl;
				break;
  			}
		}
		if(!pushed){
			
			Person new_one ;
			new_one.trajectory[frameN]= current_body;
			new_one.status_ = UNKNOWN;
			new_one.direction_ = UNKNOWNDIR;
			new_one.ID_ = ID;
			cout << "pushing body "<<ID << "at frame "<<frameN<<endl;
			updated_people.push_back(new_one);
		}
		
	}
	
	people.clear();
	people.insert(people.end(),updated_people.begin(),updated_people.end());
	updated_people.clear();
	char dummy;
	//cin >> dummy;
	//if(dummy == 'q')
	//	exit(0);
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
    cout << buf <<" " << frameN << endl;
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
   waitKey(1);
   
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
  if (candidate_rect.area() >= RECTMAXAREA or candidate_rect.area() < RECTMINAREA)
        return;
  vector<Rect> rects = contour_map[thresh-1];
  for (Rect rect : rects  ){
    if (rect.area() >= RECTMAXAREA or rect.area() < RECTMINAREA)
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
  for(map<int,map<int,vector<Rect>>>::iterator it = body_parts.begin();it != body_parts.end();it++){
    int body_location = it->first;
    Rect body ;
    for(Rect rect:bodies){
      if (rect.x == body_location){
        body = rect;
        break;
      }
    }
    float confidence = 0;
    map<int,vector<Rect>> parts = it->second;
    for(map<int,vector<Rect>>::iterator inner_it = parts.begin();inner_it!= parts.end();inner_it++){
        Rect result;
        int temperature = it->first;
        for(Rect rect:inner_it->second){
          result |= rect;
        }
        Rect coverage = result & body;
        float coverage_conf = (float)coverage.area() / (float)body.area();
        float temp_conf = 1.0 / (float)max(1, 24 - temperature);
        float height_conf = (float)result.height / (float)extended_resolution;
        confidence += (temp_conf)*(coverage_conf + height_conf);
        if(confidence > 0.6){
			Body body_ ;
			body_.rect_ = body;
			body_.T = temperature;
			body_.confidence = confidence;
          body_confidence[body_location] = body_;
		}
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
          if(body_parts.find(body.x) == body_parts.end())
              body_parts[body.x] = map<int,vector<Rect>>();
          body_parts[body.x][it->first].push_back(rect);
        }
      }
    }
  }
  map<int,Body> body_confidence = calc_confidence(body_parts,bodies);
  //cout << "body confidence size " << body_confidence.size() << endl;
  return body_confidence;
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
  
  //cout << "body map size is " <<body_map.size() << endl;
  if(body_map.size()> 0){
    //cout << body_map.size() << endl;
    vector<Rect> bodies ;
    for (map<int,vector<Rect>>::iterator it = body_map.begin();it !=body_map.end(); it++){
        bodies.insert(bodies.end(),it->second.begin(),it->second.end());
    }
    map<int,Body> body_confidence = find_matching_contours(contour_map,bodies);
    if(body_confidence.size()>0){
    	for( int i = 0; i< bodies.size(); i++ ){
      		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      		//drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      		rectangle( org_im, bodies[i].tl(), bodies[i].br(), 25, 2, 8, 0 );
    	}
    	update_people_info(body_confidence , people ,frameN);
    	cout << "people size " << people.size() << endl;
    	
    }
    update_people_status(people,frameN);
   	show_image(org_im);
  }
}

void read_from_file(string file_name){
    std::ifstream infile(input_file_name);
    int** frame = NULL;
    int frameN = 0;
    char quite = 'n';
    vector<Person> people;
    while(true){
		frameN++;
		//cout << "processing frame " << frameN << endl;
       frame = read_frame(&infile);
       if (frame == NULL)
            return;	
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
       
       //cin >> quite;
       if (quite == 'y')
          return;
    }
}

int main(int argc, char **argv){
    if(LIVE == false){
		cout << "reading from file" << endl;
		read_from_file("../8.txt");
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
