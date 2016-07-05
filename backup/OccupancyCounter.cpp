/**@file OccupancyCounter.cpp
 *
 * @date July 3, 2016
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
#include "OccupancyCounter.h"
char temp;
#define pause() (cin >> temp)

OccupancyCounter::OccupancyCounter( ){
    this->people_inside = 0;
    this->frameN = 0;
}
void OccupancyCounter::set_people_inside(int people_inside){
    this->people_inside = people_inside;
}
int OccupancyCounter::get_people_inside(){
    return this->people_inside;
}
float OccupancyCounter::calc_match_weight (Person person_ , Body body){
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
float OccupancyCounter::avg_(int start, int end, vector<int> input_){
    //cout << "averageing vector with size "<< input_.size() << "starting from " << start << "ending at" << end << endl;
    float sum = 0;
    int count = 0;
    for(int i=start;i<end;i++){
        sum+= input_.at(i);
        count += 1;
    }
    return sum / max(count , 1);
}
Mat OccupancyCounter::convert_to_Mat(int** frame){
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
Mat OccupancyCounter::resize_frame(Mat im, int frameN){
    Size size(extended_resolution,extended_resolution);
    Mat result;
    resize(im,result,size,INTER_CUBIC);
    double min, max;
    Point minL,maxL;
    minMaxLoc(result,&min,&max, &minL,&maxL);
    //cout << frameN << " " << min << " " << max << endl; 
    return result;
}
void OccupancyCounter::show_image(Mat im,VideoWriter outputVideo){
   
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
void OccupancyCounter::show_scaled (Mat im){
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

void OccupancyCounter::update_people_status(vector<Person>& people, int frameN){
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
void OccupancyCounter::update_people_info(map<int,Body> body_confidence,vector<Person>& people, int frameN){
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
    //  pause();
}


void OccupancyCounter::blob_detect(Mat im){
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

void OccupancyCounter::find_body(Rect candidate_rect, map<int,vector<Rect>> contour_map, 
    int thresh , int level,map<int,vector<Rect>>& body_map){
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
        float percentage = (float)intersect.area() / (float)min(candidate_rect.area(),rect.area());
        if (percentage >= 0.7){
            if(level == BODY_DEPTH){
                bool inserted = false;
                for(Rect rect_temp : body_map[thresh-1]){
                    if (rect_temp == rect)
                        inserted = true;
                }
                if(!inserted)
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

map<int,Body> OccupancyCounter::calc_confidence(map<int,map<int,vector<Rect>>> body_parts, vector<Rect> bodies){
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

map<int, Body> OccupancyCounter::find_matching_contours(map<int,vector<Rect>> contour_map , vector<Rect> bodies){
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
bool OccupancyCounter::match_to_body(Rect a, Rect b){
    int min_x = min (a.x, b.x);
    int max_x = max (a.x + a.width,b.x+ b.width);
    int distance = max_x - min_x ;
    if(abs(distance - max(a.width,b.width)) < 5)
        return true;
    return false;
}

map<int,vector<Rect>> OccupancyCounter::counter_map_extract(Mat im){
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
vector<Rect> OccupancyCounter::extract_bodies(map<int,vector<Rect>>& body_map){
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
void OccupancyCounter::contour_detector(Mat im, int frameN,vector<Person>& people,VideoWriter outputVideo){
    Mat org_im;
    vector<Rect> root_rects;
    org_im = im.clone();
    RNG rng(12345);
    double min,max;
    minMaxIdx(im,&min,&max);
    if(max - min >=MIN_DIFF_IN_FRAME){
        map<int,vector<Rect>> contour_map = counter_map_extract(im);
        map<int,vector<Rect>> layerd_countors;
        for (int index = max;index >= max - ROOT_DEPTH;index--){
            root_rects = contour_map[index];
            for (Rect rect : root_rects){
                find_body(rect,contour_map,index,0,layerd_countors);
            }
            root_rects.clear();
        }
        if(layerd_countors.size()> 0){
            vector<Rect> body_rects = extract_bodies(layerd_countors);
            map<int,Body> body_confidence = find_matching_contours(contour_map,body_rects);
            if(body_confidence.size()>0){
                for( int i = 0; i< body_rects.size(); i++ ){
                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                    rectangle( org_im, body_rects[i].tl(), body_rects[i].br(), 25, 2, 8, 0 );
                }
                update_people_info(body_confidence , people ,frameN);
                //pause();
            }
        }
    }
    
    update_people_status(people,frameN);
    show_image(org_im,outputVideo);
    //pause();
}