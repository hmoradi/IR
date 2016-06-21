#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include "params.h"
#include "clipper.hpp"
using namespace cv;
using namespace std;
using namespace ClipperLib;

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
	cout << frameN << " " << min << " " << max << endl; 
	return result;
}
void show_image(Mat im){
   if(!showImage)
   	   return;
   Mat im_color;
   double min,max;
   minMaxIdx(im,&min,&max);
   Mat scaled_image;
  
   if(scaleImage){
   	    convertScaleAbs(im,scaled_image,255 / max);
   	    applyColorMap(scaled_image,im_color,COLORMAP_JET);
   	    imshow("key points- scaled",im_color);
   }else{
        applyColorMap(im,im_color,COLORMAP_JET);
   	    imshow("key points - raw image",im_color);
   }
   waitKey(100);
}
void blob_detect(Mat im){
    //cout << im << endl;
    //Mat im = imread( "../blob2.jpg", IMREAD_GRAYSCALE );

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
    cout << keypoints.size() << endl;
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
double polygonArea(double *X, double *Y, int points) {

  double  area=0. ;
  int     i, j=points-1  ;
  for (i=0; i<points; i++) {
    area+=(X[j]+X[i])*(Y[j]-Y[i]); j=i; 
  }
  return area*.5; 
} 

void find_body(Rect candidate_rect, map<int,vector<Rect>> contour_map, int thresh , int level,map<int,vector<Rect>>& body_map){
  //  cout << "find body " << candidate_rect.x <<" " << candidate_rect.y <<" area "<<candidate_rect.area() << " th " << thresh <<" le "  << level << endl; 
  if (candidate_rect.area() >= RECTMAXAREA or candidate_rect.area() < RECTMINAREA)
        return;
  vector<Rect> rects = contour_map[thresh-1];
  for (Rect rect : rects  ){
    //cout << "next rect is " << rect.x  <<" " << rect.y << " " << rect.area() << endl;
    if (rect.area() >= RECTMAXAREA or rect.area() < RECTMINAREA)
        continue;
    //cout << "this one is passed 1" << endl;
    Rect intersect = candidate_rect & rect;
    //cout << "intersected" << endl;
    if (intersect == candidate_rect){
      //cout << "intersection is matching " << endl;
      if(level == 2){
        if (body_map.find(thresh - 1) == body_map.end()){
          body_map[thresh - 1]= vector<Rect>();
        }
        body_map[thresh-1].push_back(rect);
        //cout << "rect is pushed back to body map at thresh " << thresh - 1 << endl;

      }
      else{
        //cout << "got to next level" << endl;
        find_body(rect,contour_map,thresh -1,level + 1,body_map);  
      }
      
    }
    //cout << "intersection did not match" << endl;
  }
  rects.clear();
  //cout << "cleared rects " << endl;
  return;
}
map<int,float> calc_confidence(map<int,map<int,vector<Rect>>> body_parts, vector<Rect> bodies){
  map<int,float> body_confidence;
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
        if(confidence > 0.6)
          body_confidence[body_location] = confidence;
    }
  }
  return body_confidence;
}
map<int,float> find_matching_contours(map<int,vector<Rect>> contour_map , vector<Rect> bodies){
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
  map<int,float> body_confidence = calc_confidence(body_parts,bodies);
  cout << "body confidence size " << body_confidence.size() << endl;
  return body_confidence;
}
void contour_detector(Mat im, int frameN){
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
      //cout << "before "  << im.data(0,0)<endl;
      threshold(im,thresh_result,thresh,maxValue,THRESH_BINARY);
      //cout << "after "  << im.at<uchar>(0,0)<<endl;
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
  
  cout << "body map size is " <<body_map.size() << endl;
  if(body_map.size()> 0){
    cout << body_map.size() << endl;
    vector<Rect> bodies ;
    for (map<int,vector<Rect>>::iterator it = body_map.begin();it !=body_map.end(); it++){
        bodies.insert(bodies.end(),it->second.begin(),it->second.end());
    }
    map<int,float> body_confidence = find_matching_contours(contour_map,bodies);
    if(body_confidence.size()>0){
    	for( int i = 0; i< bodies.size(); i++ ){
      		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      		//drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      		rectangle( org_im, bodies[i].tl(), bodies[i].br(), 25, 2, 8, 0 );
    	}	
    }
   	show_image(org_im);
  }
}
void read_from_file(string file_name){
    std::ifstream infile(input_file_name);
    int** frame = NULL;
    int frameN = 0;
    char quite = 'n';
    while(true){
       frame = read_frame(&infile);
       if (frame == NULL)
            return;	
       Mat im = convert_to_Mat(frame);
       Mat extended_im = resize_frame(im,frameN);
       if(blobDetection)
       		blob_detect(extended_im);
       if(contourDetection)
       		contour_detector(extended_im,frameN);
       frameN++;
       //cin >> quite;
       if (quite == 'y')
          return;
    }
}
int main(int argc, char** argv ){
    read_from_file("../8.txt");
    return 0;
}