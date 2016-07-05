/**@file OccupancyCounter.h
 *
 * @date June 30, 2016
 * @author Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 */

#ifndef OCCUPANCYCOUNTER_H_
#define OCCUPANCYCOUNTER_H_
using namespace std;
using namespace cv;
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
class OccupancyCounter {
public:
	OccupancyCounter();
	void set_people_inside(int people_inside);
	int get_people_inside();
	
	void contour_detector(Mat im, int frameN,vector<Person>& people,VideoWriter outputVideo);
	Mat convert_to_Mat(int** frame);
	Mat resize_frame(Mat im, int frameN);
	void blob_detect(Mat im);
	//virtual ~OccupancyCounter();

private:
	int people_inside;
	int frameN;
	float calc_match_weight (Person person_ , Body body);
	float avg_(int start, int end, vector<int> input_);
	void update_people_status(vector<Person>& people, int frameN);
	void update_people_info(map<int,Body> body_confidence,vector<Person>& people, int frameN);
	
	
	void show_image(Mat im,VideoWriter outputVideo);
	void show_scaled (Mat im);
	
	void find_body(Rect candidate_rect, 
		map<int,vector<Rect>> contour_map, int thresh , int level,map<int,vector<Rect>>& body_map);
	map<int,Body> calc_confidence(map<int,map<int,vector<Rect>>> body_parts, vector<Rect> bodies);
	map<int, Body> find_matching_contours(map<int,vector<Rect>> contour_map , vector<Rect> bodies);
	bool match_to_body(Rect a, Rect b);
	map<int,vector<Rect>> counter_map_extract(Mat im);
	vector<Rect> extract_bodies(map<int,vector<Rect>>& body_map);
	
};

#endif /* OCCUPANCYCOUNTER_H_ */
