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

class ID_Rect {
	public:
		Rect rect_;
		int T;
		string getID(){
			return to_string(T)+to_string(rect_.x)+to_string(rect_.area());
		}
		bool operator< (const ID_Rect& rhs) const
		{
			return rect_.area() < rhs.rect_.area();
		}
};
struct rectCompare {
    bool operator() (const ID_Rect& lhs, const ID_Rect& rhs) const
    {
        return lhs.T < rhs.T;
    }
    
};


struct Body {
	int order_id;
    string ID_;
    set<ID_Rect> rects;
    ID_Rect baseRect;
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
	
	void process_frame(Mat im, int frameN,vector<Person>& people,VideoWriter outputVideo);
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
	void update_people_info(map<string,Body> body_confidence,vector<Person>& people, int frameN);
	
	
	void show_image(Mat im,VideoWriter outputVideo);
	void show_scaled (Mat im);
	
	void find_body(Rect candidate_rect, 
		map<int,vector<Rect>> contour_map, int thresh ,map<string,Body>& body_map,set<ID_Rect> parents,vector<string>& taken);
	map<string,Body> calc_confidence(map<string,Body> body_map);
	bool match_to_body(Rect a, Rect b);
	map<int,vector<Rect>> counter_map_extract(Mat im);
};

#endif /* OCCUPANCYCOUNTER_H_ */
