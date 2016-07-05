// General configuration
#define extended_resolution 256
#define contourDetection true  
#define RECTMAXAREA 80
#define RECTMINAREA 1
#define ROOT_DEPTH 1
#define BODY_DEPTH 2
#define OPOSITE_DIRECTION false
#define BODY_THRESHOLD 0.5
#define TRAJECTORY_LENGTH 4
#define DETECTION_DELAY 5
#define SAVE_VIDEO false
#define enable_publishing_indicator_count true
#define ENABLE_ACTUATION false
#define reset_count_at_midnight true
#define ENABLE_XMPP_REPORTING false
#define MIN_DIFF_IN_FRAME 10
//Debug Configuration
#define PRINT_FRAME false
//Offline stream
#define input_file_name "../data/exp1.txt"
#define START_FRAME 1
#define END_FRAME 999
//Live Stream
#define LIVE true
#define PacketLength 134
//GUI Settings
#define colorMap true
#define scaleImage true  // scale image to focus on colors
#define showImage false //show all the frames
#define WAIT_TIME 100
//OpenCV Blob Detection Parameteres 
#define blobDetection false
#define blobMinThreshold 12
#define blobMaxThreshold 25
#define blobThresholdStep 1
#define minDistBetweenBlobs 4
#define blobFilterByArea true
#define blobMinArea 2
#define blobMaxArea 5000
#define blobFilterByCircularity false
#define blobMinCircularity 0.1
#define blobMaxCircularity 1
#define blobFilterByConvexity false
#define blobMinConvexity 0.87
#define blobMaxConvexity 1
#define blobFilterByInertia false
#define blobMinInertiaRatio 0.01
#define blobMaxInertiaRatio 1
#define blobFilterByColor false
#define blobColor 255
