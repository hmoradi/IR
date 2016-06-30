// General configuration
#define extended_resolution 256
#define contourDetection true  
#define RECTMAXAREA 80
#define RECTMINAREA 1
#define INTEREST_THRESH 3
#define OPOSITE_DIRECTION true
#define DEBUGLEVEL 1
//Debug Configuration
#define PRINT_FRAME false
//Offline stream
#define input_file_name "../data/d150_1.txt"
#define START_FRAME 1
#define END_FRAME 600
//Live Stream
#define LIVE false
#define PacketLength 134
//GUI Settings
#define colorMap true
#define scaleImage true  // scale image to focus on colors
#define showImage true //show all the frames
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