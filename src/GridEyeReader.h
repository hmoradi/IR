/**@file GridEyeReader.h
 *
 * @date June 30, 2016
 * @author Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 */

#ifndef GRIDEYEREADER_H_
#define GRIDEYEREADER_H_
using namespace std;
class GridEyeReader {
public:
	GridEyeReader();
	int ReadFrame(unsigned char* buf);
	int** read_frame(std::ifstream* infile);
	map<int,int**> interpret_data(unsigned char *buf, int len);
	virtual ~GridEyeReader();

private:
	//std::ifstream infile;
	struct ftdi_context *ftdi;
	unsigned char residue[1024] ;
	int exitRequested = 0;
	int frameNumber = 0;
	unsigned char buf[1024];
    int f ; 
    int i;
    int vid;
    int pid;
    int baudrate;
    int interface;
    int find_packet_head(unsigned char *buf, int index, int len);
	void array_copy(unsigned char* dest, unsigned char* source,int start_index, int len);
	void add_to_residue(unsigned char* buf, int index);
	int** print_packet(unsigned char* buf,int index);
	int Init();
};

#endif /* GRIDEYEREADER_H_ */
