/**@file GridEyeReader.cpp
 *
 * @date June 30, 2016
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
#include "GridEyeReader.h"
using namespace std;
using namespace cv;

GridEyeReader::GridEyeReader(){

    f = 0;
    i = 0;
    vid = 0x403;
    pid = 0x6015;
    baudrate = 115200;
    interface = INTERFACE_A; //INTERFACE_ANY;
    
    //this-> retval = EXIT_FAILURE;
}
int GridEyeReader::find_packet_head(unsigned char *buf, int index, int len){
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
void GridEyeReader::array_copy(unsigned char* dest, unsigned char* source,int start_index, int len){
    
    for (int i=0;i<len;i++){
        dest[i] = source[start_index+i];
    }
}
//If there is not complete packet, buffer it at residue buffer 
void GridEyeReader::add_to_residue(unsigned char* buf, int index){
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
int** GridEyeReader::print_packet(unsigned char* buf,int index){
    
    
	int** frame = new int*[8];
    for (int i=0;i<8;i++)
        frame[i] = new int[8];    
    //int thermistor_data = ((int)buf[index + 4]<<8) | buf[index +3]; //Next 2 characters [index: 3, 4] are thermistor bytes	
    int k = index + 5; 
    int high_byte;
    int temperature;
    unsigned char low_byte;
    int i,j;
    for( i = 0; i < 8; i++) {
        for( j = 0; j < 8; j++) {
            high_byte = buf[k+1];
            low_byte = buf[k];
            temperature = (high_byte << 8) | low_byte;
            temperature = temperature / 4;
            frame[i][j] = min(max(temperature,15),40);
            k+=2;
        }
    }
	if (i ==8 and j == 8){
		frameNumber++;
		return frame;
	}
	return NULL;
}

map<int,int**> GridEyeReader::interpret_data(unsigned char *buf, int len){
	int** frame;
	map<int,int**> frames;
	int index = find_packet_head(buf,0,len);
	if(index != 0){
        add_to_residue(buf,index);
        frame = print_packet(residue,0);
        if(frame != NULL)
			frames[frameNumber] = frame;
        memset(residue,0,sizeof residue);
    }else{
        if(residue[0] != 0){
           frame = print_packet(residue,0);
           if(frame != NULL)
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
        if(frame != NULL)
			frames[frameNumber] = frame;
        index+= PacketLength;
        index = find_packet_head(buf,index,len); 
    }
    return frames;	
}
int** GridEyeReader::read_frame(std::ifstream* infile){
    int** frame = new int*[8];
    for (int i=0;i<8;i++)
        frame[i] = new int[8];
    int epoch,frameN,Ta;
    int index = 0;
    
    while(*infile >> epoch >> frameN >> Ta >> frame[index][0] >> frame[index][1] >> frame[index][2] >> frame[index][3] >> frame[index][4] >> frame[index][5] >> frame[index][6] >> frame[index][7]){
    	
    	for(int j=0;j<8;j++)
			frame[index][j] = min(max(frame[index][j],14),40);
    	index ++;
    	if (index >=8){
    		return frame;
    	}
    }
    return NULL;
}

int GridEyeReader::Init(){
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
    f = ftdi_usb_open(ftdi, this->vid, this->pid);
    if (f < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", this->f, ftdi_get_error_string(this->ftdi));
        exit(-1);
    }
    fprintf(stderr,"FTDI device is open now.\n");
	fprintf(stderr,"Setting up baudrate.\n");
    // Set baudrate
    f = ftdi_set_baudrate(this->ftdi, this->baudrate);
    if (f < 0)
    {
        fprintf(stderr, "unable to set baudrate: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        exit(-1);
    }
    fprintf(stderr,"Baudrate (%d) set up done.\n",baudrate);
}
    
int GridEyeReader::ReadFrame(unsigned char* buf){
	unsigned char buf2[1];
    buf2[0] = '*';
    f = ftdi_write_data(ftdi, buf2, 1); //Ask PIC24F04KA200 microcontroller to start sending data
    memset(buf, 0, sizeof buf);
    //usleep(1 * 500);
   	f = ftdi_read_data(ftdi, buf, 1024);
    //cout << "read form sensor " << f << endl;
    if (f<0){
        fprintf(stderr, "Something is wrong. %d bytes read\n",f);
        usleep(1 * 1000000);
    }
    return f;
}
GridEyeReader::~GridEyeReader(){
    if(!LIVE)
        return;
    unsigned char buf2[1];
    buf2[0] = '~';
    int f = ftdi_write_data(ftdi, buf2, 1); // Asking PIC24F04KA200 microcontroller to stop sending data
    if( f < 0)
        cout<< "Error in writing data: " << endl;
	ftdi_usb_close(this->ftdi);
    ftdi_free(this->ftdi);
}
     
    
	
