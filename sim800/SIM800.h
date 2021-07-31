#ifndef SIM_800_H
#define SIM_800_H
#include "mbed.h"
#include "SerialHandler.h"

extern SerialHandler ser;
extern RawSerial pc;
extern void custom_wait(int duration);
#define DATA_BUFFER_SIZE 500
#define TEST_CMND 0
#define READ_CMND 1
#define EXEC_CMND 2
#define WRITE_CMND 3

class SIM800{
public:
    SIM800(PinName pwr, PinName st);
    void enable();
    void disable();
    void power_key();
	int ATEx(int x);
	int AT_CREG(uint8_t command_type=TEST_CMND, bool debug=true);
	int AT_CSQ(uint8_t command_type=TEST_CMND);
	int AT_SAPBR(uint8_t command_type=TEST_CMND, int cmd_type=0, int cid=0, string con_param_tag="", string con_param_value="");
	int AT_HTTPINIT(uint8_t command_type=TEST_CMND);
	int AT_HTTPTERM(uint8_t command_type=TEST_CMND);
	int AT_HTTPPARA(uint8_t command_type=TEST_CMND, string http_param_tag="", string http_param_value="");
	int AT_HTTPDATA(uint8_t command_type=TEST_CMND, int size=0, int tm=0,  char* data=(char*)"");
	int AT_HTTPACTION(uint8_t command_type=TEST_CMND, int method=0, int timeout = 20000);
	int AT_HTTPREAD(uint8_t command_type=TEST_CMND, int* data_len=0, char* data_buffer=(char*)"", int start_address=0, int byte_size=0);
	int AT_FTPTYPE(uint8_t command_type=TEST_CMND, char value='A');
	int AT_FTPCID(uint8_t command_type=TEST_CMND, int value=0);
	int AT_FTPSERV(uint8_t command_type=TEST_CMND, string value="");
    int AT_FTPPORT(uint8_t command_type=TEST_CMND, string value="");
	int AT_FTPUN(uint8_t command_type=TEST_CMND, string value="");
	int AT_FTPPW(uint8_t command_type=TEST_CMND, string value="");
	int AT_FTPGETNAME(uint8_t command_type=TEST_CMND, string value="");
	int AT_FTPGETPATH(uint8_t command_type=TEST_CMND, string value="");
	int AT_FTPPUTNAME(uint8_t command_type=TEST_CMND, string value="");
	int AT_FTPPUTPATH(uint8_t command_type=TEST_CMND, string value="");
	int AT_FTPSIZE(uint8_t command_type=TEST_CMND, int* size=0);
	int AT_FTPGET(uint8_t command_type=TEST_CMND, int mode=1, int req_length=0, char* data_buffer=(char*)"");
	// int AT_FTPPUT(uint8_t command_type=TEST_CMND, int mode=1, int req_length=0);
	int AT_FTPQUIT(uint8_t command_type=TEST_CMND);
	int AT_FTPDELE(uint8_t command_type=TEST_CMND);
    int AT_CMGD(uint8_t command_type=TEST_CMND, int index=0, int delflag=-1);
    int AT_CMGF(uint8_t command_type=TEST_CMND, int mode=0);
    // int AT_CMGL(uint8_t command_type=TEST_CMND, string stat="", int mode=0);
    int AT_CMGR(uint8_t command_type=TEST_CMND, int index=0);
    int AT_CMGS(uint8_t command_type=TEST_CMND, string phone_number="", string message="");
    int AT_CSMP(uint8_t command_type=TEST_CMND, int fo=0, int vp=0, int pid=0, int dcs=0);
    int AT_CUSD(uint8_t command_type=TEST_CMND, int n=0, string str="");

    void clear_data();
	int string_to_int(string s);
    bool sim_registered;

    int FTP_SIZE = 1360;
	char data[DATA_BUFFER_SIZE];
    DigitalOut power;
    DigitalOut strt;
};

#endif
