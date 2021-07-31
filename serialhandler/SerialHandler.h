#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#define RECEIVED_DATA_BUFFER_SIZE 1500
#define RECEIVED_OUTPUT_BUFFER_SIZE 150
#define RECEIVED_RAW_DATA_BUFFER_SIZE 2000
#include "mbed.h"
#include "events/EventQueue.h"
#include <string>

extern RawSerial serial;
class SerialHandler{
public:
    SerialHandler(int t);
    void rx();
    void setReceiveParam(string output, int dataLen);
    void sendCmd(char *cmd);
    int sendCmdAndWaitForResp(char* cmd, string resp, int dataSize, int timeout);
	void print_error();
	void print_output();
	void print_data();
	void read_data(char* buffer, bool debug=true);
    bool gotExpectedData;
    bool gotExpectedOutput;

	char raw_data[RECEIVED_RAW_DATA_BUFFER_SIZE];
    char receivedData[RECEIVED_DATA_BUFFER_SIZE];
    char receivedOutput[RECEIVED_OUTPUT_BUFFER_SIZE];
    char expectedOutput[RECEIVED_OUTPUT_BUFFER_SIZE];

	int raw_received_len;
    int expectedOutputLen;
    int receivedOutputLen;
    int expectedDataLen;
    int receivedDataLen;

    char c;
};

#endif
