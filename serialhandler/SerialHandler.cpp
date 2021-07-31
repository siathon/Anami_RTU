#include "SerialHandler.h"

SerialHandler::SerialHandler(int t){}

void SerialHandler::rx(void){
    c = serial.getc();
	raw_data[raw_received_len] = c;
	raw_received_len++;
    if (gotExpectedOutput && gotExpectedData) {
        return;
    }
    else if (!gotExpectedOutput) {
		// if(c == '\r' || c == '\n'){
		// 	c = '|';
		// }
		// if(c == '|' && receivedOutput[receivedOutputLen] == '|'){
		// 	return;
		// }
        if (expectedOutput[receivedOutputLen] == c) {
            receivedOutput[receivedOutputLen] = c;
            receivedOutputLen++;
        }
        gotExpectedOutput = expectedOutputLen == receivedOutputLen;
    }
    else if (!gotExpectedData) {
        receivedData[receivedDataLen] = c;
        receivedDataLen++;
        gotExpectedData = receivedDataLen == expectedDataLen;
    }
}

void SerialHandler::setReceiveParam(string output, int dataLen){
	for(int i = 0;i < RECEIVED_DATA_BUFFER_SIZE;i++){
		receivedData[i] = '\0';
	}
	for(int i = 0;i < RECEIVED_OUTPUT_BUFFER_SIZE;i++){
		receivedOutput[i] = '\0';
	}
	for(int i = 0;i < RECEIVED_RAW_DATA_BUFFER_SIZE;i++){
		raw_data[i] = '\0';
	}
	sprintf(expectedOutput, "%s", output.c_str());
    expectedDataLen = dataLen;
    receivedOutputLen = 0;
    receivedDataLen = 0;
	raw_received_len = 0;
    expectedOutputLen = output.length();
    gotExpectedOutput = false;
    gotExpectedData = false;
    if (expectedOutputLen == 0) {
        gotExpectedOutput = true;
    }
    if (expectedDataLen == 0) {
        gotExpectedData = true;
    }
}

void SerialHandler::sendCmd(char *cmd){
    serial.puts(cmd);
}

int SerialHandler::sendCmdAndWaitForResp(char* cmd, string resp, int dataSize, int timeout){
    setReceiveParam(resp, dataSize);
    sendCmd(cmd);
    while (!gotExpectedData || !gotExpectedOutput){
        Watchdog::get_instance().kick();
        timeout -= 1;
        wait_us(1000);
        if (timeout == 0) {
            if (gotExpectedOutput && dataSize == -1) {
                return 0;
            }
            return -1;
        }
    }
    return 0;
}

void SerialHandler::print_error(){
	raw_data[raw_received_len] = '\0';
	printf("raw output: %s\n", raw_data);
}

void SerialHandler::print_output(){
	printf("Output: %s\n", receivedOutput);
}

void SerialHandler::read_data(char* buffer, bool debug){
	if(debug){
		printf("Data: ");
	}
	for(int i = 0;i<receivedDataLen;i++){
		buffer[i] = receivedData[i];
		if(debug){
			printf("%c", buffer[i]);
		}
	}
	if(debug){
		printf("\n");
	}
	buffer[receivedDataLen] = '\0';
}