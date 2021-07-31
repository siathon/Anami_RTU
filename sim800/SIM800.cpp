#include "SIM800.h"

SIM800::SIM800(PinName pwr, PinName st):power(pwr, 0), strt(st, 1){
    sim_registered = false;
}

void SIM800::enable(){
    power = 1;
}

void SIM800::disable(){
    sim_registered = false;
    power = 0;
}

void SIM800::power_key(){
    strt = 0;
    custom_wait(500000);
    strt = 1;
}

int SIM800::ATEx(int x){
	char cmnd[10];
	sprintf(cmnd, "ATE%d\r", x);
	pc.printf("%s\n", cmnd);
	if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
		pc.printf("%s\n", ser.receivedOutput);
		return 0;
	}
	else{
		data[0] = '\0';
		ser.print_error();
		return -1;
	}
}

int SIM800::AT_CREG(uint8_t command_type, bool debug){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CREG=?\r");
			if(debug){
				pc.printf("%s\n", cmnd);
			}
			
			if(ser.sendCmdAndWaitForResp(cmnd, "+CREG: ", 5, 1000) == 0){
				if(debug){
					ser.print_output();
				}
				clear_data();
				ser.read_data(data, debug);
				return 0;
			}
			else{
				if(debug){
					data[0] = '\0';
		        ser.print_error();
				}
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+CREG?\r");
			if(debug){
				pc.printf("%s\n", cmnd);
			}
			if(ser.sendCmdAndWaitForResp(cmnd, "+CREG: ", 3, 1000) == 0){
				if(debug){
					ser.print_output();
				}
                
				clear_data();
				ser.read_data(data, debug);
				return 0;
			}
			else{
				if(debug){
					data[0] = '\0';
		        ser.print_error();
				}
				return -1;
			}
		}
		case WRITE_CMND:
			pc.printf("Command not impelemented");
			return -2;
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_CSQ(uint8_t command_type){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CSQ=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CSQ: ", 18, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case EXEC_CMND:{
			sprintf(cmnd, "AT+CSQ\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CSQ: ", 5, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_SAPBR(uint8_t command_type, int cmd_type, int cid, string con_param_tag, string con_param_value){
	char cmnd[150];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+SAPBR=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+SAPBR: ", 42, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			if(con_param_tag.compare("") == 0){
				sprintf(cmnd, "AT+SAPBR=%d,%d\r", cmd_type, cid);
			}
			else{
				sprintf(cmnd, "AT+SAPBR=%d,%d,%s,%s\r", cmd_type, cid, con_param_tag.c_str(), con_param_value.c_str());
			}
			pc.printf("%s\n", cmnd);
			switch(cmd_type){
				case 0:
				case 1:
				case 3:{
					if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 85000) == 0){
						ser.print_output();
						return 0;
					}
					else{
						data[0] = '\0';
		        ser.print_error();
						return -1;
					}
				}
				case 4:
				case 2:{
					if(ser.sendCmdAndWaitForResp(cmnd, "+SAPBR:", -1, 1000) == 0){
						ser.print_output();
						clear_data();
						ser.read_data(data);
						return 0;
					}
					else{
						data[0] = '\0';
		        ser.print_error();
						return -1;
					}
				}
				default:
					pc.printf("Invalid cmd_typ\n");
					return -4;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_HTTPINIT(uint8_t command_type){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+HTTPINIT=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case EXEC_CMND:{
			sprintf(cmnd, "AT+HTTPINIT\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_HTTPTERM(uint8_t command_type){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+HTTPTERM=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case EXEC_CMND:{
			sprintf(cmnd, "AT+HTTPTERM\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_HTTPPARA(uint8_t command_type, string http_param_tag, string http_param_value){
	char cmnd[150];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+HTTPPARA=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+HTTPPARA: \"HTTPParamTag\",\"HTTPParmValue\"\r\nOK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+HTTPPARA?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+HTTPPARA:", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+HTTPPARA=%s,%s\r", http_param_tag.c_str(), http_param_value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_HTTPDATA(uint8_t command_type, int size, int tm, char* data_buffer){
	char cmnd[150];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+HTTPDATA=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+HTTPDATA: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+HTTPDATA=%d,%d\r", size, tm);
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "DOWNLOAD", 0, 1000) == 0){
				ser.print_output();
				sprintf(data_buffer, "%s\r", data_buffer);
				if(ser.sendCmdAndWaitForResp(data_buffer, "OK", 0, 1000) == 0){
					ser.print_output();
					return 0;
				}
				else{
					data[0] = '\0';
		ser.print_error();
					return -1;
				}
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_HTTPACTION(uint8_t command_type, int method, int timeout){
	char cmnd[20];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+HTTPACTION=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+HTTPACTION: ", 5, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+HTTPACTION=%d\r", method);
			pc.printf("%s\n", cmnd);
            ser.raw_data[0] = '\0';
			if(ser.sendCmdAndWaitForResp(cmnd, "OK\r\n+HTTPACTION: ", 5, timeout) == 0){
				ser.print_output();
                custom_wait(100000);
                string temp_raw(ser.raw_data);
                int idx = temp_raw.find("+HTTPACTION");
                temp_raw = temp_raw.substr(idx+13);
                int idx2 = temp_raw.find("\r\n");
                for(int i = 0;i < idx2+2;i++){
                    data[i] = ser.raw_data[idx+13+i];
                }
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_HTTPREAD(uint8_t command_type, int* data_len, char* data_buffer, int start_address, int byte_size){
	char cmnd[50];
    char rspns[50];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+HTTPREAD=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+HTTPREAD: ", -1, 1000) == 0){
				ser.print_output();
                
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+HTTPREAD=%d,%d\r", start_address, byte_size);
            sprintf(rspns, "+HTTPREAD: %d\r\n", byte_size);
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, rspns, byte_size, 5000) == 0){
				clear_data();
				ser.read_data(data_buffer, false);
				ser.print_output();
				pc.printf("OK\n");
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case EXEC_CMND:{
			sprintf(cmnd, "AT+HTTPREAD\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+HTTPREAD: ", -1, 5000) == 0){
				clear_data();
				ser.read_data(data);
				string temp(data);
				int idx = temp.find("\r\n");
				string len = temp.substr(0, idx);
				int _data_len = string_to_int(len);
				*data_len = _data_len;
				for(int i = 0; i < _data_len; i++){
					data_buffer[i] = data[i+idx+2];
				}
				ser.print_output();
				pc.printf("OK\n");
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPTYPE(uint8_t command_type, char value){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPTYPE=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPTYPE?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPTYPE: ", 1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPTYPE=%c\r", value);
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPCID(uint8_t command_type, int value){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPCID=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPCID?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPCID: ", 1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPCID=%d\r", value);
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPSERV(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPSERV=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPSERV?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPSERV: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPSERV=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPPORT(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPPORT=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPPORT?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPPORT: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPPORT=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPUN(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPUN=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPUN?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPUN: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPUN=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPPW(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPPW=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPPW?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPPW: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPPW=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPGETNAME(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPGETNAME=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPGETNAME?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPGETNAME: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPGETNAME=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPGETPATH(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPGETPATH=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPGETPATH?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPGETPATH: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPGETPATH=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPPUTNAME(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPPUTNAME=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPPUTNAME?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPPUTNAME: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPPUTNAME=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPPUTPATH(uint8_t command_type, string value){
	char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPPUTPATH=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+FTPPUTPATH?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPPUTPATH: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+FTPPUTPATH=%s\r", value.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPSIZE(uint8_t command_type, int* size){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPSIZE=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case EXEC_CMND:{
			sprintf(cmnd, "AT+FTPSIZE\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+FTPSIZE:1,0,", -1, 10000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				string temp(data);
				int idx = temp.find("\r\n");
				*size = string_to_int(temp.substr(0, idx));
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPGET(uint8_t command_type, int mode, int req_length, char* data_buffer){
	char cmnd[30];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPGET=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			
			switch(mode){
				case 1:{
					sprintf(cmnd, "AT+FTPGET=1\r");
					pc.printf("%s\n", cmnd);
					if(ser.sendCmdAndWaitForResp(cmnd, "FTPGET: 1,1", 0, 10000) == 0){
						ser.print_output();
						return 0;
					}
					else{
						data[0] = '\0';
		                ser.print_error();
						return -1;
					}
				}
				case 2:{
					char rspns[30];
					sprintf(cmnd, "AT+FTPGET=2,%d\r", req_length);
					sprintf(rspns, "+FTPGET: 2,%d\r\n", req_length);
					pc.printf("%s\n", cmnd);
					if(ser.sendCmdAndWaitForResp(cmnd, rspns, req_length, 5000) == 0){
						ser.print_output();
						clear_data();
						ser.read_data(data_buffer, false);
						pc.printf("OK\n");
						return 0;
					}
					else{
						data[0] = '\0';
		                ser.print_error();
						return -1;
					}
				}
				default:
					pc.printf("Invalid cmd_typ\n");
					return -4;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPQUIT(uint8_t command_type){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPQUIT=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case EXEC_CMND:{
			sprintf(cmnd, "AT+FTPQUIT\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_FTPDELE(uint8_t command_type){
	char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+FTPDELE=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case EXEC_CMND:{
			sprintf(cmnd, "AT+FTPDELE\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK\r\n+FTPDELE:1,0", 0, 5000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_CMGD(uint8_t command_type, int index, int delflag){
    char cmnd[20];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CMGD=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CMGD: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
            if(delflag == -1){
                sprintf(cmnd, "AT+CMGD=%d\r", index);
            }
            else{
                sprintf(cmnd, "AT+CMGD=%d,%d\r", index, delflag);
            }
			
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 25000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_CMGF(uint8_t command_type, int mode){
    char cmnd[15];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CMGF=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CMGF:", -1, 1000) == 0){
				ser.print_output();
                clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+CMGF?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CMGF: ", 1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+CMGF=%d\r", mode);
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_CMGR(uint8_t command_type, int index){
    char cmnd[20];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CMGR=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+CMGR=%d\r", index);
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CMGR: ", -1, 5000) == 0){
				ser.print_output();
                clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_CMGS(uint8_t command_type, string phone_number, string message){
    char cmnd[30];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CMGS=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+CMGS=\"%s\"\r", phone_number.c_str());
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "> ", 0, 5000) == 0){
                char text[300];
                sprintf(text, "%s%c", message.c_str(), 26);
                pc.printf("%s\n", text);
				if(ser.sendCmdAndWaitForResp(text, "+CMGS: ", -1, 10000) == 0){
                    ser.print_output();
                    clear_data();
				    ser.read_data(data);
                    return 0;
                }
                else{
                    data[0] = '\0';
		ser.print_error();
				    return -2;
                }
			}
			else{
				data[0] = '\0';
		ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_CSMP(uint8_t command_type, int fo, int vp, int pid, int dcs){
    char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CSMP=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CSMP: ", -1, 1000) == 0){
				ser.print_output();
                clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+CSMP?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CSMP: ", -1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
			sprintf(cmnd, "AT+CSMP=%d,%d,%d,%d\r", fo, vp, pid, dcs);
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "OK", 0, 1000) == 0){
				ser.print_output();
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

int SIM800::AT_CUSD(uint8_t command_type, int n, string str){
    char cmnd[100];
	switch(command_type){
		case TEST_CMND:{
			sprintf(cmnd, "AT+CUSD=?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CUSD: ", -1, 1000) == 0){
				ser.print_output();
                clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case READ_CMND:{
			sprintf(cmnd, "AT+CUSD?\r");
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CUSD: ", 1, 1000) == 0){
				ser.print_output();
				clear_data();
				ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		case WRITE_CMND:{
            if(str.length() == 0){
                sprintf(cmnd, "AT+CUSD=%d\r", n);
            }
            else{
                sprintf(cmnd, "AT+CUSD=%d,\"%s\"\r", n, str.c_str());
            }
			pc.printf("%s\n", cmnd);
			if(ser.sendCmdAndWaitForResp(cmnd, "+CUSD: ", -1, 10000) == 0){
				ser.print_output();
                clear_data();
                ser.read_data(data);
				return 0;
			}
			else{
				data[0] = '\0';
		        ser.print_error();
				return -1;
			}
		}
		default:
			pc.printf("Invalid command type");
			return -3;
	}
}

void SIM800::clear_data(){
    for(int i = 0;i < DATA_BUFFER_SIZE;i++){
        data[i] = '\0';
    }
}

int SIM800::string_to_int(string s){
    int indx = s.find('.');
    if (indx != -1) {
        s = s.substr(0, indx);
    }
    int neg = 1;
    if (s.find('-') != -1) {
        neg = -1;
        s = s.substr(1);
    }

    double t = 0;
    int l = s.length();
    for(int i = l-1; i >= 0; i--)
        t += (s[i] - '0') * pow(10.0, l - i - 1);
    return (int)(neg * t);
}

