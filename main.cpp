#include "mbed.h"
#include <map>

#include "SerialHandler.h"
#include "SIM800.h"
#include "img.h"
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>
#include "calibril_12.h"
#include "calibril_5.h"

TFT_ILI9163C tft(PA_7, PA_6, PA_5, PC_4, PA_3, PA_1);

RawSerial pc(PC_10, PC_11, 115200);
RawSerial serial(PB_6, PB_7, 9600); //Serial_1
SerialHandler ser(0);
SIM800 sim800(PB_12, PB_3);
FlashIAP flash;

DigitalOut led(PB_4, 0);
DigitalOut relay(PA_2, 1);
DigitalIn pin(PA_0, PullUp);
DigitalIn reset(PA_4, PullUp);

int wait_duration = 0;
bool time_set = false;
bool ready = false;
bool reset_ready = false;
string device_id = "cam0018";
string gprs_url = "http://gw.abfascada.ir/anami/getdata.php";
char temp_buffer[3000];
char sms_buffer[100];
char disp[20];
int scan_cnt = 0, data_cnt = 0;
long counter = 0;
float f1 = 0, f2 = 0, flow = 0, lcd_flow = -1;
float volume = 0.0, scale = 0.0, lcd_volume = -1;
float high_th = 0.0, low_th = 0.0;
int data_post_interval = 0, sms_check_interval = 180;
time_t last_post_time = 0, last_sms_time = 0;
bool update_lcd = true;
int pin_scan_interval = 0, flow_interval = 0;

struct info_t{
    time_t timestamp;
    float volume;
    long raw;
    float flow;
    bool state;
};

map<int, info_t> data;

int uint32_to_bytes(uint32_t num, char* buffer){
    for(int i = 0;i < 4;i++){
        buffer[i] = (num >> (i * 8)) & 0xFF; 
    }
    return 0;
}

int bytes_to_uint32(char* buffer, uint32_t* num){
    *num = 0;
    for(int i = 0;i < 4;i++){
        *num += buffer[i] << (i * 8);
    }
    return 0;
}

void save_param_to_flash(){
    flash.init();
    flash.erase(0x801FC00, 1024);
    uint32_t addr = 0x801FFE8;
    uint32_t page_size = 4;
    char buffer[4];

    uint32_to_bytes((uint32_t)(scale * 100), buffer);
    flash.program(buffer, addr, page_size);

    addr += page_size;
    uint32_to_bytes((uint32_t)(high_th * 100), buffer);
    flash.program(buffer, addr, page_size);

    addr += page_size;
    uint32_to_bytes((uint32_t)(low_th * 100), buffer);
    flash.program(buffer, addr, page_size);

    addr += page_size;
    uint32_to_bytes((uint32_t)(data_post_interval), buffer);
    flash.program(buffer, addr, page_size);

    addr += page_size;
    uint32_to_bytes((uint32_t)(pin_scan_interval), buffer);
    flash.program(buffer, addr, page_size);

    addr += page_size;
    uint32_to_bytes((uint32_t)(flow_interval), buffer);
    flash.program(buffer, addr, page_size);

    flash.deinit();
}

void load_param_from_flash(){
    flash.init();
    char buffer[4];
    uint32_t addr = 0x801FFE8;
    uint32_t page_size = 4;
    uint32_t num;

    flash.read(buffer, addr, page_size);
    bytes_to_uint32(buffer, &num);
    scale = (float)num / 100.0;

    addr += page_size;
    flash.read(buffer, addr, page_size);
    bytes_to_uint32(buffer, &num);
    high_th = (float)num / 100.0;

    addr += page_size;
    flash.read(buffer, addr, page_size);
    bytes_to_uint32(buffer, &num);
    low_th = (float)num / 100.0;

    addr += page_size;
    flash.read(buffer, addr, page_size);
    bytes_to_uint32(buffer, &num);
    data_post_interval = num;

    addr += page_size;
    flash.read(buffer, addr, page_size);
    bytes_to_uint32(buffer, &num);
    pin_scan_interval = num;

    addr += page_size;
    flash.read(buffer, addr, page_size);
    bytes_to_uint32(buffer, &num);
    flow_interval = num;
}

void status_update(string mystr){
    int x = 33, y = 144;
    tft.setCursor(x, y);
    tft.fillRect(x,y-6,128,9,BLACK);
    tft.setTextSize(1);
    tft.setFont(&calibril_5);
    tft.setTextColor(YELLOW);
    tft.setCursor(x, y);
    sprintf(disp, "%s", mystr.c_str());
    tft.print(disp);
}

void update_volume(){
    volume = counter * scale;
    if(lcd_volume == volume){
        return;
    }
    lcd_volume = volume;
    tft.fillRect(3,41,89,15,BLACK);
    tft.setFont(&calibril_12);
    tft.setCursor(3, 55);
    tft.setTextColor(WHITE);
    sprintf(disp,"%.2f", volume);
    tft.print(disp);
}

void update_flow(){
    if(lcd_flow == flow){
        return;
    }
    lcd_flow = flow;
    tft.fillRect(3,101,53,15,BLACK);
    tft.setFont(&calibril_12);
    tft.setCursor(3, 115);
    tft.setTextColor(WHITE);
    sprintf(disp,"%.2f", flow);
    tft.print(disp);
}

void init_lcd(){
    tft.begin();
    tft.setBitrate(64000000);
    tft.setRotation(2);
    tft.clearScreen();
    tft.setFont(&calibril_12);

    tft.setTextColor(YELLOW);
    tft.setCursor(5, 25);
    sprintf(disp,"Volume: ");
    tft.print(disp);

    tft.setCursor(99, 55);
    sprintf(disp,"m3");
    tft.print(disp);

    tft.setTextColor(YELLOW);
    tft.setCursor(5, 85);
    sprintf(disp,"Flow: ");
    tft.print(disp);

    tft.setCursor(77, 115);
    sprintf(disp,"m3/h");
    tft.print(disp);

    tft.drawRGBBitmap(0, 130, alt_wide, 160, 38);
    update_volume();
    update_flow();
}

void blink(){
    led = !led;
}

void scan_pin(){
    scan_cnt++;

    if (pin == 1){
        ready = 1;
    }

    if (reset == 1){
        reset_ready = 1;
    }

    if((!pin) && ready){
        counter++;
        ready = 0;
    }

    if((!reset) && reset_ready){
        counter = 0;
        volume = 0.0;
        flow = 0.0;
        f1 = 0.0;
        f2 = 0.0;
        relay = 1;
        update_volume();
        update_flow();
        reset_ready = 0;
    }

    if (scan_cnt == (int)((flow_interval * 1000) / pin_scan_interval)) {
        volume = counter * scale;
        f1 = f2;
        f2 = volume;
        flow = (f2 - f1) * (3600 / flow_interval);
        scan_cnt = 0;

        if(flow < low_th || flow > high_th){
            relay = 1;
        }
        else{
            relay = 0;
        }

        info_t info;
        info.flow = flow;
        info.raw = counter;
        info.volume = volume;
        info.state = !relay;
        info.timestamp = time(NULL);

        data[data_cnt] = info;
        data_cnt++;

        update_lcd = true;
    }

    if (scan_cnt % 10 == 0) {
        blink();
    }
}

void custom_wait(int duration){
    for(int i = 0;i < duration;i++){
        wait_duration++;
        wait_us(1);
        if(wait_duration == pin_scan_interval * 1000){
            scan_pin();
            wait_duration = 0;
        }
    }
}

int check_sim800(){
    pc.printf("\r\n\r\nFrom check_sim:\r\n");
    ser.sendCmd((char*)"\r");
    sim800.AT_CREG(READ_CMND, true);
    string data(sim800.data);
    if(data.compare("0,1") == 0 || data.compare("0,5") == 0){
        sim800.ATEx(0);
        return 0;
    }
    int retries = 0, r = 0;
    while(retries < 3){
        Watchdog::get_instance().kick();
        pc.printf("Registering sim800 on the network...");
        sim800.disable();
        custom_wait(100000);
        sim800.enable();
        custom_wait(100000);
        sim800.power_key();
        while(r < 20){
            Watchdog::get_instance().kick();
            sim800.AT_CREG(READ_CMND, false);
            string data(sim800.data);
            if(data.compare("0,1") == 0 || data.compare("0,5") == 0){
                pc.printf("Done\r\n");
                sim800.ATEx(0);
                sim800.sim_registered = true;
                custom_wait(1000000);
                return 0;
            }
            r++;
            custom_wait(1000000);
        }
        pc.printf("Failed!\r\n");
        r = 0;
        retries++;
    }
    return -1;
}

int string_to_int(string str, int* d){
    if(str.length() == 0){
        return -2;
    }
    int indx = str.find('.');
    if (indx != -1) {
        str = str.substr(0, indx);
    }
    int neg = 1;
    if (str.find('-') == 0) {
        neg = -1;
        str = str.substr(1);
    }

    double t = 0;
    int l = str.length();
    for(int i = l-1; i >= 0; i--){
        if(!isdigit(str[i])){
            return -1;
        }
        t += (str[i] - '0') * pow(10.0, l - i - 1);
    }
    *d = (int)(neg * t);
    return 0;
}

void get_time(){
    pc.printf("\r\n\r\nFrom get_time\r\n");
    status_update("Sync time");
    int retries = 3, result;
    int data_len = 0;
    while(retries > 0){
        Watchdog::get_instance().kick();
        if(check_sim800() != 0){
            pc.printf("Could not register SIM800 on network");
            time_set = false;
            set_time(0);
            return;
        }
        Watchdog::get_instance().kick();
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "Contype", "GPRS");
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "APN", "www");
        sim800.AT_HTTPINIT(EXEC_CMND);
        sim800.AT_HTTPPARA(WRITE_CMND, "CID", "1");
        char temp[60];
        sprintf(temp, "http://gw.abfascada.ir/ahv_rtu/settings.php?co=%s", device_id.c_str());
        sim800.AT_HTTPPARA(WRITE_CMND, "URL", temp);
        custom_wait(100000);
        result = sim800.AT_SAPBR(WRITE_CMND, 1, 1);
        if(result != 0){
            retries--;
            sim800.disable();
            continue;
        }
        custom_wait(100000);
        result = sim800.AT_HTTPACTION(WRITE_CMND, 0, 20000);
        printf("action data: %s\r\n", sim800.data);
        if(result != 0){
            retries--;
            sim800.disable();
            continue;
        }

        string temp_data(sim800.data);
        int idx = temp_data.find(",");
        temp_data = temp_data.substr(idx+1);

        idx = temp_data.find(",");
        string parse = temp_data.substr(0, idx).c_str();
        int result_code;
        string_to_int(parse, &result_code);

        printf("result = %d\r\n", result_code);
        if(result_code != 200){
            retries--;
            sim800.disable();
            continue;
        }

        result = sim800.AT_HTTPREAD(EXEC_CMND, &data_len, temp_buffer);
        if(result != 0){
            retries--;
            sim800.disable();
            continue;
        }
        break;
    }
    if(retries <= 0){
        time_set = false;
        set_time(0);
        return;
    }
    temp_buffer[data_len] = '\0';
    string temp_data(temp_buffer);
    for(int i = 0;i < 4;i++){
        int idx = temp_data.find("%");
        temp_data = temp_data.substr(idx+1);
    }
    int idx = temp_data.find("%");
    temp_data = temp_data.substr(0, idx).c_str();
    pc.printf("timestamp = %s\r\n", temp_data.c_str());
    int tm;
    string_to_int(temp_data, &tm);
    set_time(tm);
    time_set = true;
    custom_wait(100000);
    sim800.AT_HTTPTERM(EXEC_CMND);
    custom_wait(100000);
    sim800.AT_SAPBR(WRITE_CMND, 0, 1);
}

void post_data(){
    Watchdog::get_instance().kick();
    pc.printf("\r\n\r\nFrom post_data:\r\n");
    last_post_time = time(NULL);
    if(data_cnt == 0){
        pc.printf("No data Available\r\n");
        return;
    }
    status_update("Sending data to server");
    custom_wait(100000);
    sprintf(temp_buffer, "{\"device_id\":\"%s\",\"data_cnt\":\"%d\",\"scale\":\"%.2f\",\"high_th\":\"%.2f\",\"low_th\":\"%.2f\",\"data\":[", device_id.c_str(), data_cnt, scale, high_th, low_th);
    bool first_data = true;

    for(int i = 0;i < data_cnt;i++){
        if(!first_data){
            sprintf(temp_buffer, "%s,", temp_buffer);
        }
        sprintf(temp_buffer, "%s{\"raw\":\"%ld\",\"volume\":\"%.2f\",\"flow\":\"%.2f\",\"relay_state\":\"%d\", \"ts\":\"%d\"}", temp_buffer, data[i].raw, data[i].volume, data[i].flow, data[i].state, data[i].timestamp);
        first_data = false;
    }
    int sz = sprintf(temp_buffer, "%s]}", temp_buffer);
    pc.printf("post size = %d, data = %s", sz, temp_buffer);
    int retries = 3;
    while(retries > 0){
        Watchdog::get_instance().kick();
        if(check_sim800() != 0){
            pc.printf("Could not register SIM800 on network");
            return;
        }
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "Contype", "GPRS");
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "APN", "www");
        sim800.AT_HTTPINIT(EXEC_CMND);
        sim800.AT_HTTPPARA(WRITE_CMND, "CID", "1");
        sim800.AT_HTTPPARA(WRITE_CMND, "URL", gprs_url);
        sim800.AT_HTTPPARA(WRITE_CMND, "CONTENT", "application/x-www-form-urlencoded");
        custom_wait(100000);
        if(sim800.AT_SAPBR(WRITE_CMND, 1, 1) != 0){
            retries--;
            sim800.disable();
            continue;
        }

        if(sim800.AT_HTTPDATA(WRITE_CMND, sz, 5000, temp_buffer) != 0){
            retries--;
            sim800.disable();
            continue;
        }
        if(sim800.AT_HTTPACTION(WRITE_CMND, 1, 120000) != 0){
            retries--;
            sim800.disable();
            continue;
        }
        string temp_data(sim800.data);
        int idx = temp_data.find(",");
        temp_data = temp_data.substr(idx+1);

        idx = temp_data.find(",");
        string parse = temp_data.substr(0, idx).c_str();
        int result_code;
        string_to_int(parse, &result_code);

        printf("result = %d\r\n", result_code);
        if(result_code != 200){
            retries--;
            sim800.disable();
            continue;
        }
        data.clear();
        data_cnt = 0;
        // int data_len;
        // if(sim800.AT_HTTPREAD(EXEC_CMND, &data_len, temp_buffer) != 0){
        //     retries--;
        //     sim800.disable();
        //     continue;
        // }
        sim800.AT_HTTPTERM(EXEC_CMND);
        sim800.AT_SAPBR(WRITE_CMND, 0, 1);
        // pc.printf("data len = %d\r\n", data_len);
        // for(int i = 0;i < data_len; i++){
        //     pc.printf("%02x", temp_buffer[i]);
        // }
        break;
    }
}

int string_to_double(string s, double* d){
    if(s.length() == 0){
        return -2;
    }
    int indx = s.find('.');
    string f;
    if (indx != -1) {
        f = s.substr(indx+1);
        s = s.substr(0, indx);
    }
    int neg = 1;
    if (s.find('-') != -1) {
        neg = -1;
        s = s.substr(1);
    }
    else if(s.find('+') != -1){
        neg = 1;
        s = s.substr(1);
    }

    double t = 0;
    int l = s.length();
    for(int i = l-1; i >= 0; i--){
        if(!isdigit(s[i])){
            return -1;
        }
        t += (s[i] - '0') * pow(10.0, l - i - 1);
    }
    l = f.length();
    for(int i = 0; i < l; i++){
        if(!isdigit(f[i])){
            return -1;
        }
        t += (f[i] - '0') * pow(10.0, -1 * (i+1));
    }
    *d = neg * t;
    return 0;
}

int get_value(string data, float* f){
    int idx = data.find(":");
    if(idx == -1){
        return -1;
    }
    int idx2 = data.find("$");
    if(idx2 == -1){
        return -1;
    }
    double d;
    if(string_to_double(data.substr(idx+1, idx2-(idx + 1)), &d) != 0){
        return -1;
    }
    *f = (float)d;
    return 0;
}

void print_param(){
    pc.printf("scale: %.2f\nhigh_th: %.2f\nlow_th: %.2f\ndata_post_interval: %d\npin_scan_interval: %d\nflow_interval: %d\n", scale, high_th, low_th, data_post_interval, pin_scan_interval, flow_interval);
}

void check_for_sms(){
    last_sms_time = time(NULL);
    pc.printf("\r\n\r\nCheck for sms\r\n");
    status_update("Check sms command");
    if(check_sim800() != 0){
        pc.printf("Could not register SIM800 on network");
        return;
    }
    sim800.AT_CMGF(WRITE_CMND, 1);
    
    for(int i=1;i<16;i++){
        bool send_param_sms = false;
        bool send_settings_sms = false;
        Watchdog::get_instance().kick();
        if(sim800.AT_CMGR(WRITE_CMND, i) != 0){
            return;
        }
        string output(sim800.data);
        remove(output.begin(), output.end(), '\r');
        remove(output.begin(), output.end(), '\n');
        for (size_t j = 0; j < 3; j++) {
            int u = output.find("\"");
            output = output.substr(u+1);
        }
        int u = output.find("\"");
        string phone_number = output.substr(0, u);
        for (size_t j = 0; j < 5; j++) {
            int u = output.find("\"");
            output = output.substr(u+1);
        }
        u = output.find("OK");
        output = output.substr(0, u);
        pc.printf("sms: %s, from %s\n", output.c_str(), phone_number.c_str());
        sim800.AT_CMGD(WRITE_CMND, i);
        if(output.find("#stat") == 0){
            sprintf(sms_buffer, "raw=%ld,volume=%.2f,flow=%.2f,state=%d", counter, volume, flow, !relay.read());
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sms_buffer);
        }
        else if(output.find("#gp") == 0){
            post_data();
        }
        else if(output.find("#qu") == 0){
            sim800.AT_CSQ(EXEC_CMND);
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sim800.data);
        }
        else if(output.find("#reset") == 0){
            NVIC_SystemReset();
        }
        else if(output.find("#balance") == 0){
            sim800.AT_CUSD(WRITE_CMND, 1);
            sim800.AT_CUSD(WRITE_CMND, 1, "*555*4*3*2#");
            sim800.AT_CUSD(WRITE_CMND, 1, "*555*1*2#");
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sim800.data);
        }
        else if(output.find("#set_scale") == 0){
            float f;
            if(get_value(output, &f) != 0){
                return;
            }
            scale = f;
            save_param_to_flash();
            print_param();
            send_param_sms = true;
        }
        else if(output.find("#set_high_th") == 0){
            float f;
            if(get_value(output, &f) != 0){
                return;
            }
            high_th = f;
            save_param_to_flash();
            print_param();
            send_param_sms = true;
        }
        else if(output.find("#set_low_th") == 0){
            float f;
            if(get_value(output, &f) != 0){
                return;
            }
            high_th = f;
            save_param_to_flash();
            print_param();
            send_param_sms = true;
        }
        else if(output.find("#set_data_interval") == 0){
            float f;
            if(get_value(output, &f) != 0){
                return;
            }
            data_post_interval = (int)f;
            save_param_to_flash();
            print_param();
            send_settings_sms = true;
        }
        else if(output.find("#set_pin_scan_interval") == 0){
            float f;
            if(get_value(output, &f) != 0){
                return;
            }
            pin_scan_interval = (int)f;
            save_param_to_flash();
            print_param();
            send_settings_sms = true;
        }
        else if(output.find("#set_flow_interval") == 0){
            float f;
            if(get_value(output, &f) != 0){
                return;
            }
            if( f > 0){
                flow_interval = (int)f;
                save_param_to_flash();
            }
            print_param();
            send_settings_sms = true;
        }
        if(send_param_sms){
            sprintf(sms_buffer, "scale=%.2f,high_th=%.2f,low_th=%.2f", scale, high_th, low_th);
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sms_buffer);
        }
        if(send_settings_sms){
            sprintf(sms_buffer, "pin_scan_interval=%d(ms),flow_interval=%d(s),data_interval=%d(m)", pin_scan_interval, flow_interval, data_post_interval);
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sms_buffer);
        }
    }
}

void write_default_param_to_flash(){
    scale = 0.1;
    low_th = 15.0;
    high_th = 70.0;
    data_post_interval = 3;
    pin_scan_interval = 20;
    flow_interval = 180;
    save_param_to_flash();
    pc.printf("done\r\n");
}

int main(){
    // write_default_param_to_flash();
    // return 0;
    Watchdog::get_instance().start();
    init_lcd();
    status_update("Initializing...");
    load_param_from_flash();
    print_param();
    if(serial.readable()){
        while(serial.readable()){
            serial.getc();
        }
    }
    serial.attach(callback(&ser, &SerialHandler::rx));
    
    get_time();
    while(true){
        Watchdog::get_instance().kick();
        time_t now = time(NULL);
        if(update_lcd){
            update_flow();
            update_lcd = false;
        }
        if(!time_set){
            get_time();
        }
        if(now > last_post_time + (data_post_interval * 60)){
            post_data();
        }
        if(now > last_sms_time + sms_check_interval){
            check_for_sms();
        }
        update_volume();
        status_update("Calculating flow");
        custom_wait(10000000);
    }
}

