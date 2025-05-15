// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       shoot_clock.ino
    Created:	13/9/2023 8:00:10 πμ
    Author:     ROUSIS_FACTORY\user
*/

#define SEND_TO_SLAVE 1
#define INVERT_DISPLAY false 
#define DISPLAY_DIGITS 2
#define DEVICE_ID 2     // 1 = Console,  2 = Master display, 3 = master/Slave
#define BUZZER 15
#define SHOOT_TIME_A 30 //25
#define SHOOT_TIME_B 20 //15 for new Polo
#define LED 13

#include "IR_rousis_keys.h"
#include <IRrecv.h>
#include <IRutils.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Rousis7segment.h>
#include "Chrono.h"
#include <EEPROM.h>

//  ----- EEProme addresses -----
//EEPROM data ADDRESSER
#define EEP_TIME_A 0 //1 byte
#define EEP_TIME_B 1 //1 byte

const char device_id[] = { "Shoot_clock_M 24"};
bool slave = false;
bool set_on = false; 
bool cursor_on = false;
uint8_t cursor_index = 0;
uint8_t IR_key_timeout = 0;
bool IR_key_set;
bool Display_on = true;

#define PHOTO_SAMPLES 60
#define PHOTO_SAMPLE_TIME 3000
const int photoPin = 36;
uint8_t photoValue = 0;
uint8_t brightness = 255;
uint8_t sample_metter = 0;
uint16_t page_center = 0;
uint8_t brigthsamples[PHOTO_SAMPLES];
unsigned long photo_delay = 0;

Rousis7segment myLED(DISPLAY_DIGITS, 33, 26, 27, 14);    // Uncomment if not using OE pin

uint8_t brightness_out = 0xFF;
uint8_t buzzer_cnt = 0;
uint8_t flash_cnt = 0;

uint16_t countdown_time;
int minutes = 0;
int seconds = 0; 

unsigned int compare_elapsed = 10000;
char Display_buf[9] = { "00000000" };
char Set_buffer[3];

//Maxairidis  IR remote header 0xAB 0xFF
#define HEADER_A 0xC1
#define HEADER_B 0xA1
#define RXD2 16
#define TXD2 17

#define REMOTE_D0 23
#define  REMOTE_D1 19
#define  REMOTE_D2 18
#define  REMOTE_D3 22

#define INSTR_24 0xC1A1
#define INSTR_START 0xC1A3
#define INSTR_14 0xC1A2
#define INSTR_STOP 0xC1A4
#define INSTR_REPLAY 0xA1A1
#define INSTR_ID_REQ 0xA1A3
#define INSTR_BRIGHTNESS 0xA1A4

#define INSTR_1 0xA151
#define INSTR_2 0xA152
#define INSTR_3 0xA153
#define INSTR_4 0xA154

Chrono chrono(Chrono::SECONDS);
//-----------------------------------------------------------------------------
// MAC address 88:13:BF:1C:27:EC
uint8_t broadcastAddress1[] = { 0x88, 0x13, 0xBF, 0x1C, 0x27, 0xEC };
esp_err_t result;
uint8_t received_arr[32] = { 0 };
uint8_t device_addr = DEVICE_ID; // 1 = Console,  2 = Master display, 3 = master/Slave
//-----------------------------------------------------------------------------
const uint16_t kRecvPin = 35;
IRrecv irrecv(kRecvPin);
decode_results results;
//-------------------------------------------------------------------------------
typedef struct struct_packet {
    char time[9];
    uint8_t alarm;
    uint8_t id;
    uint16_t insrtuction;
    char device[16];
    char display[32];
    uint8_t buzzer_time;
    uint8_t brightness;
} struct_packet;

typedef struct struct_console {
    uint16_t insrtuction;
	uint8_t id;
} struct_console;


struct_packet packet_to_slave;
struct_packet packet_to_console;
struct_packet display_received;
struct_console console_received;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    /*if (status == 0) {
        success = "Delivery Success :)";
    }
    else {
        success = "Delivery Fail :(";
    }*/
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&console_received, incomingData, sizeof(console_received));
 
    Serial.print("Instruction WiFi received: ");
    Serial.println(console_received.insrtuction, HEX);
    
    bool timer_status = chrono.isRunning();

    if (console_received.id == 2)
    {
        slave = true;
        myLED.print(display_received.time, INVERT_DISPLAY);
        memcpy(Display_buf, display_received.time, 2);
        if (display_received.alarm)
        {
            digitalWrite(BUZZER, HIGH);
            buzzer_cnt = display_received.alarm;
        }
    }
    else if (console_received.id == 1)
    {
        switch (console_received.insrtuction)
        {
        case INSTR_1 : //case INSTR_24:
            countdown_time = EEPROM.read(EEP_TIME_A); // SHOOT_TIME_A;
            chrono.restart();
            if (!timer_status)
            {
                chrono.stop();
            }


			compare_elapsed = 10000;
            //timer_to_display();
            //myLED.print(Display_buf, INVERT_DISPLAY);
#if SEND_TO_SLAVE
            memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
#endif // SEND_TO_SLAVE
            break;
        case INSTR_2: //INSTR_14:
            countdown_time = EEPROM.read(EEP_TIME_B); // SHOOT_TIME_B;
            chrono.restart();
            if (!timer_status)
            {
                chrono.stop();
            }

compare_elapsed = 10000;
//timer_to_display();
//myLED.print(Display_buf, INVERT_DISPLAY);
#if SEND_TO_SLAVE
memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
#endif // SEND_TO_SLAVE            
break;
        case INSTR_3: // INSTR_START:
            if (!timer_status)
            {
                chrono.resume();
            }
            break;
        case  INSTR_4: // INSTR_STOP:
            chrono.stop();
            break;
        case INSTR_BRIGHTNESS:
            myLED.displayBrightness(display_received.brightness);

            packet_to_slave.brightness = 0;
            packet_to_slave.alarm = 0;
            packet_to_slave.brightness = display_received.brightness;
#if SEND_TO_SLAVE
            memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
#endif // SEND_TO_SLAVE   
            break;

        default:
            break;
        }
    }

    compare_elapsed = 10000;

    if (display_received.insrtuction == INSTR_ID_REQ)
    {
        packet_to_console.insrtuction = INSTR_ID_REQ;
        memcpy(packet_to_console.display, device_id, sizeof(device_id));
        result = esp_now_send(mac, (uint8_t*)&packet_to_slave, sizeof(device_id));
        Serial.print("Try to replay device ID: ");
        Serial.println(result == ESP_OK ? "Delivery Success" : "Delivery Fail");
    }
}

//-----------------------------------------------------------------------------
//void IRAM_ATTR remote_ISR() {
//    char button;
//    bool timer_status = chrono.isRunning();
//
//    if (digitalRead(REMOTE_D0))
//    {
//        button = 'A';
//        countdown_time = SHOOT_TIME_A;
//        chrono.restart();
//        if (!timer_status)
//        {
//            chrono.stop();
//        }   
//      
//    }
//    else if (digitalRead(REMOTE_D1))
//    {
//        button = 'B';
//        countdown_time = SHOOT_TIME_B;
//        chrono.restart();
//        if (!timer_status)
//        {
//            chrono.stop();
//        }
//    }
//    else if (digitalRead(REMOTE_D2))
//    {
//        chrono.resume();
//        button = 'C';
//        
//    }
//    else if (digitalRead(REMOTE_D3))
//    {
//        chrono.stop();
//        button = 'D';
//
//    }
//    compare_elapsed = 10000;
//    Serial.print("ISR Remote pressed button: ");
//    Serial.println(button);
//
//    //while (digitalRead(REMOTE_D0) || digitalRead(REMOTE_D1) || digitalRead(REMOTE_D2) || digitalRead(REMOTE_D3) )
//}

hw_timer_t* flash_timer = NULL;
portMUX_TYPE falshMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR FlashInt()
{
    portENTER_CRITICAL_ISR(&falshMux);

    if (set_on && cursor_on) {
        Display_buf[cursor_index] = ' ';
        myLED.print(Display_buf, INVERT_DISPLAY);
		cursor_on = false;
    }
    else if (set_on && cursor_on == false){
        Display_buf[0] = Set_buffer[0]; Display_buf[1] = Set_buffer[1];
        myLED.print(Display_buf, INVERT_DISPLAY);
		cursor_on = true;
    }

    if (buzzer_cnt) {
        buzzer_cnt--;
    }
    else {
        digitalWrite(BUZZER, LOW);
        digitalWrite(12, LOW);
        //analogWrite(12, 0, 4000, 8, 0);
    }

	if (IR_key_timeout) {
		IR_key_timeout--;		
    }
    else
    {
        IR_key_set = false;
    }

    // Ελέγχουμε την τρέχουσα κατάσταση του pin
    int state = digitalRead(LED);
    // Ανάβουμε το LED αν είναι σβηστό
    digitalWrite(LED, !state);

    portEXIT_CRITICAL_ISR(&falshMux);

}

TaskHandle_t Task0;

// The setup() function runs once each time the micro-controller starts
void setup()
{
    Serial.begin(115200);
    EEPROM.begin(255);
	while (!Serial) { ; }
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); //Transmiter for receiving ATTINY804
    delay(100);
    Serial.println("Started Initializing...");
    Serial.println(__FILE__);
    Serial.println(__DATE__);
    Serial.println("-------------------------------------------------");
    //-----------------------------------------------------------------------
    irrecv.enableIRIn();

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    photoValue = (255 / 4095.0) * analogRead(photoPin);
    brightness = 255 - photoValue;
    if (!brightness) { brightness = 10; }
    myLED.displayBrightness(brightness);
    Serial.print("Bbrightness: ");
    Serial.println(brightness);

    chrono.stop();
    countdown_time = EEPROM.read(EEP_TIME_A); // SHOOT_TIME_A;
//------------------------------------------------------------------------
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    else {
        Serial.println("initialized ESP - NOW");
    }

    esp_now_register_recv_cb(OnDataRecv);
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    // Register peer

#if SEND_TO_SLAVE
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer 1");
        //return;
    }
#endif // SEND_TO_SLAVE

//------------------------------------------------------------------------

    // return the clock speed of the CPU
    uint8_t cpuClock = ESP.getCpuFreqMHz();
    flash_timer = timerBegin(1, cpuClock, true);
    timerAttachInterrupt(flash_timer, &FlashInt, true);
    timerAlarmWrite(flash_timer, 100000, true);
    timerAlarmEnable(flash_timer);

    /*pinMode(REMOTE_D0, INPUT_PULLUP);
    pinMode(REMOTE_D1, INPUT_PULLUP);
    pinMode(REMOTE_D2, INPUT_PULLUP);
    pinMode(REMOTE_D3, INPUT_PULLUP);
    attachInterrupt(REMOTE_D0, remote_ISR, RISING);
    attachInterrupt(REMOTE_D1, remote_ISR, RISING);
    attachInterrupt(REMOTE_D2, remote_ISR, RISING);
    attachInterrupt(REMOTE_D3, remote_ISR, RISING);*/

    myLED.displayEnable();     // This command has no effect if you aren't using OE pin
    myLED.normalMode();
    myLED.displayBrightness(255);
    myLED.TestSegments(DISPLAY_DIGITS);

    myLED.print("--", INVERT_DISPLAY);
    digitalWrite(BUZZER, HIGH);
    buzzer_cnt = 2;
    
	//Create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
        Task0code,   /* Task function. */
        "Task0",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        0,           /* priority of the task */
        &Task0,      /* Task handle to keep track of created task */
        0);          /* pin task to core 0 */
    delay(500);

	Serial.println("Setup completed.");
    Serial.println("----------------------");
}

void Task0code(void* pvParameters) {
	for (;;)
	{
        if (irrecv.decode(&results)) {
            if (results.decode_type == NEC && results.address == NEC_ADD_ROUSIS) {
                handleIRCommand(results.command);
				Serial.print("IR Command: "); 
				//Serial.print(results.command, HEX);
            }
            else if (results.decode_type == NEC && results.command == 0xFFFFFFFF) {
                //handleIRCommand(results.command); // Handle repeat code
				/*Serial.print("IR Command: ");
				Serial.print(results.command, HEX);*/
			}
            else {
                /*Serial.print("Unsupported IR signal or invalid address: ");
                Serial.print("Protocol="); Serial.print(results.decode_type);
                Serial.print(", Address=0x"); Serial.println(results.address, HEX);*/
            }
            irrecv.resume();
        }

        delay(2);
	}
	//vTaskDelete(Task0);
}

// Add the main program code into the continuous loop() function
void loop()
{
    if (chrono.hasPassed(countdown_time) && !slave)
    {
        countdown_time = EEPROM.read(EEP_TIME_A); //SHOOT_TIME_A;
        chrono.restart();
        digitalWrite(BUZZER, HIGH);
        buzzer_cnt = 4;

        timer_to_display();
        myLED.print(Display_buf, INVERT_DISPLAY);
        if (device_addr == 2)
        {
            packet_to_slave.id = device_addr;
            packet_to_slave.brightness = 0;
            packet_to_slave.alarm = 4;
#if SEND_TO_SLAVE
            memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
#endif        
        }
    }
    else if (compare_elapsed != chrono.elapsed() && !slave)
    {
        compare_elapsed = chrono.elapsed();
        timer_to_display();
        myLED.print(Display_buf, INVERT_DISPLAY);
        
        if (device_addr == 2)
        {
            packet_to_slave.id = device_addr;
            packet_to_slave.brightness = 0;
            packet_to_slave.alarm = 0;
#if SEND_TO_SLAVE
            memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
#endif        
        }
    }

    /*for (;;)
    {
        if (chrono.hasPassed(countdown_time))
        {
            countdown_time = SHOOT_TIME_A;
            chrono.restart();
            digitalWrite(BUZZER, HIGH);
            buzzer_cnt = 4;

            timer_to_display();
            myLED.print(Display_buf, INVERT_DISPLAY);
            packet_to_slave.brightness = 0;
            packet_to_slave.alarm = 4;
            memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
        }
        else if (compare_elapsed != chrono.elapsed())
        {
            compare_elapsed = chrono.elapsed();
            timer_to_display();
            myLED.print(Display_buf, INVERT_DISPLAY);

            packet_to_slave.brightness = 0;
            packet_to_slave.alarm = 0;
            memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
        }               
    }*/

    if (Serial2.available() >= 3)
    {
        byte received_bytes[10];
        uint8_t i = 0;
        while (Serial2.available())
        {
            received_bytes[i++] = Serial2.read();
        }
        /*Serial.print("RF Intruction: ");
        Serial.print(received_bytes[0], HEX);
        Serial.print(" ");
        Serial.print(received_bytes[1], HEX);
        Serial.print(" ");
        Serial.print(received_bytes[2], HEX);
        Serial.println();*/
        bool timer_status = chrono.isRunning();
        if (received_bytes[0] != HEADER_A || received_bytes[1] != HEADER_B || slave)
        {
            //Serial.println("RF Wrong instruction...");
        }
        else if (received_bytes[2] == 'A')
        {
            countdown_time = EEPROM.read(EEP_TIME_A); //SHOOT_TIME_A;
            chrono.restart();
            if (!timer_status)
            {
                chrono.stop();
            }
            compare_elapsed = 10000;
            Serial.println("RF A");
        }
        else if (received_bytes[2] == 'B')
        {
            countdown_time = EEPROM.read(EEP_TIME_B); //SHOOT_TIME_B;
            chrono.restart();
            if (!timer_status)
            {
                chrono.stop();
            }
            compare_elapsed = 10000;
            Serial.println("RF B");
        }
        else if (received_bytes[2] == 'C')
        {
            chrono.resume();
            Serial.println("RF C");
        }
        else if (received_bytes[2] == 'D')
        {
            chrono.stop();
            Serial.println("RF D");
        }
    }

    if (millis() - photo_delay > PHOTO_SAMPLE_TIME)
    {
        Photo_sample();
        photo_delay = millis();
    }
}

void timer_to_display() {
    char buffer[5];

    int countdown_elapled;
    countdown_elapled = countdown_time - chrono.elapsed();

    seconds = countdown_elapled % 60;
    minutes = (countdown_elapled / 60) % 60;

    itoa(minutes, buffer, 10);
    if (buffer[1] == 0)
    {
        buffer[1] = buffer[0];
        buffer[0] = '0';
    }
    Display_buf[0] = buffer[0]; Display_buf[1] = buffer[1] | 0x80;

    itoa(seconds, buffer, 10);
    if (buffer[1] == 0)
    {
        buffer[1] = buffer[0];
        buffer[0] = '0';
    }
    Display_buf[0] = buffer[0]; Display_buf[1] = buffer[1];
}

void Photo_sample() {
    if (sample_metter < PHOTO_SAMPLES)
    {
        photoValue = (255 / 4095.0) * analogRead(photoPin);
        brigthsamples[sample_metter++] = 255 - photoValue;
    }
    else {
        int sum_smpl = 0;
        for (size_t i = 0; i < PHOTO_SAMPLES; i++)
        {
            sum_smpl += brigthsamples[i];
        }
        brightness = sum_smpl / PHOTO_SAMPLES;
        sample_metter = 0;
        if (!brightness) { brightness = 10; }
        myLED.displayBrightness(brightness);

        Serial.println();
        Serial.print("New average brightness: ");
        Serial.println(brightness);
    }
}

void handleIRCommand(uint32_t command) {
    bool timer_status = chrono.isRunning();

    switch (command) {
	case KEY_RESET:
		// reset the device
		ESP.restart();
		Serial.println("IR: EXIT");
		break;
	case KEY_POWER:
		if (!Display_on)
		{
            myLED.print(Display_buf, INVERT_DISPLAY);
			Display_on = true;
#if SEND_TO_SLAVE
            memcpy(packet_to_slave.time, Display_buf, DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
#endif // SEND_TO_SLAVE            
		}
		else
		{
            // show spaces on the display
            chrono.stop();
            myLED.print("  ", INVERT_DISPLAY);
#if SEND_TO_SLAVE
            memcpy(packet_to_slave.time, "  ", DISPLAY_DIGITS);
            result = esp_now_send(broadcastAddress1, (uint8_t*)&packet_to_slave, sizeof(packet_to_slave));
#endif // SEND_TO_SLAVE            
			Display_on = false;
		}
		Serial.println("IR: POWER");
		
		break;
	case KEY_COUNTDOWN_1:
		if (IR_key_set)
		{
			Set_timer(EEP_TIME_A);
			Serial.println("IR: KEY_SET 1 pressed");
		}
        else {
            countdown_time = EEPROM.read(EEP_TIME_A);
            chrono.restart();
            if (!timer_status)
            {
                chrono.stop();
            }
            compare_elapsed = 10000;
            Serial.println("IR = RESET 1");
        }
		break;    
    case KEY_COUNTDOWN_2:
        if (IR_key_set)
        {
            Set_timer(EEP_TIME_B);
			Serial.println("IR: KEY_SET 2 pressed");
        }
        else {
            countdown_time = EEPROM.read(EEP_TIME_B);
            chrono.restart();
            if (!timer_status)
            {
                chrono.stop();
            }
            compare_elapsed = 10000;
            Serial.println("IR = RESET 2");
        }
		break;
	case KEY_PLAY:
		chrono.resume();
		Serial.println("IR: PLAY");
	    break;
	case KEY_STOP:
		chrono.stop();
		Serial.println("IR: STOP");
		break;
	case KEY_SET:
		IR_key_set = true;
		IR_key_timeout = 12;
		Serial.println("IR: KEY_SET pressed");
		break;
	case KEY_HORN:
		digitalWrite(BUZZER, HIGH);
		buzzer_cnt = 4;
		Serial.println("IR: HORN");
		break;
    default:
        Serial.print("IR: Unknown command 0x");
        Serial.println(results.command, HEX);
        Serial.print("Protocol: ");
        Serial.print(results.decode_type, HEX);
        Serial.print("address: ");
        Serial.print(results.address, HEX);
		Serial.println();
        break;
    }

}

uint8_t Set_timer(uint16_t Address_time) {

	uint8_t timer_value = EEPROM.read(Address_time);
	if (timer_value > 99) 
		timer_value = 99;
    chrono.stop();

	//make a code for setting the timer in 2 digits 
	// With a cursor on the display
	
	itoa(timer_value, Set_buffer, 10);
	if (Set_buffer[1] == 0)
	{
		Set_buffer[1] = Set_buffer[0];
		Set_buffer[0] = '0';
	}
	Display_buf[0] = Set_buffer[0]; Display_buf[1] = Set_buffer[1];
	myLED.print(Display_buf, INVERT_DISPLAY);
	cursor_index = 0;
	set_on = true;
    results.command = 0xff;
	// Wait for the user to press the OK button
	while (true) {
		if (irrecv.decode(&results)) {
			if (results.decode_type == NEC && results.address == NEC_ADD_ROUSIS && results.command == KEY_EXIT) {
                timer_to_display();
                myLED.print(Display_buf, INVERT_DISPLAY);
				break; // Exit the loop when OK is pressed
			}
			irrecv.resume();
        }
        else if (results.decode_type == NEC && results.address == NEC_ADD_ROUSIS && results.command == KEY_RIGHT) {
			if (cursor_index == 0) {
				cursor_index = 1;
			}
            results.command = 0xff;
		}
		else if (results.decode_type == NEC && results.address == NEC_ADD_ROUSIS && results.command == KEY_LEFT) {
			cursor_index = 0;
            results.command = 0xff;
		}
        else if (results.address == NEC_ADD_ROUSIS && results.command <= 9) {
			char digit = results.command + '0';
			Set_buffer[cursor_index] = digit;
            if (cursor_index == 0) {
                cursor_index = 1;
            }
            Display_buf[0] = Set_buffer[0]; Display_buf[1] = Set_buffer[1];
            myLED.print(Display_buf, INVERT_DISPLAY);            
            results.command = 0xff;
        }
        else if (results.decode_type == NEC && results.address == NEC_ADD_ROUSIS && results.command == KEY_OK) {
            // Save the timer value
            timer_value = (Set_buffer[0] - '0') * 10 + (Set_buffer[1] - '0');
            EEPROM.write(Address_time, timer_value);
            EEPROM.commit();
            timer_to_display();
            myLED.print(Display_buf, INVERT_DISPLAY);
            break; // Exit the loop when OK is pressed
        }
		
		delay(2);
	}

	set_on = false;
	return timer_value;
}