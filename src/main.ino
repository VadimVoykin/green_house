#define CLK 2
#define DT 3
#define SW 4

#define SOIL_HUM_MIN_VAL_0 560
#define SOIL_HUM_MAX_VAL_0 230
#define SOIL_HUM_MIN_VAL_1 560
#define SOIL_HUM_MAX_VAL_1 230

#define SOIL_HUM_FROM 0
#define SOIL_HUM_TO 100

#define ITEMS 9 //amount of menu items
#define SIZE 4 //size of display in rows
#define ROM_INDEX 4 // amount of immutable variables (info from sensors)
#define PUMP_DELAY 5000 //time estimated for water imbibition
#define BACKGROUND_PROCESS_DELAY 1000 //period of executing of background processes

#define DHT_TEMP_INDEX 1 //INDEX array(SENSORS); air temperature 
#define DHT_HUM_INDEX 0 //INDEX array(SENSORS); air humidity
#define DHT_PIN 5 //pin of DHT sensor
 
#define LED_PIN 6 //PIN; led 
#define LED_INDEX ROM_INDEX + 4 //INDEX array(INFO); led pwm

#define PUMP_PIN_0 7 //PIN; first pump
#define PUMP_PIN_1 8 //PIN; second pump

#define PUMP_HUM_INDEX_0 ROM_INDEX //INDEX array(INFO); min soil hum of first plant 
#define PUMP_TIME_INDEX_0 ROM_INDEX + 1 //INDEX array(INFO); time of pumping of first plant
#define PUMP_HUM_INDEX_1 ROM_INDEX + 2 //INDEX array(INFO); min soil hum of second plant
#define PUMP_TIME_INDEX_1 ROM_INDEX + 3 //INDEX array(INFO); time of pumping of second plant
#define SOIL_HUM_INDEX_0 2 //INDEX array(SENSORS); soil hum value of first plant 
#define SOIL_HUM_INDEX_1 3 //INDEX array(SENSORS); soil hum value of second plant

#define HUM_SEN_PIN_0 A2 //PIN; first soil humidity sensor
#define HUM_SEN_PIN_1 A3 //PIN; second soil humidity sensor
 
#define background_process_delay 100;
#define current_menu_info_delay 500;

#include <GyverOLED.h> //need init 
#include "GyverEncoder.h" //DO NOT need init
#include "DHT.h"  //need init

Encoder enc1(CLK, DT, SW);  
GyverOLED<SSD1306_128x32> oled;
DHT dht(DHT_PIN, DHT11);

const char sm_1_item_0[] PROGMEM = "  hum of air:";
const char sm_1_item_1[] PROGMEM = "  temp of air:";
const char sm_1_item_3[] PROGMEM = "  hum of soil 0:";
const char sm_1_item_4[] PROGMEM = "  hum of soil 1:";
const char sm_1_item_5[] PROGMEM = "  pump hum 0:";
const char sm_1_item_6[] PROGMEM = "  pump t 0:";
const char sm_1_item_7[] PROGMEM = "  pump hum 1:";
const char sm_1_item_8[] PROGMEM = "  pump t 1:";
const char sm_1_item_9[] PROGMEM = "  led pwm:";

const char* const data[ITEMS] PROGMEM = {
sm_1_item_0, sm_1_item_1,
sm_1_item_3, sm_1_item_4, sm_1_item_5,
sm_1_item_6, sm_1_item_7, sm_1_item_8,
sm_1_item_9};

byte pointer;
bool flag;
bool disp_update;
uint8_t sensors[ITEMS];
uint8_t info[ITEMS];
uint8_t display_size;
unsigned long background_process_last_time;
unsigned long last_time_watering_0;
unsigned long last_time_watering_1;
uint8_t pump_state_0;
uint8_t pump_state_1;

void get_air_info(){
    sensors[DHT_HUM_INDEX] =runMiddleArifm_0((int)dht.readHumidity());
    sensors[DHT_TEMP_INDEX] = runMiddleArifm_1((int)dht.readTemperature());
    disp_update = true;
} 

int runMiddleArifm_0(int newVal){
  static byte idx_0 = 0;
  static int valArray_0[10];
  valArray_0[idx_0] = newVal;
  if (++idx_0 >= 10) idx_0 = 0;     
  int average_0 = 0;                  
  for (int i = 0; i < 10; i++) {
    average_0 += valArray_0[i];        
  }
  return (int)average_0 / 10;
  }

  int runMiddleArifm_1(int newVal){
  static byte idx_1 = 0;
  static int valArray_1[10];  
  valArray_1[idx_1] = newVal;
  if (++idx_1 >= 10) idx_1 = 0;     
  int average_1 = 0;                  
  for (int i = 0; i < 10; i++) {
    average_1 += valArray_1[i];        
  }
  return (int)average_1 / 10;
  }

  

void get_hum_info(){
      //Serial.print(analogRead(HUM_SEN_PIN_0));
      //Serial.print("   ");
      //Serial.println(analogRead(HUM_SEN_PIN_1));
      int array[20]; 
        for(int i = 0; i < 11; i++){
          array[i] = map(analogRead(HUM_SEN_PIN_0), SOIL_HUM_MIN_VAL_0, SOIL_HUM_MAX_VAL_0,
          SOIL_HUM_FROM, SOIL_HUM_TO);
        }
      //int8_t temporary_0 = map(analogRead(HUM_SEN_PIN_0), SOIL_HUM_MIN_VAL_0, SOIL_HUM_MAX_VAL_0,
      //  SOIL_HUM_FROM, SOIL_HUM_TO);
      //sensors[SOIL_HUM_INDEX_0] = constrain(temporary_0, SOIL_HUM_FROM, SOIL_HUM_TO);
      sensors[SOIL_HUM_INDEX_0] = midHumidity(array);
      for(int i = 0; i < 11; i++){
        array[i] = map(analogRead(HUM_SEN_PIN_1), SOIL_HUM_MIN_VAL_1, SOIL_HUM_MAX_VAL_1,
        SOIL_HUM_FROM, SOIL_HUM_TO);
      }
      sensors[SOIL_HUM_INDEX_1] = midHumidity(array);
     // sensors[SOIL_HUM_INDEX_1] = constrain(temporary_1 , SOIL_HUM_FROM, SOIL_HUM_TO);
}

int midHumidity(int array[]){
    int sum = 0;
    for (int i = 0; i < sizeof(array) / 2; i++){
      sum += array[i];
      return (sum/(sizeof(array) / 2));
    }
  }

void set_led_pwm(){
  analogWrite(LED_PIN, info[LED_INDEX]);  
}

void pump_plant_0(){
  if (sensors[SOIL_HUM_INDEX_0] < info[PUMP_HUM_INDEX_0] && pump_state_0 == 0){
    pump_state_0 = 1;
    digitalWrite(PUMP_PIN_0, true);
    Serial.println("Pump 0 on");
    last_time_watering_0 = millis(); 
  }
  if (millis() - last_time_watering_0 > info[PUMP_TIME_INDEX_0] * 100 && pump_state_0 == 1){
    pump_state_0 = 2;
    digitalWrite(PUMP_PIN_0, false);
    Serial.println("Pump 0 off");
    last_time_watering_0 = millis();
  }
  if (millis() - last_time_watering_0 > PUMP_DELAY && pump_state_0 == 2){
      pump_state_0 = 0;
      Serial.println("Pump 0 ready to water");
  }  
}

void pump_plant_1(){
  if (sensors[SOIL_HUM_INDEX_1] < info[PUMP_HUM_INDEX_1] && pump_state_1 == 0){
    pump_state_1 = 1;
    digitalWrite(PUMP_PIN_1, true);
    Serial.println("Pump 1 on");
    last_time_watering_1 = millis(); 
  }
  if (millis() - last_time_watering_1 > info[PUMP_TIME_INDEX_1] && pump_state_1 == 1){
    pump_state_1 = 2;
    digitalWrite(PUMP_PIN_1, false);
    Serial.println("Pump 1 off");
    last_time_watering_1 = millis();
  }
  if (millis() - last_time_watering_1 > PUMP_DELAY && pump_state_1 == 2){
      pump_state_1 = 0;
      Serial.println("Pump 1 ready to water");
  }  
}

void func(){ 
    if(enc1.isClick()){
        Serial.println("Click");
        if(pointer > ROM_INDEX - 1){
          flag = !flag;
        }
        disp_update = true;
    }
    
    if (enc1.isRight()){
        if(flag){
            info[pointer]--;
            eeprom_write_block((void*)info, 0, sizeof(info));
        }else{
            pointer = constrain(pointer - 1, 0, ITEMS-1);
        }
        Serial.println("Right");
        disp_update = true;
    }
    if(enc1.isLeft()){
      if(flag){
        info[pointer]++;
        eeprom_write_block((void*)info, 0, sizeof(info));
       }else{
        pointer = constrain(pointer + 1, 0, ITEMS-1);
       } 
       Serial.println("Left");
       disp_update = true;
    }
    if(enc1.isLeftH() && flag){
      Serial.println("lefth");
      if(pointer == LED_INDEX){
        info[pointer] += 16;
      }
      if(pointer == PUMP_HUM_INDEX_0 || pointer == PUMP_TIME_INDEX_0 ||
         pointer == PUMP_HUM_INDEX_1 || pointer == PUMP_TIME_INDEX_1){
        info[pointer] += 10; 
      }
      disp_update = true;
    }
    if(enc1.isRightH() && flag){
      Serial.println("righth");
      if(pointer == LED_INDEX){
        info[pointer] -= 16;
      }
      if(pointer == PUMP_HUM_INDEX_0 || pointer == PUMP_TIME_INDEX_0 ||
         pointer == PUMP_HUM_INDEX_1 || pointer == PUMP_TIME_INDEX_1){
        info[pointer] -= 10; 
      }
      disp_update = true;
    }
    if(disp_update){
      oled.clear();
      oled.home();
      for(int i = 0; i < SIZE; ++i){
          char buffer[20];
          strcpy_P(buffer, pgm_read_word(&(data[(pointer > (SIZE - 1)) ? i + (pointer - (SIZE - 1)) : i])));
          oled.println(buffer);
      }
      for(int i = 0; i < SIZE; ++i){
          //uint8_t len = strlen_P(pgm_read_word(&(data[(pointer > 3) ? i + (pointer - 3) : i])));
          oled.setCursor(100, i);
          uint8_t cur_pos = (pointer > (SIZE - 1)) ? i + (pointer - (SIZE - 1)) : i;
          if(cur_pos > (ROM_INDEX - 1)){
            oled.print(info[cur_pos]);
          }else{
            oled.print(sensors[cur_pos]);
          }
      }
      printCursor((pointer > (SIZE - 1)) ? (SIZE - 1) : pointer, flag);
      oled.update();
      disp_update = false;
    }
}

void printCursor(uint8_t pointer, bool flag){
    if(flag){
        oled.setCursor(125, pointer);
        oled.print("<");
    }else{
        oled.setCursor(0, pointer);
        oled.print(">");
    }
}

void setup() {
    //--init--
    Serial.begin(9600);
    enc1.setType(TYPE2);
    oled.init();
    dht.begin();
    pinMode(HUM_SEN_PIN_0, INPUT);
    pinMode(HUM_SEN_PIN_1, INPUT);
    pinMode(PUMP_PIN_0, OUTPUT);
    pinMode(PUMP_PIN_1, OUTPUT);
    //--service--
    oled.setScale(1);
    oled.setContrast(15);
    //--timers--
    last_time_watering_0 = millis();
    last_time_watering_1 = millis();
    background_process_last_time = millis();
    //--variables--
    flag = false;
    pointer = 0;
    disp_update = true; 
    eeprom_read_block((void*)info, 0, sizeof(info));
    for(int i = 0; i < ROM_INDEX; i++){
      sensors[i] = 10;
    }
    pump_state_0 = 0;
    //--welcome--
    oled.clear();
    oled.setCursor(4*8, 1);
    oled.println("Science Hub");
    oled.setCursor(6*8, 2);
    oled.println("edition");
    oled.update();
    delay(1000);
    oled.clear();
    oled.home();
    oled.update();
}

void loop() {
    enc1.tick();
    if(millis() - background_process_last_time > BACKGROUND_PROCESS_DELAY){
      background_process_last_time = millis();
      get_air_info();
      set_led_pwm();
      get_hum_info();
      pump_plant_0();
      pump_plant_1();   
    }
    func();
}
