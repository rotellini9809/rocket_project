#include <cstdlib>

/*
  ESP32 CAM Camera with MicroSD storage
  esp32cam-microsd.ino
  Take picture when button pressed
  Store image on MicroSD card
 
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
// Include Required Libraries
 
// Camera libraries
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
 
// MicroSD Libraries
#include "FS.h"
#include "SD_MMC.h"
 
// EEPROM Library
#include "EEPROM.h"
 
// Use 1 byte of EEPROM space
#define EEPROM_SIZE 1
 
// Counter for picture number
unsigned int videoCount = 0;
 
// Pin definitions for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define RED_LED           33
#define ROCKET_INPUT      3
#define FLASH_LED         4


  
 
bool configESPCamera() {
  // Configure Camera parameters
 
  // Object to store the camera configuration parameters
  camera_config_t config;
 
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // Choices are YUV422, GRAYSCALE, RGB565, JPEG
 
  
  // Select lower framesize if the camera doesn't support PSRAM
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10; //10-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
 
  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    
    return false;
  }
 
  // Camera quality adjustments
  sensor_t * s = esp_camera_sensor_get();
 
  // BRIGHTNESS (-2 to 2)
  s->set_brightness(s, 0);
  // CONTRAST (-2 to 2)
  s->set_contrast(s, 0);
  // SATURATION (-2 to 2)
  s->set_saturation(s, 0);
  // SPECIAL EFFECTS (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_special_effect(s, 0);
  // WHITE BALANCE (0 = Disable , 1 = Enable)
  s->set_whitebal(s, 1);
  // AWB GAIN (0 = Disable , 1 = Enable)
  s->set_awb_gain(s, 1);
  // WB MODES (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_wb_mode(s, 0);
  // EXPOSURE CONTROLS (0 = Disable , 1 = Enable)
  s->set_exposure_ctrl(s, 1);
  // AEC2 (0 = Disable , 1 = Enable)
  s->set_aec2(s, 0);
  // AE LEVELS (-2 to 2)
  s->set_ae_level(s, 0);
  // AEC VALUES (0 to 1200)
  s->set_aec_value(s, 300);
  // GAIN CONTROLS (0 = Disable , 1 = Enable)
  s->set_gain_ctrl(s, 1);
  // AGC GAIN (0 to 30)
  s->set_agc_gain(s, 0);
  // GAIN CEILING (0 to 6)
  s->set_gainceiling(s, (gainceiling_t)0);
  // BPC (0 = Disable , 1 = Enable)
  s->set_bpc(s, 0);
  // WPC (0 = Disable , 1 = Enable)
  s->set_wpc(s, 1);
  // RAW GMA (0 = Disable , 1 = Enable)
  s->set_raw_gma(s, 1);
  // LENC (0 = Disable , 1 = Enable)
  s->set_lenc(s, 1);
  // HORIZ MIRROR (0 = Disable , 1 = Enable)
  s->set_hmirror(s, 0);
  // VERT FLIP (0 = Disable , 1 = Enable)
  s->set_vflip(s, 0);
  // DCW (0 = Disable , 1 = Enable)
  s->set_dcw(s, 1);
  // COLOR BAR PATTERN (0 = Disable , 1 = Enable)
  s->set_colorbar(s, 0);
 
  return true;
}
 
bool initMicroSDCard() {
  // Start the MicroSD card
 
  Serial.println("Mounting MicroSD Card");
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("MicroSD Card Mount Failed");
    return false;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No MicroSD Card found");
    return false;
  }
 return true;
}
 
bool takeNewPhoto(String path) {
  bool output;
  // Take Picture with Camera
 
  // Setup frame buffer
  camera_fb_t  * fb = esp_camera_fb_get();

  
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
 
  // Save picture to microSD card
  fs::FS &fs = SD_MMC;
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in write mode");
    output = false;
    
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    output = true;
  }
  // Close the file
  file.close();
 
  // Return the frame buffer back to the driver for reuse
  esp_camera_fb_return(fb);

  return output;
}

bool createDirectory(String name) {
  fs::FS &fs = SD_MMC;
  if (fs.mkdir(name.c_str())) {
    Serial.println("Directory created");
    return true;
  } else {
    Serial.println("Directory creation failed");
    return false;
  }
}
 
void setup() {
  bool flag1 = false;
  bool flag2 = false;
  bool flag3 = false;

  // red led (inverted logic: high to turn it on low to turn it off)
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);

  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);


  //input to start and stop the video form the rocket
  pinMode(ROCKET_INPUT,INPUT);
 
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
 
  // Start Serial Monitor
  Serial.begin(115200);
 
  // Initialize the camera
  Serial.print("Initializing the camera module...");
  flag1 = configESPCamera();
  Serial.println("Camera OK!");
 
  // Initialize the MicroSD
  Serial.print("Initializing the MicroSD card module... ");
  flag2 = initMicroSDCard();

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  videoCount = EEPROM.read(0) + 1;

  String directory_name = "/vid_" + String(videoCount);
  flag3 = createDirectory(directory_name);


  if (flag1 && flag2 && flag3){
    sleep(1000)
    Serial.println("waiting for signal to start the video");
    return;

  } else {
    //led light if error occours
    
    digitalWrite(FLASH_LED, HIGH);
    while(true){}
  }
 
  
}



bool flag = false;

void loop() {
  
  

  if (digitalRead(ROCKET_INPUT) == HIGH){

    int frame_num = 1;
    
    unsigned long start;
    unsigned long millis_to_next_frame;

    Serial.printf("saving the video %d\n",videoCount);

    // Update EEPROM picture number counter
    EEPROM.write(0, videoCount);
    EEPROM.commit();

    flag = true;
    start = millis();
    digitalWrite(RED_LED, LOW);
    while(digitalRead(ROCKET_INPUT) == HIGH){   
      // Path where new picture will be saved in SD Card
      String path = "/vid_" + String(videoCount) +"/fr_" + String(frame_num)  + ".jpg";
      
    
      // Take and Save Photo
      takeNewPhoto(path);

      
      millis_to_next_frame = start + frame_num*31;  // 1000/32 = 31   to make the framerate 32
      while (millis() < millis_to_next_frame){ 
        sleep(1);
        Serial.println("riposo");
      }

      frame_num++;
    }
  }
  
  
  


  if (flag==true){
    digitalWrite(RED_LED, HIGH);
    Serial.println("video stopped");
    while(true){};
  }

  /*
  // Bind Wakeup to GPIO13 going LOW
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);
 
  Serial.println("Entering sleep mode");
  delay(1000);
 
  // Enter deep sleep mode
  esp_deep_sleep_start();
  */

 
}