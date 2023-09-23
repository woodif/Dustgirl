#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2cScd30.h>
#include <Wire.h>
#include <TFT_Touch.h>
/*Don't forget to set Sketchbook location in File/Preferencesto the path of your UI project (the parent foder of this INO file)*/
TaskHandle_t Task1;
/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */


#define DOUT 39  /* Data out pin (T_DO) of touch screen */
#define DIN  32  /* Data in pin (T_DIN) of touch screen */
#define DCS  33  /* Chip select pin (T_CS) of touch screen */
#define DCLK 25  /* Clock pin (T_CLK) of touch screen */

TFT_Touch touch = TFT_Touch(DCS, DCLK, DIN, DOUT);

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  tft.startWrite();
  tft.setAddrWindow( area->x1, area->y1, w, h );
  tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
  tft.endWrite();

  lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
uint8_t seting_mode = 0;
int coinx = 0;
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
  uint16_t touchX = 0, touchY = 0;

  bool touched = touch.Pressed();//tft.getTouch( &touchX, &touchY, 600 );
  Serial.println(coinx);
  if ( !touched )
  {
    data->state = LV_INDEV_STATE_REL;
    coinx = coinx-1;
    if(coinx <0)
      coinx = 0;
      seting_mode = 0;
    
  }
  else
  {
    
    touchX = touch.X();
    touchY = touch.Y();
    if(touchX>200 && touchY>180){
      coinx = coinx+1;
    }
    if(coinx > 50){
      seting_mode = 1;
    }
    
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif
SensirionI2CSen5x sen5x;
SensirionI2cScd30 sensor;
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;
float co2Concentration = 0.0;
float temperature = 0.0;
float humidity = 0.0;
int IDX = 0;
void setup()
{
  Serial.begin( 115200 ); /* prepare for possible serial debug */

  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  Serial.println( LVGL_Arduino );
  Serial.println( "I am LVGL_Arduino" );

  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

  tft.begin();          /* TFT init */
  tft.setRotation( 1 ); /* Landscape orientation, flipped */
  touch.setCal(526, 3443, 750, 3377, 320, 240, 1);
  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );


  ui_init();
  lv_obj_clear_flag(ui_Image7, LV_OBJ_FLAG_HIDDEN);
  lv_timer_handler();
  delay(3000);
  lv_obj_add_flag(ui_Image7, LV_OBJ_FLAG_HIDDEN);
  Serial.println( "Setup done" );
  Wire.begin(22, 27);
  sen5x.begin(Wire);
  sensor.begin(Wire, SCD30_I2C_ADDR_61);
  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
  if (error) {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  float tempOffset = 0.0;
  error = sen5x.setTemperatureOffsetSimple(tempOffset);
  if (error) {
    Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("Temperature Offset set to ");
    Serial.print(tempOffset);
    Serial.println(" deg. Celsius (SEN54/SEN55 only");
  }

  // Start Measurement
  error = sen5x.startMeasurement();
  if (error) {
    Serial.print("Error trying to execute startMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  error = sensor.startPeriodicMeasurement(0);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
  error = sen5x.readMeasuredValues(
            massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
            massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
            noxIndex);
 xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    20000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    10,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
}
void Task1code( void * pvParameters ) {

  for (;;) {
    lv_timer_handler();
    vTaskDelay(5);
  }

}
unsigned long previousMillis = 0;
void loop()
{

   /* let the GUI do its work */
  if (millis() - previousMillis >= 1000) {
    previousMillis = millis();
    Serial.print( "seting_mode " );
    Serial.println( seting_mode );
    String xxx = "ID-" + String(IDX);
    lv_label_set_text(ui_Label7, xxx.c_str());
    uint16_t error;
    char errorMessage[256];
    error = sen5x.readMeasuredValues(
              massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
              massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
              noxIndex);
    if (error) {
      Serial.print("Error trying to execute readMeasuredValues(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    } else {
      String xxx = String(massConcentrationPm2p5, 0);
      if (massConcentrationPm2p5 < 15) {
        lv_obj_set_style_bg_color(ui_Button1, lv_color_make(0x00, 0xF0, 0xFF), LV_PART_MAIN);
      } else if (massConcentrationPm2p5 < 25) {
        lv_obj_set_style_bg_color(ui_Button1, lv_color_make(0x00, 0xFF, 0x00), LV_PART_MAIN);
      } else if (massConcentrationPm2p5 < 37.5) {
        lv_obj_set_style_bg_color(ui_Button1, lv_color_make(0xFF, 0xFF, 0x00), LV_PART_MAIN);
      } else if (massConcentrationPm2p5 < 75) {
        lv_obj_set_style_bg_color(ui_Button1, lv_color_make(0xFF, 0xF0, 0x00), LV_PART_MAIN);
      } else {
        lv_obj_set_style_bg_color(ui_Button1, lv_color_make(0xFF, 0x00, 0x00), LV_PART_MAIN);
      }
      lv_label_set_text(ui_Label1, xxx.c_str());
      xxx = String(massConcentrationPm10p0, 0);
      lv_label_set_text(ui_Label2, xxx.c_str());
      xxx = String(vocIndex, 0);
      lv_label_set_text(ui_Label5, xxx.c_str());
    }
    error = sensor.blockingReadMeasurementData(co2Concentration, temperature, humidity);
    if (error != NO_ERROR) {
      Serial.print("Error trying to execute blockingReadMeasurementData(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    } else {
      String xxx = String(temperature, 0);
      lv_label_set_text(ui_Label3, xxx.c_str());
      xxx = String(humidity, 0);
      lv_label_set_text(ui_Label4, xxx.c_str());
      xxx = String(co2Concentration, 0);
      lv_label_set_text(ui_Label6, xxx.c_str());
    }
  }
}
