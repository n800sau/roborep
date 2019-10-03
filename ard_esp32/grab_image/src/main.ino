#include <Arduino.h>


#include "camera_index.h"
#include "esp_timer.h"
#include "esp_camera.h"

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

#define LED_BUILTIN 1

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif


}

void loop()
{
	// turn the LED on (HIGH is the voltage level)
	digitalWrite(LED_BUILTIN, HIGH);
	print_jpg();
	// wait for a second
	delay(1000);
	// turn the LED off by making the voltage LOW
	digitalWrite(LED_BUILTIN, LOW);
	 // wait for a second
	delay(1000);
}

#define DIVIDER "#----------#"

void output_jpeg(uint8_t *buf, size_t size)
{
	Serial.println(DIVIDER);
	Serial.write(buf, size);
	Serial.println();
	Serial.println(DIVIDER);
}

void print_jpg()
{
	size_t _jpg_buf_len = 0;
	uint8_t * _jpg_buf = NULL;

	camera_fb_t * fb = NULL;
	int64_t fr_start = esp_timer_get_time();

	fb = esp_camera_fb_get();
	if (!fb) {
		Serial.println("Camera capture failed");
	} else {
		if(fb->format == PIXFORMAT_JPEG){
			Serial.print("Captured jpeg size ");
			Serial.println(fb->len);
			output_jpeg(fb->buf, fb->len);
		} else {
			bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
			if(jpeg_converted) {
				Serial.print("Captured converted to jpeg size ");
				Serial.println(_jpg_buf_len);
				output_jpeg(_jpg_buf, _jpg_buf_len);
				free(_jpg_buf);
			} else {
				Serial.println("fmt2jpg failed");
			}
		}
		esp_camera_fb_return(fb);
	}
}
