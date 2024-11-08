#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"             
#include "soc/rtc_cntl_reg.h"    
#include "esp_http_server.h"
#include <ESP32Servo.h>          

// Replace with your network credentials
const char* ssid = "my_car";
const char* password = "12345678";

#define PART_BOUNDARY "123456789000000000000987654321"

#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
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
#endif

#define MOTOR_1_PIN_1    14
#define MOTOR_1_PIN_2    15
#define MOTOR_2_PIN_1    13
#define MOTOR_2_PIN_2    12
#define SERVO_PIN        2   
#define LED_PIN          4   
#define RED_LED_PIN      16  // GPIO pin for Red LED

Servo servo;   
int servoPos = 90;  

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM Robot</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
      table { margin-left: auto; margin-right: auto; }
      td { padding: 8px; }
      .button {
        background-color: #2f4468;
        border: none;
        color: white;
        padding: 10px 20px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 18px;
        margin: 6px 3px;
        cursor: pointer;
      }
      img { width: auto; max-width: 100%; height: auto; }
      input[type="range"] {
        width: 100%;
      }
    </style>
  </head>
  <body>
    <h1>ESP32-CAM Robot</h1>
    <img src="" id="photo">
    <table>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('forward');" ontouchstart="toggleCheckbox('forward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Forward</button></td></tr>
      <tr><td align="center"><button class="button" onmousedown="toggleCheckbox('left');" ontouchstart="toggleCheckbox('left');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Left</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('stop');" ontouchstart="toggleCheckbox('stop');">Stop</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('right');" ontouchstart="toggleCheckbox('right');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Right</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('backward');" ontouchstart="toggleCheckbox('backward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Backward</button></td></tr>                   
      <tr><td colspan="3" align="center"><button class="button" onclick="toggleCheckbox('ledOn');">LED On</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onclick="toggleCheckbox('ledOff');">LED Off</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onclick="toggleCheckbox('servoUp');">Tilt Up</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onclick="toggleCheckbox('servoDown');">Tilt Down</button></td></tr>
      <tr><td colspan="3" align="center"><input type="range" min="0" max="180" value="90" id="servoSlider" onchange="updateServoPosition(this.value);"></td></tr>
    </table>
   <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=" + x, true);
     xhr.send();
   }
   function updateServoPosition(value) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=servoPosition&value=" + value, true);
     xhr.send();
   }
   window.onload = document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";
  </script>
  </body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
  }
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    }
    
    if (strcmp(variable, "forward") == 0) {
      moveForward();
    } else if (strcmp(variable, "backward") == 0) {
      moveBackward();
      digitalWrite(RED_LED_PIN, HIGH);  // LED on when moving backward
    } else if (strcmp(variable, "stop") == 0) {
      stopMotors();
      digitalWrite(RED_LED_PIN, LOW);  // LED off when stopped
    } else if (strcmp(variable, "left") == 0) {
      turnLeft();
    } else if (strcmp(variable, "right") == 0) {
      turnRight();
    } else if (strcmp(variable, "ledOn") == 0) {
      digitalWrite(LED_PIN, HIGH);
    } else if (strcmp(variable, "ledOff") == 0) {
      digitalWrite(LED_PIN, LOW);
    } else if (strcmp(variable, "servoPosition") == 0) {
      int position;
      if (httpd_query_key_value(buf, "value", variable, sizeof(variable)) == ESP_OK) {
        position = atoi(variable);
        setServoPosition(position);
      }
    }
    
    free(buf);
  }
  
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
  pinMode(MOTOR_2_PIN_1, OUTPUT);
  pinMode(MOTOR_2_PIN_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  servo.write(servoPos);

  // Camera configuration
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
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Connect to Wi-Fi
  WiFi.softAP(ssid, password);
  httpd_config_t config_httpd = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};
  httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};
  httpd_uri_t cmd_uri = {.uri = "/action", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL};

  // Start HTTP server
  httpd_start(&camera_httpd, &config_httpd);
  httpd_register_uri_handler(camera_httpd, &index_uri);
  httpd_register_uri_handler(camera_httpd, &stream_uri);
  httpd_register_uri_handler(camera_httpd, &cmd_uri);
}

void loop() {
  // Nothing to do here; everything is handled by the HTTP server
}

void moveForward() {
  digitalWrite(MOTOR_1_PIN_1, HIGH);
  digitalWrite(MOTOR_1_PIN_2, LOW);
  digitalWrite(MOTOR_2_PIN_1, HIGH);
  digitalWrite(MOTOR_2_PIN_2, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_1_PIN_1, LOW);
  digitalWrite(MOTOR_1_PIN_2, HIGH);
  digitalWrite(MOTOR_2_PIN_1, LOW);
  digitalWrite(MOTOR_2_PIN_2, HIGH);
}

void turnLeft() {
  digitalWrite(MOTOR_1_PIN_1, LOW);
  digitalWrite(MOTOR_1_PIN_2, HIGH);
  digitalWrite(MOTOR_2_PIN_1, HIGH);
  digitalWrite(MOTOR_2_PIN_2, LOW);
}

void turnRight() {
  digitalWrite(MOTOR_1_PIN_1, HIGH);
  digitalWrite(MOTOR_1_PIN_2, LOW);
  digitalWrite(MOTOR_2_PIN_1, LOW);
  digitalWrite(MOTOR_2_PIN_2, HIGH);
}

void stopMotors() {
  digitalWrite(MOTOR_1_PIN_1, LOW);
  digitalWrite(MOTOR_1_PIN_2, LOW);
  digitalWrite(MOTOR_2_PIN_1, LOW);
  digitalWrite(MOTOR_2_PIN_2, LOW);
}

void setServoPosition(int position) {
  servo.write(position);
}
