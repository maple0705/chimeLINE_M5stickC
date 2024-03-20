
/*
 * https://homemadegarbage.com/m5stickc02
 * https://wak-tech.com/archives/1810
 */

#include <M5StickC.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ssl_client.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

//----------
#define PIN_CLK  0
#define PIN_DATA 34
#define READ_LEN (2 * 1024)
#define SAMPLING_FREQUENCY 44100
uint8_t BUFFER[READ_LEN] = {0};
uint16_t *adcBuffer = NULL;

const uint16_t FFTsamples = 512;  // サンプル数は2のべき乗
double vReal[FFTsamples];  // vReal[]にサンプリングしたデーターを入れる
double vImag[FFTsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFTsamples, SAMPLING_FREQUENCY);  // FFTオブジェクトを作る

unsigned int sampling_period_us;

float dmax = 7000000.0;
//----------

void wifiConnect() {
  char* ssid = "YOUR_SSID";
  char* password = "PASSWORD";
  
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);  // Wi-Fi接続
  while (WiFi.status() != WL_CONNECTED) {  // Wi-Fi 接続待ち
    delay(100);
    Serial.printf(".");
  }
  Serial.println("\nwifi connect ok");
}

void sendtoLINE(String message) {
  const char* host = "notify-api.line.me";
  const char* token = "LINE_TOKEN";
  WiFiClientSecure client;
  Serial.println("Try");
  //LineのAPIサーバに接続
  if (!client.connect(host, 443)) {
    Serial.println("Connection failed");
    return;
  }
  Serial.println("Connected");
  //リクエストを送信
  String query = String("message=") + message;
  String request = String("") +
               "POST /api/notify HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Authorization: Bearer " + token + "\r\n" +
               "Content-Length: " + String(query.length()) +  "\r\n" + 
               "Content-Type: application/x-www-form-urlencoded\r\n\r\n" +
                query + "\r\n";
  client.print(request);

  //受信終了まで待つ 
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    Serial.println(line);
    if (line == "\r") {
      break;
    }
  }

  String line = client.readStringUntil('\n');
  Serial.println(line);
}

void i2sInit(){
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = SAMPLING_FREQUENCY,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
   };

   i2s_pin_config_t pin_config;
   pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
   pin_config.ws_io_num    = PIN_CLK;
   pin_config.data_out_num = I2S_PIN_NO_CHANGE;
   pin_config.data_in_num  = PIN_DATA;
   
   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
   i2s_set_pin(I2S_NUM_0, &pin_config);
   i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void mic_record_task (void* arg){
  int detected, cnt_detect = 0;
  unsigned long t = millis(); // detected time

  while(1){
    i2s_read_bytes(I2S_NUM_0,(char*)BUFFER,READ_LEN,(100/portTICK_RATE_MS));
    adcBuffer = (uint16_t *)BUFFER;
    fft();

    M5.Lcd.fillScreen(BLACK);
    detected = drawChart(FFTsamples/2/2/2);
    if( detected ) {
      cnt_detect++;
      if( millis() - t > 5000 ) {
        cnt_detect = 1;
        t = millis();
      } else if( cnt_detect >= 3 ) {
        cnt_detect = 0;
        // sent to LINE
        sendtoLINE("チャイムが鳴っているよ.");
        printf("sent to LINE\n");
      }
      
    }
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("detected : %d", cnt_detect);    
    
    //vTaskDelay(100 / portTICK_RATE_MS);
  }
}

// return 1 if chime detected
int drawChart(int nsamples) {
    int X0 = 10, Y0 = 10, _height = 80 - Y0, _width = 160;
    int band_width = floor(_width / nsamples);
    int band_pad = 1;
    int detected = 0;

    for (int band = 0; band < nsamples; band++) {
        int hpos = band * band_width + X0;
        float d = vReal[band];
        double freq = (band * 1.0 * SAMPLING_FREQUENCY) / FFTsamples / 1000;  //kHz
        
        if (d > dmax) d = dmax;
        int h = (int)(d / dmax * _height);
        M5.Lcd.fillRect(hpos, _height - h, band_pad, h, WHITE);
        if ((band % (nsamples / 4)) == 0) {
            M5.Lcd.setCursor(hpos, _height + Y0 - 10);
            M5.Lcd.printf("%.1fk", freq);
        }

        if( d > 0.5 * dmax && freq > 0.9 ) {
          detected = 1;
          printf("%.3fkHz : %f\n", freq, vReal[band]);
        }
    }

    return detected == 0 ? 0 : 1;
}

void DCRemoval(uint16_t samples) {
    double mean = 0;
    
    for (uint16_t i = 1; i < samples; i++) {
        mean += vReal[i];
    }
    mean /= samples;
    for (uint16_t i = 1; i < samples; i++) {
        vReal[i] -= mean;
    }
}

void fft(){
    
  for (int i = 0; i < FFTsamples; i++) {
    unsigned long t = micros();
    vReal[i] = adcBuffer[i];
    vImag[i] = 0;
    while ((micros() - t) < sampling_period_us) ;
  }
  
  DCRemoval(FFTsamples);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // 窓関数
  FFT.Compute(FFT_FORWARD); // FFT処理(複素数で計算)
  FFT.ComplexToMagnitude(); // 複素数を実数に変換
}

void setup() {
  Serial.begin(115200);
  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Axp.ScreenBreath(10);
  
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  wifiConnect();
  sendtoLINE("M5StickC connected to Wi-Fi");
  i2sInit();
  //xTaskCreatePinnedToCore(mic_record_task,"mic_record_task",2048,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(mic_record_task,"mic_record_task",8192,NULL,1,NULL,1);
}

void loop() {
}