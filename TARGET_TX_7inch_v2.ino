/*
* Electric Target v4
* Tx ... ESP32-8048S070 Yellow Board 7.0"LCD 800x480 16bit parallel 
* Rx ... Tamamoni #2 V9
* ESP-NOW
*
* TARGET
* ESP32 GPIO 18/Serial1 Rx <-- UART2 Tx (Pickit 4)....着弾データ
*   -S3 GPIO 17/Serial1 Tx --> UART2 Rx (Pickit 5)....タマモニからのコマンド(スクリーン操作)
*       GPIO 12/PT4 input P.U (<-- TARGET PT4)..........着弾タイミング信号       
*                                   (H:idle L:on)
*       GPIO 20/WiFi Tx Rx --> LED Yellow
*       GPIO 38/Pairing    --> LED Blue Tamamoni pairing
*   2023.02.12
*
*
* TAMAMONI
* ESP32 GPIO 16/Serial2 Rx <--- Tamamoni pin.3 UART2 TX2....コマンド(スクリーン操作)
*       GPIO 17/Serial2 Tx ---> Tamamoni pin.4 UART2 RX2....着弾データ
*       GPIO 13/WiFi Tx Rx ---> LED Yellow
*       GPIO  4/PT4_WIFI out -> Tamamoni pin.45 GPIO/CLCIN2 PT4_WIFI...着弾タイミング信号
*                           |-> LED Blue Target pairing     
*       GPIO  2/PT4 output ---> LED Green
*   2023.01.29
*
*   2023.02.12  ver.0.10  while(!sendFlag)のところ、空ループだとうまくいかないよう -> グローバルなフラグをvolatileで宣言でOKみたい
*                         delayMicroseconds(500)をいれた
*                         ペアリングチェック...タマモニからの受信が約7秒ない時ペアリングフェイル
*   2023.02.28  ver.2.00  1.9"LCDへも送信
*   2023.03.04  ver.2.01  着弾点　赤->明るいオレンジ 　弾痕　黄色->濃いグレイ
*   2023.03.12  ver.2.02  PT4input GPIO19->12変更。（GPIO19がつねにHighになってしまった）
*   2023.03.18  ver.2.10  コマンド追加　ターゲット縦位置、エイムポイント寸法、バックライト明るさ
*   2023.03.19  ver.2.11  オフセット方向修正
*
*
*-- バグ -----------------------------
*起動時の最初だけ、ペアリングがないときに　変なデータが入って表示してしまう
*
*/

//=====================================================================
//  HARD             : ESP32_8048S070 YELLOW BOARD
//  Display          : 7.0" 800x480 RGB LCD
//  Dev environment  : Arduino IDE 2.0.3
//  Board Manager    : arduino-esp32 2.0.6

//  Board            : "ESP32S3 Dev Module"
//  port             : "/dev/cu.wchusbserial-1430"

//  USB CDC On Boot  : "Disable"
//  CPU Frequency    : "240MHz (WiFi)"
//  Core Degug Level : "None"
//  USB DFE On Boot  : "Disable"
//  Erase All Flash before Sketch Upload : "Disable"
//  Events Run On    : "Core 1"
//  Flash Mode       : "QIO 120MHz"
//  Flash Size       : "16MB (128Mb)"
//  JTAG Adapter     : "Integrated USB JTAG"
//  Arduino Runs On  : "Core 1"
//  USB Firmware MSC On Boot : "Disable"
//  Partition Scheme : "16MB Flash (2MB APP/12.5MB FATFS)"
//  PSRAM            : "OPI PSRAM"
//  Upload Mode      : "UART0 / Hardware CDC"
//  Upload Speed     : "921600"
//  USB Mode         : "Hardware CDC and JTAG"

//  書き込み装置       : "Esptool"
//=====================================================================

#pragma GCC optimize("Ofast")
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "lgfx_8048S070.h"
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>


//pin
#define PIN_TX 17  //UART1 (default)
#define PIN_RX 18  //UART1
#define PT4 12     //Target PT4input
#define LED_YL 38  //LED Yellow WiFi indicator
#define LED_BL 20  //LED Blue Pairing

//impact_plot_graph呼び出し時の引数
#define RESET_NONE 0
#define RESET_DONE 1

//Global
uint8_t tmp[250];
volatile bool sendFlag = 0;  //割り込みがかかわる場合volatileしておかないと値が乱れることがある
volatile bool receiveFlag = 0;
volatile bool pairFlag = 0;
volatile bool pt4Flag = 0;  //ターゲットからの着弾信号
volatile uint32_t timePt4in = 0;

uint16_t scoreTotal = 0;
uint16_t scoreX = 0;


//PT4 ターゲット ~ タマモニ　遅延時間測定テスト ---------------------------------------------------------- TEST macro
// タマモニ10P-7とターゲットESP32-GPIO11をつなぐ　GNDもつなぐのを忘れないこと
#define PT4_IMPACT_DELAY_TEST 0  //test:1
#if PT4_IMPACT_DELAY_TEST
#define PT4_TAMA 11  //PT4 @Tamamoni
volatile uint32_t timePt4tama = 0;
#endif
// debug 各所のタイミング測定
#define DEBUG_TIMING 1  //debug:1 ----------------------------------------- DEBUG macro
// ESP-NOW callback DEBUG
#define DEBUG_ESPNOW 1  //debug:1 ----------------------------------------- DEBUG macro

//WiFi
esp_now_peer_info_t slaves1 = {};
esp_now_peer_info_t slaves2 = {};
esp_now_peer_info_t slaves3 = {};
esp_now_send_status_t txResult;
//通信相手のMACアドレスを設定
#define PEER_NUM 7  //受信表示器の数
uint8_t s_addr[PEER_NUM][6] = {
  { 0x34, 0x85, 0x18, 0x8F, 0x7D, 0x30 },  //0＊黄色1.9"LCDタッチ無しボード ESP32-S3-WROOM-1
  { 0x1C, 0x9D, 0xC2, 0x52, 0xE3, 0x0c },  // 　秋月緑基板 ESP32-WROOM-32E
  { 0xB8, 0xD6, 0x1A, 0x0E, 0x34, 0xC4 },  // 　黒パチDEVボード ESP32-WROOM-32E
  { 0xB8, 0xD6, 0x1A, 0x0E, 0x35, 0xF0 },  // 　JTAG最小基板 ESP32-WROOM-32E
  { 0xB8, 0xD6, 0x1A, 0x0E, 0x4B, 0x50 },  //4＊タマモニ ESP32-WROOM-32E
  { 0xDC, 0x54, 0x75, 0xC8, 0x7F, 0xF4 },  //5* M5stampS3 ESP32-S3 送信テスト
  { 0xF4, 0x12, 0xFA, 0xE2, 0x83, 0x68 },  // 　黄色7"LCDタッチ無しボード ESP32-S3-WROOM-1
};


// sub -------------------------------------------------------------
//ESP-NOW WiFi
void onSend(const uint8_t* mac_addr, esp_now_send_status_t status) {
  //送信後のコールバック
  char macStr[18];
  digitalWrite(LED_YL, HIGH);
  txResult = status;

#ifdef DEBUG_ESPNOW
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Tx to   %s  ", macStr);
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Success   ");
  } else {
    Serial.println("Failed    ");
  }
#endif
  digitalWrite(LED_YL, LOW);
  sendFlag = 1;
}


void onReceive(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
  //ESP-NOW受信　コールバック
  char macStr[18];
  //data
  digitalWrite(LED_YL, HIGH);
  memcpy(tmp, data, data_len);  //data -> temp へコピー
  tmp[data_len] = 0;            //文字列のエンドマーク
#ifdef DEBUG_ESPNOW
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Rx from %s  ", macStr);
  Serial.printf("Data(%d)-'", data_len);
  Serial.printf("%s'\n", tmp);
#endif
  receiveFlag = 1;
  digitalWrite(LED_YL, LOW);
}


uint8_t pairing_check(uint8_t cnt) {
  //ターゲットとのペアリング確認
  //cnt:確認時間　x 1秒
  //戻り値: 0:成功, 1:不可
  const uint8_t test[] = "test,";
  uint8_t i = 0;

  while (i < cnt) {
    Serial.printf("Pairing check(%d)- ", i);
    esp_err_t esp_stat = esp_now_send(slaves1.peer_addr, test, 5);  //タマモニへテスト送信　esp-nowの接続の状態
    while (!sendFlag) {}                                            //送信完了待ち(約1ms)
    sendFlag = 0;

    if (txResult == ESP_NOW_SEND_SUCCESS) {
      //送信完了=ペアリングOK
      Serial.println("Tamamoni pairing success");
      digitalWrite(LED_BL, HIGH);
      //LCD
      tft.setCursor(0, 15);
      tft.setTextSize(1);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.print("Tamamoni Pairing     ");
      return 0;
    }
    //ペアリング解除されている
    Serial.println("NO Pair!!!");
    //LCD
    tft.setCursor(0, 15);
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("NO Pair!!!            ");
    delay(1000);
    i++;
  }
  return 1;
}


//interrupt
void IRAM_ATTR pt4_impact_isr(void) {
  //着弾信号を割り込みでキャッチ
  timePt4in = micros();
  pt4Flag = 1;
  //Serial.println//////割り込みルーチン中にSerial.printは危険。リセットがかかるようだ。
}

#if PT4_IMPACT_DELAY_TEST
void IRAM_ATTR pt4_tama_isr(void) {
  //タマモニの着弾信号認識を割り込みでキャッチ　ディレイ計測実験
  timePt4tama = micros();
}
#endif


// main --------------------------------------------------------------------------
void setup() {
  //pin
  pinMode(LED_BL, OUTPUT);
  pinMode(LED_YL, OUTPUT);
  pinMode(PT4, INPUT_PULLUP);
  digitalWrite(LED_BL, LOW);
  digitalWrite(LED_YL, HIGH);
#if PT4_IMPACT_DELAY_TEST
  pinMode(PT4_TAMA, INPUT_PULLUP);
#endif
  Serial.begin(115200);  //シリアルモニタ用
  while (!Serial) {
    //ポートを開くまでの待ち
  }
  //Title
  Serial.println();
  Serial.printf("******************************\n");
  Serial.printf("* Electric Target            *\n");
  Serial.printf("*  7.0inch LCD + ESP32S3     *\n");
  Serial.printf("*    ESP-Now (2.4GHz WiFi)   *\n");
  Serial.printf("******************************\n");
  //LCD init
#define LCD_BRIGHTNESS 90
  tft.init();
  tft.setBrightness(LCD_BRIGHTNESS);    //250...0.59A, 200...0.50A, 100...0.34A, 90...0.33A, 80...0.31A, 50...0.27A　やや暗い, 30...0.26A, 0...0.23A
  tft.setRotation(1);
  tft.setColorDepth(24);
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  //LCD title
  tft.setCursor(0, 0);
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN);
  tft.setTextWrap(false);  //改行しない
  tft.printf("*** Electric Target 7inch  ESP-NOW --> Tamamoni Rx ***");
  target_graph_initialize();

  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);  //ターゲットよりデータ受信用  ピン指定
  while (!Serial1) {
    //ポートを開くまでの待ち
  }
  delay(800);
  digitalWrite(LED_YL, LOW);
  //スタート合図
  delay(200);
  digitalWrite(LED_YL, HIGH);
  delay(100);
  digitalWrite(LED_YL, LOW);
  delay(200);
  digitalWrite(LED_YL, HIGH);
  delay(100);
  digitalWrite(LED_YL, LOW);

  //WiFi ESP-NOW init
  Serial.print("MAC address ");
  Serial.println(WiFi.macAddress());  //自機アドレス
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Init Success");
  } else {
    Serial.println("ESP-NOW Init Failed!!!");
    delay(3000);
    ESP.restart();
  }
  //送信先設定
  //peer1
  esp_err_t addStatus;
  memcpy(slaves1.peer_addr, &s_addr[4], 6);
  slaves1.channel = 0;
  slaves1.encrypt = false;
  addStatus = esp_now_add_peer(&slaves1);
  if (addStatus == ESP_OK) {
    Serial.printf("Peer1 set success\n");
  } else {
    Serial.printf("Peer1 set failed!!!\n");
    //Serial.println("@@@@@ ESP32 RESTART @@@@@@@");
    //delay(3000);
    //ESP.restart();  //////////////////////
  }
  //peer2
  memcpy(slaves2.peer_addr, &s_addr[0], 6);
  slaves2.channel = 0;
  slaves2.encrypt = false;
  addStatus = esp_now_add_peer(&slaves2);
  if (addStatus == ESP_OK) {
    Serial.printf("Peer2 set success\n");
  } else {
    Serial.printf("Peer2 set failed!!!\n");
    //delay(3000);
  }
  //peer3_test
  memcpy(slaves3.peer_addr, &s_addr[5], 6);
  slaves3.channel = 0;
  slaves3.encrypt = false;
  addStatus = esp_now_add_peer(&slaves3);
  if (addStatus == ESP_OK) {
    Serial.printf("Peer3(test) set success\n");
  } else {
    Serial.printf("Peer3(test) set failed!!!\n");
    //delay(3000);
  }

  //コールバックの定義
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);

  //タマモニとペアリングチェック
  uint8_t pa = pairing_check(1);  //1回...ペアリングしなくてもターゲット表示動作をさせるため
  if (pa == 0) {
    //ペアリング成功
    //Serial.printf("OK!\n");
    pairFlag = 1;
  } else {
    //ペアリング失敗
    //Serial.printf("\nNo Pair\n");
    pairFlag = 0;
  }

  //Interrupt
  attachInterrupt(PT4, pt4_impact_isr, FALLING);  //target着弾信号割込
  delay(100);
  pt4Flag = 0;
#if PT4_IMPACT_DELAY_TEST
  attachInterrupt(PT4_TAMA, pt4_tama_isr, FALLING);  //tamamoni着弾信号割込
#endif
}


void loop() {
#define RX_LEN 80                                        //受信文字数
#define BAUDRATE 115200                                  //通信ボーレート
#define RX_TIMEOUT (float)RX_LEN * 10 / BAUDRATE * 1000  //受信タイムアウトmsec　1文字10ビット

  const uint8_t pt[] = "!";                 //PT4オンの時にWiFiへ出力する文字
  static uint32_t pairing_check_timer = 0;  //ペアリングチェックタイマー
  esp_err_t ans;
  uint8_t le;
  uint16_t i;
  uint8_t score;

  //[優先]PT4着弾信号
  if (pt4Flag == 1) {
    ans = esp_now_send(slaves1.peer_addr, pt, 1);
    uint32_t timeSend = micros();
    Serial.printf("Target PT4 on '!' --> WiFi  ");
    while (!sendFlag) {}  //送信完了待ち(約1ms)
    sendFlag = 0;

    uint32_t timeNow = micros();
    Serial.printf("PT4 -> WiFi send: %d usec  ", timeSend - timePt4in);  //PT4 in ~ send
    Serial.printf("-> complete: %d usec  ", timeNow - timeSend);         //send ~ "!"送信完了までの時間　約1200~1300usec
#if PT4_IMPACT_DELAY_TEST
    Serial.printf("Target PT4 -> Tamamoni Rx comp: %d usec ", timePt4tama - timePt4in);    //PT4 target ~ tamamoni 遅延時間測定テスト
    Serial.printf("[delta t = %d usec]", timeNow - timeSend - (timePt4tama - timePt4in));  //差は一定というわけでもない???
#endif
    Serial.println();
    pairing_check_timer = 0;
    pt4Flag = 0;
  }

  //target data read　<-- serial1
  String str;
  const char endMark = ',';
  if (Serial1.available()) {
    uint32_t timeSerial1RxStart = micros();
    Serial1.setTimeout(RX_TIMEOUT);
    //str = Serial1.readString();   //実行速度が遅い。たぶん文字列の終了判断に時間がかかる
    str = Serial1.readStringUntil(endMark);
    str.toCharArray((char*)tmp, RX_LEN);

#if DEBUG_TIMING
    uint32_t timeDataRxComp = micros();
#endif
    //LCD 7"
    tft.setTextWrap(false);   //テキスト折り返さない
    tft.setCursor(0, 30);
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.printf("Target --> Rx '%s'                                   ", tmp);  //受信した生データを画面に表示
#if DEBUG_TIMING
    uint32_t timeLcdPrintComp = micros();
    uint32_t timeCalcComp = timeLcdPrintComp;
    uint32_t timePlotComp = timeLcdPrintComp;
    uint32_t timeCommandCheck = timeLcdPrintComp;
    uint32_t timeEspNowTxComp = timeLcdPrintComp;
#endif
    //着弾データ読み取り
    float impactData[3] = { 999.99, 999.99, 0.00 };
    uint8_t ans = data_uart_calc((char*)tmp, impactData);
    //Serial.printf("ans = %d\n", ans);
    if (ans < 3) {
      //0..OK, 1..計算エラー, 2..数値化失敗, 3..データ形式が違う(コマンドかも)
      //0,1,2はWiFi送信する
#if DEBUG_TIMING
      timeCalcComp = micros();
#endif
      //着弾表示
      score = impact_plot_graph(impactData, RESET_NONE);
      //ターゲットオフセット分を補正した数値が入って戻ってくる
      //WiFi送信するデータを書き換え
      //Serial.printf("Rx data = '%s'  ", tmp);
      sprintf((char*)tmp, "BINX0Y0dT %8.3f %8.3f %8.4f END ", impactData[0], impactData[1], impactData[2]);  //最後のコンマは付けない
                                                                                                             //Serial.printf("+ offset = '%s'\n", tmp);

#if DEBUG_TIMING
      timePlotComp = micros();
#endif

      if (pairFlag == 1) {  //ペアリングしている時だけ送信
        delay(100);         //UART衝突回避のため???のディレイ
        uint8_t len = strlen((char*)tmp);
        tmp[len] = endMark;  //endMarkで切られるので追加
        tmp[len + 1] = 0;
        Serial.printf("Serial1 -> Rx: '%s' --> WiFi \n", tmp);  //serial monitor
        //データを送信
        ans = esp_now_send(slaves1.peer_addr, tmp, strlen((char*)tmp));  //Wifi --> tamamoni
        ans = esp_now_send(slaves2.peer_addr, tmp, strlen((char*)tmp));  //     --> 1.9in Yellow Display
        while (!sendFlag) {}                                             //送信完了待ち(約1ms)
        sendFlag = 0;
      }
#if DEBUG_TIMING
      timeEspNowTxComp = micros();
      delay(20);  ///////////////////////
      Serial.printf("PT4in -> Target data Rx start:     %8d usec\n", timeSerial1RxStart - timePt4in);
      Serial.printf("      -> Target data Rx complete:  %8d usec\n", timeDataRxComp - timeSerial1RxStart);
      Serial.printf("      -> LCD print complete:       %8d usec\n", timeLcdPrintComp - timeDataRxComp);
      Serial.printf("      -> Data calc complete:       %8d usec\n", timeCalcComp - timeLcdPrintComp);
      Serial.printf("      -> LCD plot complete:        %8d usec\n", timePlotComp - timeCalcComp);
      Serial.printf("      -> ESP-NOW data Tx complete: %8d usec\n", timeEspNowTxComp - timePlotComp);
#endif

    } else {
      //3..着弾データではなかった時
      //コマンドが含まれていないかチェック(UART)　　　　///////////////LCD表示しない、WiFi送信しない_____未
      Serial.print("LAN -> ");
      tamamoniCommandCheck((char*)tmp);
    }

    pairing_check_timer = 0;
  }

  //タマモニとのペアリング確認 <- WiFi ESP-NOW
  if (receiveFlag == 1) {
    //Serial.printf("%s'\n", tmp);
    tamamoniCommandCheck((char*)tmp);  //コマンドが含まれていないかチェック

    if (pairFlag == 0) {
      pairFlag = 1;
      Serial.println("Pairing success");
      digitalWrite(LED_BL, HIGH);
      tft.setCursor(0, 15);
      tft.setTextSize(1);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.print("Tamamoni or(and) Mini Disp  Pairing           ");
    }
    receiveFlag = 0;
    pairing_check_timer = 0;
  }
  pairing_check_timer++;
  if (pairing_check_timer > 0x0100000) {
    //約5秒受信がない->ペアリングフェイル
    Serial.println("NO Pair!!!");
    digitalWrite(LED_BL, LOW);
    tft.setCursor(0, 15);
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("NO Pair!!!                               ");
    pairFlag = 0;
    pairing_check_timer = 0;
  }
}


// target data calc ------------------------------------------------------------------------------------------
uint8_t data_uart_calc(char* tmp_str, float* data) {
  //電子ターゲットからの着弾データを解読
  uint8_t num_scan = sscanf(tmp_str, "BINX0Y0dT %f %f %f END", &data[0], &data[1], &data[2]);
  //ヘッダ、フッタが一致しない場合、値を代入した数が合わなくなる
  //Serial.printf("num_scan = %d \n", num_scan);
  if (num_scan == 0) {
    //着弾データではない時
    return 3;
  } else if (num_scan != 3) {
    //着弾データ数値化でエラー
    return 2;
  } else if (data[2] == 0) {
    //計算データがエラーの時
    return 1;
  }
#if DEBUG_SSCANF_CHECK
  Serial.printf("Target data read  ");
  Serial.printf("n = %d  ", num_scan);
  Serial.printf("x: %f(float)  ", data[0]);
  Serial.printf("y: %f(float)  ", data[1]);
  Serial.printf("dt:%f(float) ... OK!\n", data[2]);
#endif
  return 0;
}


// target -----------------------------------------------------------------------------------------------------------------

//APSターゲット [mm]
#define D_NUM 5        //円の数
#define APS_D_X 11.2   //円寸法mm
#define APS_D_10p1 20  //10点内側
#define APS_D_10 22    //10点外側
#define APS_D_8 35
#define APS_D_5 50
#define APS_WIDTH 91                                            //横幅
#define APS_HIGHTU 67                                           //上側
#define APS_HIGHTD 61                                           //下側
#define SCORE_X_10 ((APS_D_10p1 - APS_D_X) / 4 + APS_D_X / 2)   //点数表示中心位置
#define SCORE_X_8 ((APS_D_8 - APS_D_10) / 4 + APS_D_10 / 2)
#define SCORE_X_5 ((APS_D_5 - APS_D_8) / 4 + APS_D_8 / 2)
#define U_LINE_X0 3.5  // 中心振り分け
#define U_LINE_X1 36
#define U_LINE_Y 50
//aimline [mm]
#define LINE_NUM 6
#define AIM_LINE_1 50
#define AIM_LINE_2 60
#define AIM_LINE_3 70
#define AIM_LINE_4 80
#define AIM_LINE_5 90
#define AIM_LINE_6 100
//玉 [mm]
#define BB 6.0       //着弾円寸法
#define BB_AREA 3.2  //点数判定用半径
//
//scale [pixel <--> mm]
#define PIXEL_W 0.1790                //mm/pixel
#define PIXEL_H 0.1926                //mm/pixel
#define SCALE_W (float)(1 / PIXEL_W)  //pixel/mm
#define SCALE_H (float)(1 / PIXEL_H)  //pixel/mm
//
//LCD [pixel]
#define DISPLAY_WIDTH 480   //pixel
#define DISPLAY_HEIGHT 800  //pixel
//screen [pixel]
#define TARGET_X0 (DISPLAY_WIDTH / 2)   //グラフ原点0位置 pixel
#define TARGET_Y0 (DISPLAY_HEIGHT / 2)  //オフセット無しの時の原点y
#define TARGET_X_MIN 0
#define TARGET_X_MAX 480
#define TARGET_Y_MIN 38
#define TARGET_Y_MAX 800


//オフセット [mm]   移動できるように変数に代入
float targetX0offset = 0.0;   //マト表示横方向のオフセット　＋：右へ
float targetY0offset = -15.0;  //マト表示高さのオフセット   ＋：上へ
float aimPointY = 74.0;       //狙点高さ
//offset [pixel]
int16_t targetX0, targetY0;                  //ターゲット中心
int16_t apsXmin, apsXmax, apsYmin, apsYmax;  //ターゲット紙のサイズ
int16_t aimY;                                //エイムポイント高さ

void target_graph_initialize(void) {
  //ターゲット画面の初期表示
  //1ラウンドの間、維持。
  uint8_t i;

  float target_circle[D_NUM] = {
    APS_D_5,
    APS_D_8,
    APS_D_10,
    APS_D_10p1,
    APS_D_X
  };
  bool circle_fill_black[D_NUM][2] = { 0, 1, 0, 1, 1, 1, 1, 0, 0, 1 };  //fill, BLACK

  //ターゲットの計算
  //offset [pixel]
  targetX0 = (int16_t)(TARGET_X0 + targetX0offset * SCALE_W);
  targetY0 = (int16_t)(TARGET_Y0 - targetY0offset * SCALE_H);
  //APSマト紙 [pixel]
  apsXmin = targetX0 - SCALE_W * (APS_WIDTH / 2);
  if (apsXmin < TARGET_X_MIN) {
    apsXmin = TARGET_X_MIN;
  }
  apsXmax = targetX0 + SCALE_W * (APS_WIDTH / 2);
  if (apsXmax > TARGET_X_MAX) {
    apsXmax = TARGET_X_MAX;
  }
  apsYmin = targetY0 - SCALE_H * APS_HIGHTU;
  if (apsYmin < TARGET_Y_MIN) {
    apsYmin = TARGET_Y_MIN;
  }
  apsYmax = targetY0 + SCALE_H * APS_HIGHTD;
  if (apsYmax > TARGET_Y_MAX) {
    apsYmax = TARGET_Y_MAX;
  }

  //クリア
  target_clear_screen();
  //紙枠
  //Serial.printf("x0:%6d y0:%6d\n", targetX0, targetY0);
  //Serial.printf("APS x min:%6d x max:%6d  y min:%6d y max:%6d \n",apsXmin, apsXmax, apsYmin, apsYmax);
  tft.fillRect(apsXmin, apsYmin, (apsXmax - apsXmin), (apsYmax - apsYmin), TFT_WHITE);
  //同心円
  uint16_t color;
  for (i = 0; i < D_NUM; i++) {
    //color
    if (circle_fill_black[i][1] == 1) {
      color = TFT_BLACK;
    } else {
      color = TFT_WHITE;
    }
    //radius
    uint16_t rx = target_circle[i] / 2 * SCALE_W;
    uint16_t ry = target_circle[i] / 2 * SCALE_H;
    //Serial.printf("circle:%4.1f  rx:%3d, ry:%3d  ", target_circle[i], rx, ry);
    //circle
    if (circle_fill_black[i][0] == 1) {
      tft.fillEllipse(targetX0, targetY0, rx, ry, color);
    } else {
      tft.drawEllipse(targetX0, targetY0, rx, ry, color);
    }
  }
  //軸
  tft.drawFastVLine(targetX0, TARGET_Y_MIN, DISPLAY_HEIGHT, TFT_DARKGREY);
  tft.drawFastHLine(TARGET_X_MIN, targetY0, DISPLAY_WIDTH, TFT_DARKGREY);
  //狙線
  tft.setFont(&fonts::Font2);
  tft.setTextDatum(middle_right);  //表示位置座標指定を中心に
  tft.setTextColor(TFT_DARKGREY);  //重ね書き
  tft.setTextSize(1);
  char s[10];
  uint16_t ya;

  for (i = 30; i < 100; i += 10) {  //////10mmごと100mmまで目盛る
    ya = targetY0 - i * SCALE_H;
    if (ya > TARGET_Y_MIN) {
      tft.drawFastHLine(targetX0 - 10, ya, 20, TFT_DARKGREY);  //目盛
      sprintf(s, "%3d", i);
      tft.drawString(s, targetX0 - 15, ya);  //数字
    }
  }
  //エイムポイント
#define AIM_POINT_R 1  //mm
  aimY = targetY0 - aimPointY * SCALE_H;
  if (aimY > TARGET_Y_MIN){
    uint16_t aimColor;
    if (aimPointY > APS_HIGHTU){
      aimColor = TFT_WHITE;
    }else if (aimPointY < APS_HIGHTU){
      aimColor = TFT_BLACK;
    }else{
      aimColor = TFT_DARKGRAY;
    }
    tft.fillEllipse(targetX0, aimY, AIM_POINT_R * SCALE_W, AIM_POINT_R * SCALE_H, aimColor);
    tft.drawFastHLine(targetX0 - 40, aimY, 80, aimColor);  //目盛
    sprintf(s, "%3d", (int8_t)aimPointY);
    tft.setTextDatum(middle_left);            //表示位置座標指定を中心に
    tft.setTextColor(aimColor);               //重ね書き
    tft.drawString(s, targetX0 + 45, aimY);   //数字
  }

  //点数
  tft.setFont(&fonts::Font4);
  tft.setTextDatum(middle_center);  //表示位置座標指定を中心に
  tft.setTextColor(TFT_BLACK);      //重ね書き
  tft.setTextSize(1.2, 1.0);
  tft.drawString("X", targetX0, targetY0 + 4);
  tft.setTextSize(0.8, 1);
  tft.drawString("10", targetX0 - SCORE_X_10 * SCALE_W, targetY0 + 3);
  tft.setTextSize(1);
  tft.drawString("8", targetX0 - SCORE_X_8 * SCALE_W, targetY0 + 3);
  tft.drawString("5", targetX0 - SCORE_X_5 * SCALE_W, targetY0 + 3);
  //フォント設定を元に戻す
  tft.setFont(&Font0);
  tft.setTextSize(1);
  tft.setTextDatum(top_left);
  //名前、点数ライン
  tft.drawFastHLine(targetX0 - (U_LINE_X1 * SCALE_W), targetY0 + U_LINE_Y * SCALE_H, (U_LINE_X1 - U_LINE_X0) * SCALE_W, TFT_BLACK);
  tft.drawFastHLine(targetX0 + (U_LINE_X0 * SCALE_W), targetY0 + U_LINE_Y * SCALE_H, (U_LINE_X1 - U_LINE_X0) * SCALE_W, TFT_BLACK);

  //点数リセット
  scoreTotal = 0;
  scoreX = 0;
}


void target_clear_screen(void) {
  //クリアスクリーン
  tft.fillRect(TARGET_X_MIN, TARGET_Y_MIN, DISPLAY_WIDTH, (TARGET_Y_MAX - TARGET_Y_MIN), TFT_BLACK);
  //受信データ表示部
  tft.setCursor(0, 30);
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  sprintf((char*)tmp, "                                                            ");
  tft.printf("%s", tmp);
}


void target_reset(void) {
  //ターゲットクリア
  float dummy[3];

  impact_plot_graph(dummy, RESET_DONE);  //直前データをリセット
  target_graph_initialize();
}


uint8_t impact_plot_graph(float* data, bool reset) {
  //着弾点表示
  static float x = 99999;  //初期データは画面外にして表示させない
  static float y = 99999;
  uint8_t score = 0;

  //ターゲットリセット時
  if (reset == RESET_DONE) {
    x = 99999;
    y = 99999;
    return score;
  }

  //前回の着弾を濃いグレイに
#define TFT_DARKDARKGRAY 0x1082
  draw_impact_point(x, y, TFT_DARKDARKGRAY);

  x = data[0] - targetX0offset;  //前回値として保存される
  y = data[1] - targetY0offset;
  //オフセットを含めたデータに書き換え
  data[0] = x;
  data[1] = y;

  //着弾を赤
  draw_impact_point(x, y, TFT_RED);

  //点数の計算
  float d = 2 * sqrt(x * x + y * y) - BB_AREA;  //(マト中心からの距離+BB)x2  直径で比較するため
  Serial.printf("BB(area%3.1fmm)  r:%8.3fmm -- ", (float)BB_AREA, d / 2);
  if (d < APS_D_X) {
    score = 10;
    scoreX++;
  } else if (d < APS_D_10) {
    score = 10;
  } else if (d < APS_D_8) {
    score = 8;
  } else if (d < APS_D_5) {
    score = 5;
  }
  scoreTotal += score;
  //Serial.printf("score:%2dpoint  ", score);
  //Serial.printf("total score:%3d\n", scoreTotal);

  //LCD
//ターゲット中心からの位置 [mm]
#define SCORE_X 18
#define SCORE_Y -65
#define TOTAL_X 9
#define TOTAL_Y 40
//文字の高さ[pixel]
#define FONT_H 40
  int16_t locX, locY;

  tft.setFont(&fonts::Font4);
  tft.setTextSize(2);
  locX = TARGET_X0 + SCORE_X * SCALE_W;  //オフセットによらず左右位置は一定に
  locY = targetY0 + SCORE_Y * SCALE_H;
  if (locY < TARGET_Y_MIN) {
    locY = TARGET_Y_MIN;
  }
  tft.setCursor(locX, locY);
  tft.setTextColor(TFT_BLUE, TFT_WHITE);
  tft.printf("%2dp     ", score);

  locX = TARGET_X0 + TOTAL_X * SCALE_W;  //オフセットによらず左右位置は一定に
  locY = targetY0 + TOTAL_Y * SCALE_H;
  if (locY > TARGET_Y_MAX - FONT_H) {
    locY = TARGET_Y_MAX - FONT_H;
  }
  tft.setCursor(locX, locY);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.printf("%3d -%2dX", scoreTotal, scoreX);

  tft.setFont(&Font0);
  tft.setTextSize(1);

  return score;
}


void draw_impact_point(int16_t x, int16_t y, uint16_t color) {
  //着弾点を描写
  //スコアを帰り値

  //LCD上の座標[pixel]
  uint16_t draw_x = (uint16_t)(targetX0 + SCALE_W * x);  //LCD x+ → 着弾 x+ →
  uint16_t draw_y = (uint16_t)(targetY0 - SCALE_H * y);  //LCD y+ ↓ 着弾 y+ ↑

  //表示範囲からはみだすかのチェック
  if ((draw_x > TARGET_X_MAX) || (draw_x < TARGET_X_MIN)) {
    //画面外
    return;
  }
  if ((draw_y > TARGET_Y_MAX) || (draw_y < TARGET_Y_MIN)) {
    //画面外
    return;
  }
#if DEBUG001
  if (color == TFT_RED) {
    Serial.printf("LCD  x:%4d y:%4d\n", draw_x, draw_y);  //LCD上の座標[pixel]
  }
#endif
  //着弾点[pixel]
  uint16_t rx = (float)BB / 2 * SCALE_W;  //BBの半径（横）
  uint16_t ry = (float)BB / 2 * SCALE_H;  //BBの半径（縦）
  tft.fillEllipse(draw_x, draw_y, rx, ry, color);
  //縁どり
  if ((draw_x < apsXmin) || (draw_x > apsXmax) || (draw_y < apsYmin) || (draw_y > apsYmax)) {
    //APSマト紙外(黒バック)
    color = TFT_DARKGRAY;
  } else {
    //APSマト紙内(白バック)
    color = TFT_LIGHTGRAY;
  }
  tft.drawEllipse(draw_x, draw_y, rx, ry, color);

  return;
}


// Tamamoni TARGET Command ------------------------------------------------------------------------------------------
void tamamoniCommandCheck(char* tmp_str) {
  //タマモニからの指令を確認し実行
  char command[10] = { 0 };  //9文字まで
  char clear[] = "CLEAR";
  char reset[] = "RESET";
  char offset[] = "OFFSET";
  char aimpoint[] = "AIMPOINT";
  char brightness[] = "BRIGHT";
  float val = 0;

  uint8_t num = sscanf(tmp_str, "TARGET_%s %f END", command, &val);   //valのところの数字は無くても正常に動くみたい
  if (num == 0) {
    //型が合わなかったとき
    return;
  }                                                                     
  //コマンド
  Serial.printf("Detect tamamoni command(%d) :%s   %f --- ", num, command, val);
  if (strcmp(clear, command) == 0) {
    Serial.println("target clear!");
    target_reset();  //ターゲットをクリア

  } else if (strcmp(reset, command) == 0) {
    Serial.println("** RESET ESP32 in 3 second **");
    //LCD
    tft.setTextDatum(top_center);  //表示位置座標指定を中心に
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString(" *** RESET ESP32 *** ", targetX0, 250);
    delay(3000);
    ESP.restart();  ////////////////////// RESET

  } else if (strcmp(offset, command) == 0) {
    targetY0offset = constrain(val, -40, 35);
    Serial.printf("target Y offset %5.1f \n", targetY0offset);
    target_reset();  //ターゲットを再描画

  } else if (strcmp(aimpoint, command) == 0) {
    Serial.println("aimpoint set");
    aimPointY = constrain(val, 30, 120);
    target_reset();  //ターゲットを再描画

} else if (strcmp(brightness, command) == 0) {
    Serial.println("LCD backlight brightness set");
    tft.setBrightness((uint8_t)constrain(val, 0, 250));

  } else {
    Serial.println("invalid command");
  }
  Serial1.readString();  //捨て読み
}
