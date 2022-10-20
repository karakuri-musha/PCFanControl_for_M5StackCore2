//--------------------------------------------------------------------------------
//  M5Stack Core2 冷却ファン制御　テストプログラム
//     M5Stack Core2 Cooling fan PWM control test program.
//  
//--------------------------------------------------------------------------------

// ------------------------------------------------------------
// ライブラリインクルード部 Library include section.
// ------------------------------------------------------------
#define LGFX_M5STACK_CORE2                                    // M5Stack (Basic / Gray / Go / Fire)
#define LGFX_AUTODETECT                                       // 自動認識 (D-duino-32 XS, PyBadge はパネルID読取りが出来ないため自動認識の対象から外れています)
#define LGFX_USE_V1

#include <M5Core2.h>                                          // M5StickC 用ライブラリ
#include "Screen_img.h"                                       // M5StickC 用ライブラリ

#include <LovyanGFX.hpp>                                      // 画面制御用ライブラリ（LovyanGFX）
#include <LGFX_AUTODETECT.hpp>                                // 画面制御用ライブラリ（LovyanGFX）

#include <FastLED.h>                                          // LED 制御用ライブラリ（FastLED）
FASTLED_USING_NAMESPACE

// GPIOのPIN番号指定
const uint8_t FAN_PWM_PIN     = 26;                           // ファン回転数制御用PWM制御用GPIO
const uint8_t FAN_SENSOR_PIN  = 19;                           // ファン回転数計算用GPIO
const uint8_t ARGB_LED_PIN    = 0;                            // ARGB対応FAN制御  ARGB compatible FAN system.

// 画面制御関連
int display_rotation = 1;                                     // 画面向きの初期化
int display_stat = 1;                                         // 画面表示切替用（1：FAN制御、2:LED点灯制御、3：ゲーム）

// LovyanGFXインスタンス生成
static LGFX lcd;                                              // LGFXのインスタンスを作成。
static LGFX_Sprite sprite1(&lcd);                             // メーター背面用スプライト作成
static LGFX_Sprite sprite2(&sprite1);                         // メーター前面用スプライト作成
static LGFX_Sprite sprite3(&sprite1);                         // デジタルメーター用スプライト作成
static LGFX_Sprite sprite4(&lcd);                             // led背面用スプライト作成
static LGFX_Sprite sprite5(&lcd);                             // led背面用スプライト作成

// 色指定
static int c_BLACK           = 0x0000;                        /*   0,   0,   0 */
static int c_NAVY            = 0x000F;                        /*   0,   0, 128 */
static int c_DARKGREEN       = 0x03E0;                        /*   0, 128,   0 */
static int c_DARKCYAN        = 0x03EF;                        /*   0, 128, 128 */
static int c_MAROON          = 0x7800;                        /* 128,   0,   0 */
static int c_PURPLE          = 0x780F;                        /* 128,   0, 128 */
static int c_OLIVE           = 0x7BE0;                        /* 128, 128,   0 */
static int c_LIGHTGREY       = 0xC618;                        /* 192, 192, 192 */
static int c_DARKGREY        = 0x7BEF;                        /* 128, 128, 128 */
static int c_BLUE            = 0x001F;                        /*   0,   0, 255 */
static int c_GREEN           = 0x07E0;                        /*   0, 255,   0 */
static int c_CYAN            = 0x07FF;                        /*   0, 255, 255 */
static int c_RED             = 0xF800;                        /* 255,   0,   0 */
static int c_MAGENTA         = 0xF81F;                        /* 255,   0, 255 */
static int c_YELLOW          = 0xFFE0;                        /* 255, 255,   0 */
static int c_WHITE           = 0xFFFF;                        /* 255, 255, 255 */
static int c_ORANGE          = 0xFD20;                        /* 255, 165,   0 */
static int c_GREENYELLOW     = 0xAFE5;                        /* 173, 255,  47 */
static int c_PINK            = 0xF81F;

// PWM制御用の定数　Constants for PWM control.
const uint8_t CH0       = 0;                                  // PWM送信チャネル PWM transmission channel.
const double  PWM_FREQ  = 25000;                              // PWM周波数 PWM frequency.
const uint8_t PWM_BIT   = 8;                                  // PWM分解能 PWM resolution.
int cur_duty_val = 0;                                         // 現在のデューティ比の格納（初期値：0）
String duty_str = "";                                         // デューティ比の表示用テキスト格納

// ファン回転数取得用定義
unsigned long lastPulse_T;                                    // 前回の割込み時の時間を格納
unsigned long pulse_Interval=0;                               // パルスのインターバルを格納
uint16_t rpm =0;                                              // １分中の回転数を格納
String rpm_str = "0000";                                      // rpmに表示する文字列の生成用
float  v_angle = -110.0;                                      // rpm表示針の表示角度格納用
uint16_t max_rpm = 2000;                                      // 測定ファンの最大回転数を設定

// ARGB対応FAN制御用　For ARGB compatible FAN control.
const uint8_t LED_COUNT = 18;                                 // 制御対象のLED数　Number of LEDs to be controlled.
const uint8_t DELAYVAL = 100;                                 // 処理遅延間隔 Processing delay interval.
const uint8_t FPS = 30;                                       // 周波数の指定 Frequency designation.
uint8_t gHue = 24;                                            // 色相指定値　Hue specified value.
uint8_t Led_select = 0;                                       // LED点灯モード選択状態格納用　For storing LED lighting mode selection status.
uint8_t Led_selected = 0;                                     // LED点灯モード選択結果格納用　For storing LED lighting mode selection results.
char* Led_pattern[] = {"OFF", "Single:RED",                   // LED表示方法の定義 Definition of LED display method.
                       "Single:PURPLE",
                       "Single:BLUE",
                       "Single:GREEN",
                       "ILLUMINATION",
                       "ROLLING"
                      };

CRGB leds[LED_COUNT];                                         // オブジェクト生成

// GAME制御用　GAME control.
int point_x0 = 160;                                           // 画面座標上の原点（x）の指定
int point_y0 = 120;                                           // 画面座標上の原点（y）の指定
int point_xc, point_yc;                                       // 画面座標上の現在位置の指定
int point_xs, point_ys;                                       // 画面座標上の現在位置の原点基準の対称点の指定
float v_PI = 3.141592653589793;


// ------------------------------------------------------------
// PWM出力用の関数 Function for PWM output.
// ------------------------------------------------------------
void Send_PWM(int cur_duty_val) {
  ledcWrite(CH0, cur_duty_val);                               // 指定されたデューティ比のPWM信号を送信
}

// ------------------------------------------------------------
// FAN回転パルス（立下り）検出用の関数 Function for FAN rotation pulse (falling) detection.
// ------------------------------------------------------------
void Fan_Rotate_Sence() {
  unsigned long cur = micros();                               // 割込み発生時の時間を格納
  unsigned long dif = cur - lastPulse_T;                      // 前回の割込み時間との差分
  pulse_Interval = (pulse_Interval - (pulse_Interval >> 2)) + (dif >> 2); // インターバルの計算
  lastPulse_T = cur;                                          // 前回の割込み時間の更新
}

// ------------------------------------------------------------
// ディスプレイ出力用の関数 Function for Display output.
// display_output()
// ------------------------------------------------------------
void display_output_duty(int cur_duty_val){

  if (cur_duty_val == 0) {                                    // デューティ比[０％]の場合の処理
    duty_str = "00" + String(cur_duty_val);                   // デューティ比表示用テキスト編集
  }
  else if (cur_duty_val == 25){                               // デューティ比[２５％]の場合の処理
    duty_str = "0" + String(cur_duty_val);                    // デューティ比表示用テキスト編集
  }
  else if (cur_duty_val == 50){                               // デューティ比[５０％]の場合の処理
    duty_str = "0" + String(cur_duty_val);                    // デューティ比表示用テキスト編集
  }
  else if (cur_duty_val == 75){                               // デューティ比[７５％]の場合の処理
    duty_str = "0" + String(cur_duty_val);                    // デューティ比表示用テキスト編集
  }
  else if (cur_duty_val == 100){                              // デューティ比[１００％]の場合の処理
    duty_str = String(cur_duty_val);                          // デューティ比表示用テキスト編集
  }
}

// ------------------------------------------------------------
// 回転速度表示　関数 
// ------------------------------------------------------------
void display_output_rpm(int rpm){
  
  if (rpm >= 0 && rpm <= 9) {                        
    rpm_str = "000" + String(rpm);                            // 回転数（rpm）表示用テキスト編集
  }
  else if (rpm >= 10 && rpm <= 99){
    rpm_str = "00" + String(rpm);                             // 回転数（rpm）表示用テキスト編集
  }
  else if (rpm >= 100 && rpm <= 999){
    rpm_str = "0" + String(rpm);                              // 回転数（rpm）表示用テキスト編集
  }
  else if (rpm >= 1000 && rpm <= 9999){
    rpm_str = String(rpm);                                    // 回転数（rpm）表示用テキスト編集
  }

  if (rpm == 0) {
    v_angle = -110.0;                                        // 回転数（rpm）表示針　初期状態（0）
  } else if (rpm <= max_rpm) {
    v_angle = (220*rpm/max_rpm)-110;                         // 回転数（rpm）表示針　回転数に応じた角度指定
  } else {
    v_angle = 110.0;                                         // 回転数（rpm）表示針　最大値を超えた状態（max）
  }
}

// ------------------------------------------------------------
// LED 点灯制御関数 
// ------------------------------------------------------------
void set_led(int Led_selected) {
  
  if (Led_selected == 0 ) {                                   // LED点灯：点灯OFF
    FastLED.clear();
    FastLED.show();
  }
  else if (Led_selected < 5) {                                // LED点灯：単色の順次点灯
    CRGB SetColored;
    if (Led_selected == 1) {
      SetColored = CRGB::Red;
    }
    else if (Led_selected == 2) {
      SetColored = CRGB::Purple;
    }
    else if (Led_selected == 3) {
      SetColored = CRGB::Blue;
    }
    else if (Led_selected == 4) {
      SetColored = CRGB::Green;
    }
    for (int i = 0; i < LED_COUNT; i++) {
      leds[i] = SetColored;
      FastLED.show();
    }
  }
  else if (Led_selected == 5) {                               // LED点灯：レインボーカラー
    fill_rainbow( leds, LED_COUNT, gHue, 24);
    FastLED.show();
    FastLED.delay(1000 / FPS);
    EVERY_N_MILLISECONDS( 800 ) {
      gHue++;
    }
  }
  else if (Led_selected == 6) {                               // LED点灯：アニメーション
    for (int i = 0; i < LED_COUNT; i++) {
      leds[i] = CRGB::Black;
      FastLED.show();
      delay(50);
    }
    for (int i = 0; i < LED_COUNT; i++) {
      leds[i] = CHSV(i * 10, 255, 255);
      FastLED.show();
      delay(50);
    }
    for (int i = 0; i < LED_COUNT; i++) {
      leds[i] = CRGB::Black;
      FastLED.show();
      delay(50);
    }
    for (int i = 0; i < LED_COUNT; i++) {
      leds[i] = CHSV(i * 10, 255, 255);
      FastLED.show();
      delay(50);
    }
  }
}

// ------------------------------------------------------------
// スプライト生成関数 （FAN画面）
// 各spriteのメモリ開放後にFAN画面用のスプライトを作成
// ------------------------------------------------------------
void create_sprite_fan() {
  sprite1.deleteSprite();                                     // スプライトメモリ開放
  sprite2.deleteSprite();
  sprite3.deleteSprite();
  sprite4.deleteSprite();
  sprite5.deleteSprite();
  sprite1.createSprite(screen_meter_img01Width, screen_meter_img01Height);  // sprite1作成
  sprite1.setPivot(0, 0);                                                   // sprite1基準点の設定
  sprite2.createSprite(screen_meter_img02Width, screen_meter_img02Height);  // sprite2作成
  sprite2.setPivot(10, 90);                                                 // sprite2基準点の設定
  sprite3.createSprite(150, 50);                                            // sprite3作成
  sprite3.setPivot(0, 0);                                                   // sprite3基準点の設定
  
}
// ------------------------------------------------------------
// スプライト生成関数 （Color画面）
// 各spriteのメモリ開放後にColor画面用のスプライトを作成
// ------------------------------------------------------------
void create_sprite_led() {
  sprite1.deleteSprite();                                     // スプライトメモリ開放
  sprite2.deleteSprite();
  sprite3.deleteSprite();
  sprite4.deleteSprite();
  sprite5.deleteSprite();
  sprite4.createSprite(led_screen_img01Width, led_screen_img01Height);      // sprite4作成
  sprite4.setPivot(0, 0);                                                   // sprite4基準点の設定
  sprite5.createSprite(100, 200);                                           // sprite5作成
  sprite5.setPivot(0, 0);                                                   // sprite5基準点の設定

}
// ------------------------------------------------------------
// スプライト生成関数 （GAME画面）
// 各spriteのメモリ開放後にGAME画面用のスプライトを作成
// ------------------------------------------------------------
void create_sprite_game() {
  sprite1.deleteSprite();                                     // スプライトメモリ開放
  sprite2.deleteSprite();
  sprite3.deleteSprite();
  sprite4.deleteSprite();
  sprite5.deleteSprite();
  sprite1.createSprite(led_screen_img01Width, led_screen_img01Height);      // sprite1作成
  sprite1.setPivot(0, 0);                                                   // sprite1基準点の設定
  sprite4.createSprite(game_go_img01Width, game_go_img01Height);            // sprite4作成
  sprite4.setPivot(0, 0);                                                   // sprite4基準点の設定
 
}

// ------------------------------------------------------------
// FAN 画面描画関数 
//  sprite1：メーターメモリと単位(rpm)、sprite2, sprite3をLcdへ描画
//  sprite2：メーター針をsprite1に描画
//  sprite3：デジタルメーターをsprite1に描画
// ------------------------------------------------------------
void draw_sprite_fan(String rpm_str, float v_angle) {
  sprite1.pushImage(0, 0, screen_meter_img01Width,screen_meter_img01Height, screen_meter_img01);
  sprite1.setCursor(150, 110, 2);                             // カーソル位置とフォントの設定
  sprite1.print("rpm");

  sprite2.pushImage(0, 0, screen_meter_img02Width,screen_meter_img02Height, screen_meter_img02);
  
  sprite3.fillRect(0, 0, 150, 50, c_BLACK);                   // 回転数表示域を塗りつぶし
  sprite3.setTextColor(c_GREEN, c_BLACK);                     // テキストカラーの指定
  sprite3.setCursor(10, 5, 7);                                // カーソル位置とフォントの設定
  sprite3.print(rpm_str);                                     // ディスプレイに表示（デューティ比）

  sprite2.pushRotateZoomWithAA(100, 98, v_angle, 1.0, 1.0, c_BLACK); // sprite1へ描画
  sprite3.pushRotateZoomWithAA(70, 105, 0.0, 0.5, 0.5);      // sprite1へ描画
  sprite1.pushRotateZoomWithAA(10, 30, 0.0, 1.5, 1.5);        // スプライトをLcdへ描画

}

// ------------------------------------------------------------
// LED 画面描画関数 
//  sprite4：ファン画像をLcdに描画
//  sprite5：Led点灯方法ボタン「単色」「虹」「アニメ」「OFF」をLcdへ描画
// ------------------------------------------------------------
void draw_sprite_led(int led_select) {
  sprite4.pushImage(0, 0, led_screen_img01Width,led_screen_img01Height, led_screen_img01);

  if (led_select == 0 ) {                                     // 起動時、および「OFF」ボタンが押された場合
    sprite5.pushImage(0, 0, led_button_img01Width,led_button_img01Height, led_button_img01);
    sprite5.pushImage(0, 50, led_button_img02Width,led_button_img02Height, led_button_img02);
    sprite5.pushImage(0, 100, led_button_img03Width,led_button_img03Height, led_button_img03); 
    sprite5.pushImage(0, 150, led_button_img042Width,led_button_img042Height, led_button_img042); 

  } else if (led_select > 1 && led_select <=4) {              // 「単色」ボタンが押された場合
    sprite5.pushImage(0, 0, led_button_img012Width,led_button_img012Height, led_button_img012);
    sprite5.pushImage(0, 50, led_button_img02Width,led_button_img02Height, led_button_img02);
    sprite5.pushImage(0, 100, led_button_img03Width,led_button_img03Height, led_button_img03);
    sprite5.pushImage(0, 150, led_button_img04Width,led_button_img04Height, led_button_img04);
    
  } else if (led_select == 5) {                               // 「虹」ボタンが押された場合
    sprite5.pushImage(0, 0, led_button_img01Width,led_button_img01Height, led_button_img01);
    sprite5.pushImage(0, 50, led_button_img022Width,led_button_img022Height, led_button_img022);
    sprite5.pushImage(0, 100, led_button_img03Width,led_button_img03Height, led_button_img03);
    sprite5.pushImage(0, 150, led_button_img04Width,led_button_img04Height, led_button_img04);
    
  } else if (led_select == 6) {                               // 「アニメ」ボタンが押された場合
    sprite5.pushImage(0, 0, led_button_img01Width,led_button_img01Height, led_button_img01);
    sprite5.pushImage(0, 50, led_button_img02Width,led_button_img02Height, led_button_img02);
    sprite5.pushImage(0, 100, led_button_img032Width,led_button_img032Height, led_button_img032);
    sprite5.pushImage(0, 150, led_button_img04Width,led_button_img04Height, led_button_img04);
    
  }
  sprite4.pushRotateZoomWithAA(10, 30, 0.0, 1.0, 1.0);          // スプライトをLcdへ描画
  sprite5.pushRotateZoomWithAA(200, 30, 0.0, 1.0, 1.0);         // スプライトをLcdへ描画
}

// ------------------------------------------------------------
// GAME 画面描画関数 
// ------------------------------------------------------------
void draw_sprite_game() {

  sprite4.pushImage(0, 0, game_go_img01Width,game_go_img01Height, game_go_img01);
  
  sprite1.fillScreen(c_BLACK);                                              // スプライトの塗りつぶし

  int v_r = 90;
  sprite1.fillScreen(c_BLACK);                                              // スプライトの塗りつぶし
  float width_harf = led_screen_img01Width / 2;                             // 幅／２の計算
  float height_harf = led_screen_img01Height / 2;                           // 高さ／２の計算
  for (int i = 30; i <= 360; i = i + 30) {                                  // ルーレットの描画用座標の計算
    float v_x1 = v_r * sin((i-15) / (180 / PI)) + width_harf;
    float v_x2 = v_r * sin((i+15)/ (180 / PI)) + width_harf;   
    float v_y1 = v_r * cos((i-15)/ (180 / PI)) + height_harf;
    float v_y2 = v_r * cos((i+15)/ (180 / PI)) + height_harf;
    if (i == 30) {
      sprite1.fillTriangle(v_x1, v_y1, v_x2, v_y2, 90, 90, c_RED);          // ルーレットの現在座標のみ赤く塗りつぶし
    } else {
      sprite1.drawTriangle(v_x1, v_y1, v_x2, v_y2, 90, 90, c_WHITE);        // ルーレットの他部分は白枠で描画
    }
  }
  v_r = 70;  
  for (int i = 30; i <= 360; i = i + 30) {                                  // ルーレット上に数字を描画
    float v_x = v_r * sin((i / (180 / PI))) + width_harf;                   // ルーレット数字座標の計算
    float v_y = v_r * cos((i / (180 / PI))) + height_harf;
    sprite1.setFont(&fonts::Font4);
    sprite1.setTextDatum(middle_center);                                    // 数字を中央寄せ指定
    sprite1.drawString(String(i/30), v_x, v_y);                             // ルーレット数字の描画
  }
  sprite1.drawCircle(width_harf, height_harf, 90, c_WHITE);                 // ルーレットの外周円を描画
  
  sprite1.pushRotateZoomWithAA(70, 50, 0.0, 1.0, 1.0);                      // sprite1をLcdへ描画
  sprite4.pushRotateZoomWithAA(0, 40, 0.0, 1.0, 1.0, c_BLACK);              // sprite4をLcdへ描画

  FastLED.clear();                                                          // ルーレットの初期位置のLEDを点灯
  leds[1] = CRGB::Red;
  FastLED.show();
  
}
// ------------------------------------------------------------
// GAME 画面　「Go」ボタン押下時の描画関数 
// ------------------------------------------------------------
void go_game() {

  sprite1.fillScreen(c_BLACK);                                              // スプライトの塗りつぶし

  int goal = random(1, 12);                                                 // ゲーム結果のランダム生成
  
  float width_harf = led_screen_img01Width / 2;                             // 幅／２の計算
  float height_harf = led_screen_img01Height / 2;                           // 高さ／２の計算

  int rotate_cnt = 1;                                                       // ルーレット回転数カウント用
  while (rotate_cnt < 5) {                                                  // ルーレット回転 （4回）             
    for (int cnt = 1; cnt <= 12; cnt++) {                                   // ルーレット描画
      int v_r = 90;                                                         // 描画半径の指定
      sprite1.fillScreen(c_BLACK);                                          // スプライトの塗りつぶし
      for (int i = 30; i <= 360; i = i + 30) {                              // ルーレットの描画用座標の計算
        float v_x1 = v_r * sin((i-15) / (180 / PI)) + width_harf;
        float v_x2 = v_r * sin((i+15)/ (180 / PI)) + width_harf;   
        float v_y1 = v_r * cos((i-15)/ (180 / PI)) + height_harf;
        float v_y2 = v_r * cos((i+15)/ (180 / PI)) + height_harf;
        if (cnt == i/30) {
          sprite1.fillTriangle(v_x1, v_y1, v_x2, v_y2, 90, 90, c_RED);      // ルーレットの現在座標のみ赤く塗りつぶし
        } else {
          sprite1.drawTriangle(v_x1, v_y1, v_x2, v_y2, 90, 90, c_WHITE);    // ルーレットの他部分は白枠で描画
        }
      }
      v_r = 70;  
      for (int i = 30; i <= 360; i = i + 30) {                              // ルーレット上に数字を描画
        float v_x = v_r * sin(i / (180 / PI)) + width_harf;                 // ルーレット数字座標の計算
        float v_y = v_r * cos(i / (180 / PI)) + height_harf;
        sprite1.setFont(&fonts::Font4);
        sprite1.setTextDatum(middle_center);                                // 数字を中央寄せ指定
        sprite1.drawString(String(i/30), v_x, v_y);                         // ルーレット数字の描画
      }
      sprite1.drawCircle(width_harf, height_harf, 90, c_WHITE);             // ルーレットの外周円を描画

      sprite1.pushRotateZoomWithAA(70, 50, 0.0, 1.0, 1.0);                  // スプライトをLcdへ描画
    
      for (int led_c = 0; led_c < 12; led_c++) {                            // ルーレットに合わせてファンのLEDを点灯
        if (cnt == led_c) {
          leds[led_c] = CRGB::Red;
        } else {
          leds[led_c] = CRGB::Black;
        }
        FastLED.show();
      }
    } 
    rotate_cnt++;
  }

  for (int cnt = 1; cnt <= goal; cnt++) {                                   // ゲーム結果まで回転させる
    int v_r = 90;                                                           // 描画半径の指定
    sprite1.fillScreen(c_BLACK);                                            // スプライトの塗りつぶし
    for (int i = 30; i <= 360; i = i + 30) {                                // ルーレットの描画用座標の計算
      float v_x1 = v_r * sin((i-15) / (180 / PI)) + width_harf;
      float v_x2 = v_r * sin((i+15)/ (180 / PI)) + width_harf;   
      float v_y1 = v_r * cos((i-15)/ (180 / PI)) + height_harf;
      float v_y2 = v_r * cos((i+15)/ (180 / PI)) + height_harf;
      if (cnt == i/30) {
        sprite1.fillTriangle(v_x1, v_y1, v_x2, v_y2, 90, 90, c_RED);        // ルーレットの現在座標のみ赤く塗りつぶし
      }
      sprite1.drawTriangle(v_x1, v_y1, v_x2, v_y2, 90, 90, c_WHITE);        // ルーレットの他部分は白枠で描画
    }
    v_r = 70;  
    for (int i = 30; i <= 360; i = i + 30) {                                // ルーレット上に数字を描画
      float v_x = v_r * sin(i / (180 / PI)) + width_harf;                   // ルーレット数字座標の計算
      float v_y = v_r * cos(i / (180 / PI)) + height_harf;
      sprite1.setFont(&fonts::Font4);
      sprite1.setTextDatum(middle_center);                                  // 数字を中央寄せ指定
      sprite1.drawString(String(i/30), v_x, v_y);                           // ルーレット数字の描画
    }
    sprite1.drawCircle(width_harf, height_harf, 90, c_WHITE);               // ルーレットの外周円を描画

    sprite1.pushRotateZoomWithAA(70, 50, 0.0, 1.0, 1.0);                    // スプライトをLcdへ描画

    for (int led_c = 0; led_c < 12; led_c++) {                              // ルーレットに合わせてファンのLEDを点灯
      if (cnt == led_c) {
        leds[led_c] = CRGB::Red;
      } else {
        leds[led_c] = CRGB::Black;
      }
      FastLED.show();
    }
  }   
}

// ------------------------------------------------------------
// Setup関数 
// ------------------------------------------------------------
void setup() {
  
  // M5StickCの初期化と動作設定　Initialization and operation settings of M5StickC.
  M5.begin(true, true, true, false, kMBusModeOutput);         // 開始
  lcd.init();

  // 初期画面生成
  lcd.setRotation(display_rotation);                          // 画面の向きを変更

  lcd.fillScreen(BLACK);                                      // 画面の塗りつぶし　Screen fill.
  lcd.setTextColor(WHITE, BLACK);                             // テキストカラーの設定

  // ヘッダーのタブ表示（初回）
  lcd.pushImage(0, 5, screen_header_img01Width,screen_header_img01Height, screen_header_img01);

  create_sprite_fan();                                        // 画面表示用スプライト作成
  draw_sprite_fan(rpm_str, v_angle);                          // FAN画面表示
         
  Serial.begin(115200);                                       // シリアルコンソール開始
  delay(500);                                                 // 処理待ち

  pinMode(FAN_PWM_PIN, OUTPUT);                               // PWM出力用GPIOの設定
  pinMode(FAN_SENSOR_PIN, INPUT);                             // ファン回転数計算のための信号用GPIOの設定

  // PWM出力設定・制御
  ledcSetup(CH0, PWM_FREQ, PWM_BIT);                          // PWMのチャネル設定　PWM Chanel set
  ledcAttachPin(FAN_PWM_PIN, CH0);                            // GPIOとチャネルの紐づけ　Linking GPIO and channel.
  Send_PWM(cur_duty_val);                                     // PWM信号の送信

  // FAN回転パルス取得関連の初期化と設定
  lastPulse_T = 0;                                            // 前回割込み時の時間を初期化
  pulse_Interval = 0;                                         // パルスのインターバルを初期化
  attachInterrupt(FAN_SENSOR_PIN, Fan_Rotate_Sence, FALLING); // FAN_SENSOR用GPIOが立ち下がった（FALLING）ときに関数を実行

  // ----------------------------------------------------------
  // Step5: LED点灯制御の設定
  //        LED lighting control settings
  // ----------------------------------------------------------
  FastLED.addLeds<SK6812, ARGB_LED_PIN, GRB>(leds, LED_COUNT);
  FastLED.setBrightness(254);
  set_max_power_in_volts_and_milliamps(5, 100);

}

void loop() {

  static bool touch = false;                                  // タッチパネル状態の初期化
  static uint16_t color = c_BLACK;                            // 描画色
  static uint16_t colorOld = c_BLACK;                         // ボタンフィードバック用
  TouchPoint_t pos = M5.Touch.getPressPoint();                // タッチ位置の保存用

  if (pos.x != -1) {
    if (!touch) {
      touch = true;
      M5.Axp.SetLed(false);
    }
    if (pos.y < 30) {
      M5.Axp.SetLDOEnable(3, true);                           // クリック感を出すために振動モーターをO
      delay(100);
      M5.Axp.SetLDOEnable(3, false);                          // クリック感を出すために振動モーターをOFF
      if (pos.x < 105) {
        lcd.fillScreen(c_BLACK);
        lcd.pushImage(0, 5, screen_header_img01Width,screen_header_img01Height, screen_header_img01);
        display_stat = 1;
        create_sprite_fan();
      } else if (pos.x < 210) {
        lcd.fillScreen(c_BLACK);
        lcd.pushImage(0, 5, screen_header_img02Width,screen_header_img02Height, screen_header_img02);
        display_stat = 2;
        create_sprite_led();
      } else if (pos.x < 320) {
        lcd.fillScreen(c_BLACK);
        lcd.pushImage(0, 5, screen_header_img03Width,screen_header_img03Height, screen_header_img03);
        display_stat = 3;
        create_sprite_game();
      }
    }
    if (display_stat == 1) {
      if (pos.y > 240) {                                      // タッチ位置のY座標が240以上はボタンエリア        
        if (pos.x < 109) {                                    // 左側のボタン位置が押された場合
          if (cur_duty_val > 0) {
            M5.Axp.SetLDOEnable(3, true);                     // クリック感を出すために振動モーターをON
            delay(200);
            M5.Axp.SetLDOEnable(3, false);                    // クリック感を出すために振動モーターをOFF
            cur_duty_val = cur_duty_val - 25;                 // PWMのデューティ比を減らす
          }
        } else if (pos.x > 218) {                             // 右側のボタン位置が押された場合
          if (cur_duty_val < 100) {
            M5.Axp.SetLDOEnable(3, true);                     // クリック感を出すために振動モーターをON
            delay(200);
            M5.Axp.SetLDOEnable(3, false);                    // クリック感を出すために振動モーターをOFF
            cur_duty_val = cur_duty_val + 25;                 // PWMのデューティ比を増やす
          }        
        } else if (pos.x >= 109 && pos.x <= 218) {            // 真ん中のボタン位置が押された場合
        }
      }

      Send_PWM(256 * cur_duty_val / 100);                     // PWM信号の出力 
       
    } else if (display_stat == 2 ) {
       int led_select = 0;
       if (pos.x > 210) {                                     // タッチ位置のX座標が200以上はボタンエリア
        M5.Axp.SetLDOEnable(3, true);                         // クリック感を出すために振動モーターをON
        delay(200);
        M5.Axp.SetLDOEnable(3, false);                        // クリック感を出すために振動モーターをOFF
        
        if (pos.y > 40 && pos.y < 80) {                       // 「単色」ボタン位置が押された場合
          led_select = random(1, 4);
          
        } else if (pos.y > 80 && pos.y < 120) {               // 「虹」ボタン位置が押された場合
          led_select = 5;
          
        } else if (pos.y > 120 && pos.y < 160) {              // 「アニメ」ボタン位置が押された場合
          led_select = 6;
        
        } else if (pos.y > 160 && pos.y < 200) {              // 「OFF」ボタン位置が押された場合
          led_select = 0;
          
        }
      }
      draw_sprite_led(led_select);
      set_led(led_select);
      
    } else if (display_stat == 3 ) {
      if (pos.x < 70 && pos.y < 80) {
        M5.Axp.SetLDOEnable(3, true);                         // クリック感を出すために振動モーターをON
        delay(200);
        M5.Axp.SetLDOEnable(3, false);                        // クリック感を出すために振動モーターをOFF
        go_game();
      } else {
        draw_sprite_game();
      }  
    }
  }

  if (display_stat == 1) {
    if (pulse_Interval != 0) {                                // インターバルが0以外の時
      rpm = 60000000 / (pulse_Interval * 2);                  // 1分あたりの回転数（RPM）を求める
      display_output_rpm(rpm);                                // 回転数の表示
    } else {                                                  // インターバルが0の時（0除算の回避）
      display_output_rpm(rpm);                                // 回転数の表示
    } 
    draw_sprite_fan(rpm_str, v_angle);                        // FAN画面表示  
  }
}
