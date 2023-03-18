//=====================================================================
// https://github.com/lovyan03/LovyanGFX/tree/master/src/lgfx/v1/platforms/esp32s3
//
//    lgfx_8048S043C.hを改変  2023.02.11
//      7.0" 黄色基板
//      タッチ無し
//
//=====================================================================
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

class LGFX : public lgfx::LGFX_Device{
  lgfx::Bus_RGB     _bus_instance;
  lgfx::Panel_RGB   _panel_instance;
  lgfx::Light_PWM   _light_instance;
  lgfx::Touch_GT911 _touch_instance;

public:LGFX(void){
  auto cfg    = _bus_instance.config();
  cfg.panel   = &_panel_instance;
  cfg.pin_d0  = GPIO_NUM_15; // B0
  cfg.pin_d1  = GPIO_NUM_7;  // B1
  cfg.pin_d2  = GPIO_NUM_6;  // B2
  cfg.pin_d3  = GPIO_NUM_5;  // B3
  cfg.pin_d4  = GPIO_NUM_4;  // B4
  cfg.pin_d5  = GPIO_NUM_9;  // G0
  cfg.pin_d6  = GPIO_NUM_46; // G1
  cfg.pin_d7  = GPIO_NUM_3;  // G2
  cfg.pin_d8  = GPIO_NUM_8;  // G3
  cfg.pin_d9  = GPIO_NUM_16; // G4
  cfg.pin_d10 = GPIO_NUM_1;  // G5
  cfg.pin_d11 = GPIO_NUM_14; // R0
  cfg.pin_d12 = GPIO_NUM_21; // R1
  cfg.pin_d13 = GPIO_NUM_47; // R2
  cfg.pin_d14 = GPIO_NUM_48; // R3
  cfg.pin_d15 = GPIO_NUM_45; // R4
  cfg.pin_henable       = GPIO_NUM_41;
  cfg.pin_vsync         = GPIO_NUM_40;
  cfg.pin_hsync         = GPIO_NUM_39;
  cfg.pin_pclk          = GPIO_NUM_42;
  cfg.freq_write        = 16000000;
  cfg.hsync_polarity    = 0;
  cfg.hsync_front_porch = 8;//8;//210;//8;
  cfg.hsync_pulse_width = 2;//4;//30;//2;
  cfg.hsync_back_porch  = 43;//8;//16;//43;
  cfg.vsync_polarity    = 0;
  cfg.vsync_front_porch = 8;//8;//4;//22;//8;
  cfg.vsync_pulse_width = 2;//4;//13;//2;
  cfg.vsync_back_porch  = 12;//8;//4;//10;//12;
  cfg.pclk_idle_high    = 1;
  _bus_instance.config(cfg);
  _panel_instance.setBus(&_bus_instance);
  
  { auto cfg = _panel_instance.config();
  cfg.memory_width  = 800;
  cfg.memory_height = 480;
  cfg.panel_width   = 800;
  cfg.panel_height  = 480;
  cfg.offset_x      = 0;
  cfg.offset_y      = 0;
  _panel_instance.config(cfg);
  }
  
  { auto cfg = _panel_instance.config_detail();
  cfg.use_psram = 1;
  _panel_instance.config_detail(cfg);
  }

  { auto cfg = _light_instance.config();
  cfg.pin_bl = GPIO_NUM_2;
  _light_instance.config(cfg);
  }
  _panel_instance.light(&_light_instance);
  
  setPanel(&_panel_instance); // 使用するパネルをセットします。
  }
};
LGFX tft; // 準備したクラスのインスタンスを作成します。
//=====================================================================
