# ewinch_remote_controller
 transmitter and receiver code for remote controlling a paragliding winch
 Based on LILYGO® TTGO ESP32-Paxcounter LoRa32 V2.1 1.6 Version 915MHZ LoRa ESP-32 OLED
 (http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1271&FId=t3:50060:3)
 
 see https://www.youtube.com/watch?v=5IkagHkxbxY

 receiver uses PPM (Pulse Position Modulation) for driving the winch and (optional) UART to read additional information (line length, battery %, dutycycle)
 VESC UART communication depends on https://github.com/SolidGeek/VescUart/

Pin Setup receiver:
PWM_PIN_OUT  13
VESC_RX  14    //connect to TX on Vesc
VESC_TX  2    //connect to RX on Vesc

transmitter buttons connected to GND:
BUTTON_UP  15 
BUTTON_DOWN  12

Line auto stop can be implemented within VESC with vesc_ppm_auto_stop.patch
Default VESC app config is vesc_app_config.xml
