/***************************************************
 * GRIDLESS - A LoRa protocol messaging system
 ***************************************************/

#include <SPI.h>
#include <LoRa.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "driver/rtc_io.h"

/**************************
* Globals
***************************/
// TFT object
TFT_eSPI tft = TFT_eSPI();


/**************************
* BLE Server Setup
***************************/
// Server Name
#define bleserverName "Gridless"
// Service UUID
#define SERVICE_UUID "9bfd14cf-53b9-44ff-9608-f65134511519"
// Sent Messsage Characteristic and Descriptor
BLECharacteristic sentMessageCharacteristics("782d8947-d51c-44b8-bace-bf1533a2f801", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor sentMessageDescriptor(BLEUUID((uint16_t)0x2902));
// Received Message Characteristic and Descriptor
BLECharacteristic recMessageCharacteristics("159fde59-e132-48ee-b943-488989514a02", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor recMessageDescriptor(BLEUUID((uint16_t)0x2902));

bool deviceConnected = false;
bool bleActive = false;

// Set callbacks on connect and disconnect
class MyServerCallbacks: public BLEServerCallbacks
{
  void onConnect(BLEServer* pServer)
  {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

// Activated BLE
void startBLE()
{
  if(bleActive) return;

  BLEDevice::init(bleserverName);

  // Create the server
  BLEDevice::setMTU(517); // Request bigger Transmission Unit
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *msgService = pServer->createService(SERVICE_UUID);

  // Create characteristics and descriptors
  // Sent Message
  msgService->addCharacteristic(&sentMessageCharacteristics);
  sentMessageDescriptor.setValue("Sent Message");
  sentMessageCharacteristics.addDescriptor(new BLE2902());
  // REceived Message
  msgService->addCharacteristic(&recMessageCharacteristics);
  recMessageDescriptor.setValue("Received Message");
  recMessageCharacteristics.addDescriptor(new BLE2902());

  // Start Service
  msgService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection");

  bleActive = true;
}

// Turn OFF BLE
void stopBLE()
{
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  if(adv) adv->stop();
  Serial.println("BLE turned off");
}

// SEND message to app
void sendBLEMessage(const char *msg)
{
  if(!bleActive || !deviceConnected) return;

  sentMessageCharacteristics.setValue(msg);
  sentMessageCharacteristics.notify();
}

void recBLEMessage(const char *msg)
{
  if(!bleActive || !deviceConnected) return;

  recMessageCharacteristics.setValue(msg);
  recMessageCharacteristics.notify();
}

/**************************
* ESP SLEEP Setup
***************************/
gpio_num_t WAKE_GPIO = GPIO_NUM_33; // Set external wake pin
#define TFT_BL  21   // Set the TFT LED pin
#define TIME_TO_SLEEP 30  // Set time to deep sleep

RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t lastTouchTime = 0;
RTC_DATA_ATTR uint32_t elapsed = 0;

void printWakeupReason()
{
  esp_sleep_wakeup_cause_t wake_reason = esp_sleep_get_wakeup_cause();

  if(wake_reason == ESP_SLEEP_WAKEUP_EXT0)
  {
    Serial.println("Wake caused by touch IRQ");
  }
  else
  {
    Serial.println("Wake up from normal boot/reset");
  }
}

/**************************
* LoRa Setup
***************************/
//  LoRa SX1276 Pin Mapping 
#define LORA_SS   5
#define LORA_RST  22
#define LORA_DIO0 26

// LoRa Frequency
#define LORA_FREQ 915E6   // 433E6 / 868E6 / 915E6

// Addresses
byte localAddr = 0xFF;
byte destination = 0xBB;

// Message Buffers
//const char outgoing;
String incomingMsg = "";

/**************************
* LVGL Setup
***************************/
// Screen Dimensions
#define SCREEN_W 320
#define SCREEN_H 480

// Draw Buffer
static lv_color_t draw_buf_mem[SCREEN_W * 10];
static lv_draw_buf_t draw_buf;

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char *buf)
{
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}
#endif

// UI Elements
static lv_obj_t *addr_bar;
static lv_obj_t *addr_label;
static lv_obj_t *msg_btn;
static lv_obj_t *msgbtn_label;
static lv_obj_t *msg_ta;
static lv_obj_t *addr_ta;
static lv_obj_t *kb;
static lv_obj_t *chat_area;
static lv_obj_t *ble_btn;
static lv_obj_t *ble_label;

// Display Flush
void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)px_map, w * h, true);
  tft.endWrite();

  lv_display_flush_ready(disp);
}

// Read touchpad
void touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
  uint16_t x, y;
  bool touched = tft.getTouch(&x, &y, 600);

  if(!touched) {
    data->state = LV_INDEV_STATE_RELEASED;
  } else {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
    
    Serial.print("x = ");
    Serial.println(x);
    Serial.print("y = ");
    Serial.println(y);

    lastTouchTime = millis();
  }
}

// Add Message to chat
void addMessage(bool isMine, const char *msg)
{
  //Bubble container
  lv_obj_t *bubble = lv_obj_create(chat_area);
  lv_obj_set_size(bubble, lv_pct(60), LV_SIZE_CONTENT);
  lv_obj_set_style_radius(bubble, 10, 0);
  lv_obj_set_style_pad_all(bubble, 5, 0);
  lv_obj_set_style_border_width(bubble, 0, 0);

  if(isMine)
  {
    lv_obj_set_style_bg_color(bubble, lv_color_hex(0xDCF8C6), 0);
  }else{
    lv_obj_set_style_bg_color(bubble, lv_color_hex(0xADD8E6), 0);
  }

  lv_obj_t *label = lv_label_create(bubble);
  lv_label_set_text(label, msg);
  lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(label, lv_pct(100));

  lv_obj_scroll_to_view(bubble, LV_ANIM_ON);
}

// Open Keyboard
static void openKeyboard(lv_obj_t *target_ta)
{
  lv_keyboard_set_textarea(kb, target_ta);

  lv_obj_set_size(chat_area, lv_pct(100), 230);
  lv_obj_remove_flag(kb, LV_OBJ_FLAG_HIDDEN);
  lv_obj_remove_flag(target_ta, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(msg_btn, LV_OBJ_FLAG_HIDDEN);
}

// Receive LoRa Packets
void onReceive(int packetSize)
{
  if(!packetSize) return;

  int recipient = LoRa.read();
  int sender = LoRa.read();
  incomingMsg = LoRa.readString();

  if(recipient != localAddr && recipient != 0xBB)
  {
    Serial.println("Message not for me.");
    return;
  }

  addMessage(false, incomingMsg.c_str());
  recBLEMessage(incomingMsg.c_str());

  Serial.print("Received: ");
  Serial.println(incomingMsg);
  Serial.print("RSSI: ");
  Serial.println(LoRa.packetRssi());
}

// Add message button call back
static void msgbtn_event_cb(lv_event_t *e)
{
  if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
    openKeyboard(msg_ta);
    Serial.println("Input Clicked");
  }
}

// Keyboard Call Back Event
static void kb_event_cb(lv_event_t *e)
{
  lv_obj_t *active_ta = lv_keyboard_get_textarea(kb); 

  if(lv_event_get_code(e) == LV_EVENT_READY) 
  {
    const char *text = lv_textarea_get_text(active_ta);

    // Message Textarea -> Send LoRa Message
    if(active_ta == msg_ta)
    {
      if(strlen(text) > 0)
      {
        LoRa.beginPacket();
        LoRa.write(localAddr);
        LoRa.write(destination);
        LoRa.print(text);
        LoRa.endPacket();

        Serial.print("Message sent: ");
        Serial.println(text);

        addMessage(true, text);
        sendBLEMessage(text);
        lv_textarea_set_text(msg_ta, "");
      }
    lv_obj_remove_flag(msg_btn, LV_OBJ_FLAG_HIDDEN);
    }

    // Address Textarea -> Update Destination Address
    else if(active_ta == addr_ta)
    {
      lv_label_set_text(addr_label, text);
      // Hide keyboard after address set
      Serial.print("New address: ");
      Serial.println(text);
    }
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(active_ta, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(msg_ta, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(addr_ta, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(msg_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(chat_area, lv_pct(100), lv_pct(100));
  }

  if(lv_event_get_code(e) == LV_EVENT_CANCEL)
  {
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(active_ta, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(msg_ta, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(addr_ta, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(msg_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(chat_area, lv_pct(100), lv_pct(100));
  }
}

// Address Bar Call Back
void addrBar_event_cb(lv_event_t *e)
{
  if(lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    openKeyboard(addr_ta);
    Serial.println("Address clicked");
  }
}

// BLE Start/Stop button callback
void blebtn_event_cb(lv_event_t *e)
{
  if(lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    if(!bleActive)
    {
      startBLE();
      lv_obj_set_style_bg_color(ble_btn, lv_color_hex(0x24A0ED), 0);
    }
    else
    {
      stopBLE();
      bleActive = false;
      lv_obj_set_style_bg_color(ble_btn, lv_color_hex(0xD3D3D3), 0);
    }
  }
}

// Create UI
void create_ui()
{
  lv_obj_t *scr = lv_screen_active();

  // Chat Area
  chat_area = lv_obj_create(scr);
  lv_obj_set_size(chat_area, lv_pct(100), lv_pct(100));
  lv_obj_set_flex_flow(chat_area, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(chat_area, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END);
  lv_obj_set_scroll_dir(chat_area, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(chat_area, LV_SCROLLBAR_MODE_AUTO);

  // Address Bar
  addr_bar = lv_obj_create(scr);
  lv_obj_set_size(addr_bar, lv_pct(100), 50);
  lv_obj_set_pos(addr_bar, 0, 0);
  lv_obj_remove_flag(addr_bar, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(addr_bar, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(addr_bar, addrBar_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_move_foreground(addr_bar);

  addr_label = lv_label_create(addr_bar);
  lv_label_set_text(addr_label, "Contact");
  lv_obj_center(addr_label);

  ble_btn = lv_button_create(addr_bar);
  lv_obj_set_size(ble_btn, 30, 30);
  lv_obj_align(ble_btn, LV_ALIGN_RIGHT_MID, NULL, NULL);
  lv_obj_add_event_cb(ble_btn, blebtn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_style_radius(ble_btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(ble_btn, lv_color_hex(0xD3D3D3), 0);

  ble_label = lv_label_create(ble_btn);
  lv_label_set_text(ble_label, LV_SYMBOL_BLUETOOTH);
  lv_obj_center(ble_label);

  // Input Bar
  msg_btn = lv_button_create(scr);
  lv_obj_set_size(msg_btn, 50, 50);
  lv_obj_add_flag(msg_btn, LV_OBJ_FLAG_FLOATING);
  lv_obj_align(msg_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
  lv_obj_add_event_cb(msg_btn, msgbtn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_style_radius(msg_btn, LV_RADIUS_CIRCLE, 0);
  
  msgbtn_label = lv_label_create(msg_btn);
  lv_label_set_text(msgbtn_label, LV_SYMBOL_PLUS);
  lv_obj_center(msgbtn_label);

  // Keyboard
  kb = lv_keyboard_create(scr);
  lv_obj_set_size(kb, lv_pct(100), 200);
  lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_ALL, NULL);

  msg_ta = lv_textarea_create(scr);
  lv_textarea_set_placeholder_text(msg_ta, "Enter a message...");
  lv_obj_set_size(msg_ta, lv_pct(100), 50);
  lv_obj_align(msg_ta, LV_ALIGN_BOTTOM_MID, 0, -200);
  lv_obj_add_flag(msg_ta, LV_OBJ_FLAG_HIDDEN);

  addr_ta = lv_textarea_create(scr);
  lv_textarea_set_placeholder_text(addr_ta, "Address");
  lv_obj_set_size(addr_ta, lv_pct(100), 50);
  lv_obj_align(addr_ta, LV_ALIGN_BOTTOM_MID, 0, -200);
  lv_obj_add_flag(addr_ta, LV_OBJ_FLAG_HIDDEN);
}

/**************************
* MAIN Setup
***************************/
void setup()
{
  Serial.begin(115200);
  delay(1000);
  bootCount++;
  Serial.print("Boot number: ");
  Serial.println(bootCount);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // INIT TFT
  tft.init();
  tft.writecommand(0x11);
  delay(120);
  tft.writecommand(0x29);
  tft.setRotation(2); // Portrait w/header @ top
  tft.fillScreen(TFT_BLACK);
  uint16_t calData[5] = {295, 3493, 320, 3602, 2};
  tft.setTouch(calData);
  printWakeupReason();

  if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0)
  {
    lastTouchTime = millis();
  }
  else
  {
    lastTouchTime  = millis();
  }

  // INIT LVGL
  lv_init();

  // Set Tick
  //lv_tick_set_cb(my_tick);
  if(lastTouchTime == 0)
  {
    lastTouchTime = millis();
  }

  // INIT Draw Buffer
  lv_draw_buf_init(
    &draw_buf,
    SCREEN_W,
    10,
    LV_COLOR_FORMAT_RGB565,
    SCREEN_W * sizeof(lv_color_t),
    draw_buf_mem,
    sizeof(draw_buf_mem)
  );

  // INIT LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if(!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa Failed");
  }
  Serial.println("LoRa Initialized");
  

  // Create display
  lv_display_t *disp = lv_display_create(SCREEN_W, SCREEN_H);
  lv_display_set_draw_buffers(disp, &draw_buf, NULL);
  lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(disp, disp_flush);

  // Create Input Device
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touch_read);

  // Build UI
  create_ui();

  // Release Classic BT memory assets (DO NOT UST CLASSIC BT)
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
}

/**************************
* MAIN Loop
***************************/
void loop()
{
  // LVGL SET CLK
  static uint32_t lastTick = 0;
  uint32_t now = millis();

  if(lastTick == 0)
  {
    lastTick = now;
  }

  lv_tick_inc(now - lastTick);
  lastTick = now;
  lv_timer_handler();

  // Set Sleep Pararmeters
  elapsed = millis() - lastTouchTime;
  // DEEP SLEEP
  if(elapsed >= TIME_TO_SLEEP * 1000UL)
  {
    delay(50);
    if(millis() - lastTouchTime >= TIME_TO_SLEEP * 1000UL)
    {
      Serial.println("Entering Deep Sleep");
      Serial.flush();

      tft.writecommand(0x28);
      delay(10);
      tft.writecommand(0x10);
      delay(20);
      digitalWrite(TFT_BL, LOW);
      tft.endWrite();
      pinMode(TFT_MOSI, INPUT);
      pinMode(TFT_SCLK, INPUT);
      pinMode(TFT_CS, INPUT);
      pinMode(TFT_DC, INPUT);
      pinMode(TFT_RST, INPUT);

      rtc_gpio_init(WAKE_GPIO);
      rtc_gpio_set_direction(WAKE_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
      rtc_gpio_pullup_en(WAKE_GPIO);
      rtc_gpio_pulldown_dis(WAKE_GPIO);

      if(bleActive)
      {
        stopBLE();
        bleActive = false;
      }

      //esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_ext0_wakeup(WAKE_GPIO, 0);
      esp_deep_sleep_start();
    }
  }
 
  // Check for incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if(packetSize)
  {
    lastTouchTime = millis();
    onReceive(packetSize);
  }

  // Add slight delay for processing
  delay(5);
}
