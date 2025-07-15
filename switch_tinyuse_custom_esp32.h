/************************************************************************
MIT License

Copyright (c) 2021 touchgadgetdev@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*************************************************************************/
#pragma once
#include "USB.h"
#include "USBHID.h"

// Dpad directions
typedef uint8_t NSDirection_t;
#define NSGAMEPAD_DPAD_UP  0
#define NSGAMEPAD_DPAD_UP_RIGHT 1
#define NSGAMEPAD_DPAD_RIGHT 2
#define NSGAMEPAD_DPAD_DOWN_RIGHT 3
#define NSGAMEPAD_DPAD_DOWN 4
#define NSGAMEPAD_DPAD_DOWN_LEFT 5
#define NSGAMEPAD_DPAD_LEFT 6
#define NSGAMEPAD_DPAD_UP_LEFT 7
#define NSGAMEPAD_DPAD_CENTERED 0xF

enum NSButtons {
  NSButton_Y = 0,
  NSButton_B,
  NSButton_A,
  NSButton_X,
  NSButton_LeftTrigger,
  NSButton_RightTrigger,
  NSButton_LeftThrottle,
  NSButton_RightThrottle,
  NSButton_Minus,
  NSButton_Plus,
  NSButton_LeftStick,
  NSButton_RightStick,
  NSButton_Home,
  NSButton_Capture,
  NSButton_Reserved1,
  NSButton_Reserved2
};

#define ATTRIBUTE_PACKED  __attribute__((packed, aligned(1)))

// 14 Buttons, 4 Axes, 1 D-Pad
typedef struct ATTRIBUTE_PACKED {
  uint16_t buttons;

  uint8_t dPad;

  uint8_t leftXAxis;
  uint8_t leftYAxis;

  uint8_t rightXAxis;
  uint8_t rightYAxis;
  uint8_t filler;

} HID_NSGamepadReport_Data_t;


// HID report descriptor using ESP32-S3 format
static const uint8_t desc_hid_report[] = {
  // Gamepad for Nintendo Switch
  // 14 buttons, 1 8-way dpad, 2 analog sticks (4 axes)
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x05,        // Usage (Game Pad)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x35, 0x00,        //   Physical Minimum (0)
  0x45, 0x01,        //   Physical Maximum (1)
  0x75, 0x01,        //   Report Size (1)
  0x95, 0x0E,        //   Report Count (14)
  0x05, 0x09,        //   Usage Page (Button)
  0x19, 0x01,        //   Usage Minimum (0x01)
  0x29, 0x0E,        //   Usage Maximum (0x0E)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x95, 0x02,        //   Report Count (2)
  0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
  0x25, 0x07,        //   Logical Maximum (7)
  0x46, 0x3B, 0x01,  //   Physical Maximum (315)
  0x75, 0x04,        //   Report Size (4)
  0x95, 0x01,        //   Report Count (1)
  0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
  0x09, 0x39,        //   Usage (Hat switch)
  0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
  0x65, 0x00,        //   Unit (None)
  0x95, 0x01,        //   Report Count (1)
  0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x26, 0xFF, 0x00,  //   Logical Maximum (255)
  0x46, 0xFF, 0x00,  //   Physical Maximum (255)
  0x09, 0x30,        //   Usage (X)
  0x09, 0x31,        //   Usage (Y)
  0x09, 0x32,        //   Usage (Z)
  0x09, 0x35,        //   Usage (Rz)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x04,        //   Report Count (4)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x01,        //   Report Count (1)
  0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,              // End Collection
};

class NSGamepad : public USBHIDDevice {
  public:
    NSGamepad(void);
    
    void begin(void);
    void end(void);
    void reset(void);
    void loop(void);

    void set(const HID_NSGamepadReport_Data_t &data);

    bool write(void);
    bool write(void *report);
    void press(uint8_t b);
    void release(uint8_t b);
    void releaseAll(void);

    void buttons(uint16_t b);
    void leftXAxis(uint8_t a);
    void leftYAxis(uint8_t a);
    void rightXAxis(uint8_t a);
    void rightYAxis(uint8_t a);

    void leftXAxisAdd(int val);
    void leftYAxisAdd(int val);
    void rightXAxisAdd(int val);
    void rightYAxisAdd(int val);

    void allAxes(uint32_t RYRXLYLX);
    void allAxes(uint8_t RY, uint8_t RX, uint8_t LY, uint8_t LX);
    void dPad(NSDirection_t d);
    void dPad(bool up, bool down, bool left, bool right);
    bool ready(void);
    bool isConnected(void);

    bool SendReport(void* data, size_t length);
    bool SendReport();

    bool compareTo(const HID_NSGamepadReport_Data_t &data);
    const HID_NSGamepadReport_Data_t getReportData();

    // USBHIDDevice implementation
    uint16_t _onGetDescriptor(uint8_t* buffer);
    void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len);

  protected:
    HID_NSGamepadReport_Data_t _report;
    uint32_t startMillis;
    bool _begun;
    USBHID _hid;
};

NSGamepad::NSGamepad(void) : _hid(), _begun(false)
{
  static bool initialized = false;
  if(!initialized){
    initialized = true;
    _hid.addDevice(this, sizeof(desc_hid_report));
  }
}

void NSGamepad::begin(void)
{
  if(_begun) return;
  _begun = true;
  
  // Set USB VID/PID
  USB.VID(0x0f0d);
  USB.PID(0x00c1);
  USB.productName("Nintendo Switch Controller");
  USB.manufacturerName("Nintendo");
  
  _hid.begin();
  USB.begin();
  
  // wait a bit for USB to initialize
  delay(100);
  
  // release all buttons, center all sticks, etc.
  reset();
  startMillis = millis();
}

void NSGamepad::end(void)
{
  if(!_begun) return;
  _begun = false;
  // ESP32-S3ではUSB.end()は利用できないため、
  // HIDデバイスの終了処理のみ行う
  // 実際にUSBを停止する必要がある場合は、
  // ESP.restart()でデバイスを再起動する必要があります
}

void NSGamepad::loop(void)
{
  if (startMillis != millis()) {
    write();
    startMillis = millis();
  }
}

void NSGamepad::reset(void)
{
  memset(&_report, 0x00, sizeof(_report));
  _report.leftXAxis = _report.leftYAxis = 0x80;
  _report.rightXAxis = _report.rightYAxis = 0x80;
  _report.dPad = NSGAMEPAD_DPAD_CENTERED;
}

bool NSGamepad::write(void)
{
  return SendReport(&_report, sizeof(_report));
}

bool NSGamepad::write(void *report)
{
  memcpy(&_report, report, sizeof(_report));
  return SendReport(&_report, sizeof(_report));
}

void NSGamepad::press(uint8_t b)
{
  b &= 0xF; // Limit value between 0..15
  _report.buttons |= (uint16_t)1 << b;
}

void NSGamepad::release(uint8_t b)
{
  b &= 0xF; // Limit value between 0..15
  _report.buttons &= ~((uint16_t)1 << b);
}

void NSGamepad::releaseAll(void)
{
  _report.buttons = 0;
}

void NSGamepad::buttons(uint16_t b)
{
  _report.buttons = b;
}

void NSGamepad::leftXAxis(uint8_t a)
{
  _report.leftXAxis = a;
}

void NSGamepad::leftYAxis(uint8_t a)
{
  _report.leftYAxis = a;
}

void NSGamepad::rightXAxis(uint8_t a)
{
  _report.rightXAxis = a;
}

void NSGamepad::rightYAxis(uint8_t a)
{
  _report.rightYAxis = a;
}

void NSGamepad::allAxes(uint32_t RYRXLYLX)
{
  _report.rightYAxis = ((RYRXLYLX >> 24) & 0xFF) ^ 0x80;
  _report.rightXAxis = ((RYRXLYLX >> 16) & 0xFF) ^ 0x80;
  _report.leftYAxis  = ((RYRXLYLX >>  8) & 0xFF) ^ 0x80;
  _report.leftXAxis  = ((RYRXLYLX      ) & 0xFF) ^ 0x80;
}

void NSGamepad::allAxes(uint8_t RY, uint8_t RX, uint8_t LY, uint8_t LX)
{
  _report.rightYAxis = RY ^ 0x80;
  _report.rightXAxis = RX ^ 0x80;
  _report.leftYAxis  = LY ^ 0x80;
  _report.leftXAxis  = LX ^ 0x80;
}

void NSGamepad::dPad(NSDirection_t d)
{
  _report.dPad = d;
}

void NSGamepad::dPad(bool up, bool down, bool left, bool right)
{
  static const NSDirection_t BITS2DIR[16] = {
    NSGAMEPAD_DPAD_CENTERED,    // 0000
    NSGAMEPAD_DPAD_RIGHT,       // 0001
    NSGAMEPAD_DPAD_LEFT,        // 0010
    NSGAMEPAD_DPAD_CENTERED,    // 0011
    NSGAMEPAD_DPAD_DOWN,        // 0100
    NSGAMEPAD_DPAD_DOWN_RIGHT,  // 0101
    NSGAMEPAD_DPAD_DOWN_LEFT,   // 0110
    NSGAMEPAD_DPAD_DOWN,        // 0111
    NSGAMEPAD_DPAD_UP,          // 1000
    NSGAMEPAD_DPAD_UP_RIGHT,    // 1001
    NSGAMEPAD_DPAD_UP_LEFT,     // 1010
    NSGAMEPAD_DPAD_UP,          // 1011
    NSGAMEPAD_DPAD_CENTERED,    // 1100
    NSGAMEPAD_DPAD_RIGHT,       // 1101
    NSGAMEPAD_DPAD_LEFT,        // 1110
    NSGAMEPAD_DPAD_CENTERED     // 1111
  };
  uint8_t dpad_bits = (up << 3) | (down << 2) | (left << 1) | (right << 0);
  _report.dPad = BITS2DIR[dpad_bits];
}

bool NSGamepad::ready(void)
{
  return _hid.ready();
}

bool NSGamepad::isConnected(void)
{
  // ESP32-S3では、HIDデバイスの準備状態のみチェック
  // USB自体の接続状態は、USBSerial.available()やUSBSerial.連結状態で確認可能
  return _begun && _hid.ready();
}

bool NSGamepad::SendReport(void* data, size_t length)
{
  return _hid.SendReport(0, data, length);
}

bool NSGamepad::SendReport()
{
  if(ready()){
    return write();
  }
  return false;
}

const HID_NSGamepadReport_Data_t NSGamepad::getReportData()
{
  return _report;
}

bool NSGamepad::compareTo(const HID_NSGamepadReport_Data_t &data)
{
  return _report.buttons == data.buttons &&
          _report.dPad == data.dPad &&
          _report.leftXAxis == data.leftXAxis &&
          _report.leftYAxis == data.leftYAxis &&
          _report.rightXAxis == data.rightXAxis &&
          _report.rightYAxis == data.rightYAxis &&
          _report.filler == data.filler;          
}

void NSGamepad::leftXAxisAdd(int val)
{
  int newVal = _report.leftXAxis + val;
  if(newVal < 0){
    _report.leftXAxis = 0;
  }
  else if(newVal > 255){
    _report.leftXAxis = 255;
  }
  else {
    _report.leftXAxis = newVal;
  }
}

void NSGamepad::leftYAxisAdd(int val)
{
  int newVal = _report.leftYAxis + val;
  if(newVal < 0){
    _report.leftYAxis = 0;
  }
  else if(newVal > 255){
    _report.leftYAxis = 255;
  }
  else {
    _report.leftYAxis = newVal;
  }
}

void NSGamepad::rightXAxisAdd(int val)
{
  int newVal = _report.rightXAxis + val;
  if(newVal < 0){
    _report.rightXAxis = 0;
  }
  else if(newVal > 255){
    _report.rightXAxis = 255;
  }
  else {
    _report.rightXAxis = newVal;
  }
}

void NSGamepad::rightYAxisAdd(int val)
{
  int newVal = _report.rightYAxis + val;
  if(newVal < 0){
    _report.rightYAxis = 0;
  }
  else if(newVal > 255){
    _report.rightYAxis = 255;
  }
  else {
    _report.rightYAxis = newVal;
  }
}

void NSGamepad::set(const HID_NSGamepadReport_Data_t &data)
{
  memcpy(&_report, &data, sizeof(data));
}

// USBHIDDevice implementation
uint16_t NSGamepad::_onGetDescriptor(uint8_t* buffer)
{
  memcpy(buffer, desc_hid_report, sizeof(desc_hid_report));
  return sizeof(desc_hid_report);
}

void NSGamepad::_onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len)
{
  // Handle output reports if needed
  // Nintendo Switch controllers typically don't use output reports for basic functionality
}

// 使用例:
/*
#include "switch_esp32s3_custom.h"

NSGamepad gamepad;

void setup() {
  Serial.begin(115200);
  
  // USB HIDを初期化
  gamepad.begin();
  
  // 初期化後、少し待機
  delay(1000);
  Serial.println("Nintendo Switch Gamepad Ready!");
}

void loop() {
  // HIDデバイスが準備できているかチェック
  if(!gamepad.ready()) {
    delay(100);
    return;
  }
  
  // ボタンのテスト
  gamepad.press(NSButton_A);
  gamepad.SendReport();
  delay(100);
  
  gamepad.release(NSButton_A);
  gamepad.SendReport();
  delay(100);
  
  // アナログスティックのテスト
  gamepad.leftXAxis(200);
  gamepad.leftYAxis(50);
  gamepad.SendReport();
  delay(100);
  
  // リセット
  gamepad.reset();
  gamepad.SendReport();
  delay(1000);
}
*/