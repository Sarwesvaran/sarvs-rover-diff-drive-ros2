#include "RMCS2303drive.h"
#include "Arduino.h"

// Removed SoftwareSerial includes and variables
const byte MaxByteArraySize = 8;
byte byteArray[MaxByteArraySize] = {0};

// Internal pointer variable moved to class member in header logic,
// but for this specific structure, we will use the class member variable 'port'.
// However, the original code used a global 'port'. To keep it clean:
// We will rely on the private member 'port' defined in the class (implied context)
// OR we can keep the global variable approach if you prefer minimal changes.
// Below is the CLEAN approach using the class member.

HardwareSerial *GlobalPort; // Helper to keep original logic working easily

void RMCS2303::Serial0(long baudrate)
{
  Serial.begin(baudrate);
}

void RMCS2303::begin(HardwareSerial *hport, long baudrate)
{
  port = hport;       // Assign to class member
  GlobalPort = hport; // Assign to global helper

  // ESP32 specific check: Serial is usually the USB debug port
  if (port == &Serial)
  {
    // Optional: You might actually want to use Serial for the motor if connected there.
    // But usually, it is for debugging.
    Serial.println("Warning: Using Main Serial. Ensure this is intended.");
  }

  port->begin(baudrate);
}

byte RMCS2303::Enable_Analog_Mode(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 2, 1);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::Disable_Analog_Mode(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 2, 0);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::Enable_Digital_Mode(byte slave_id, byte dir)
{
  int data;
  byte j;
  if (dir == 0)
    data = 257;
  if (dir == 1)
    data = 265;
  j = WriteSingleRegister(slave_id, 2, data);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::Disable_Digital_Mode(byte slave_id, byte dir)
{
  int data;
  byte j;
  if (dir == 0)
    data = 256;
  if (dir == 1)
    data = 264;
  j = WriteSingleRegister(slave_id, 2, data);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::Absolute_position(byte slave_id, long int count)
{
  byte j, k = 0;
  j = Absolute_move(slave_id, count);
  if (j == 1)
    k = EncoderPosition_enable(slave_id);

  return (k == 1) ? 1 : 0;
}

byte RMCS2303::Speed(byte slave_id, int SPEED)
{
  byte j;
  j = WriteSingleRegister(slave_id, 14, SPEED);
  return (j == 1) ? 1 : 0;
}

long int RMCS2303::Speed_Feedback(byte slave_id)
{
  String result1;
  unsigned long int q;
  long int p;
  result1 = ReadSingleRegister(slave_id, 24, 1);
  q = value(result1);
  if (q > 32765)
    p = q - 65535;
  else
    p = q;
  return (p);
}

byte RMCS2303::Brake_Motor(byte slave_id, byte dir)
{
  int data;
  byte j;
  if (dir == 0)
    data = 260;
  if (dir == 1)
    data = 268;
  j = WriteSingleRegister(slave_id, 2, data);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::EncoderPosition_LPR(byte slave_id, int LPR)
{
  byte j;
  j = WriteSingleRegister(slave_id, 10, LPR);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::Disable_Position_Mode(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 2, 512);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::EncoderPosition_enable(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 2, 513);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::Absolute_move(byte slave_id, long int count)
{
  unsigned long int data;
  byte j, k = 0;
  if (count > 0 && count < 2147483647)
    data = count;
  if (count < 0 && count >= -2147483648)
    data = 4294967295 - (count * -1);
  unsigned int Hidata = data >> 16;
  unsigned int Lodata = data & 0xFFFF;
  j = WriteSingleRegister(slave_id, 16, Lodata);
  if (j == 1)
  {
    k = WriteSingleRegister(slave_id, 18, Hidata);
  }

  return (k == 1) ? 1 : 0;
}

byte RMCS2303::Acceleration(byte slave_id, int acceleration)
{
  byte j;
  j = WriteSingleRegister(slave_id, 12, acceleration);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::ProportionalGain(byte slave_id, int PP_GAIN)
{
  byte j;
  j = WriteSingleRegister(slave_id, 4, PP_GAIN);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::IntegralGain(byte slave_id, int PI_GAIN)
{
  byte j;
  j = WriteSingleRegister(slave_id, 6, PI_GAIN);
  return (j == 1) ? 1 : 0;
}

byte RMCS2303::FeedforwardGain(byte slave_id, int VF_GAIN)
{
  byte j;
  j = WriteSingleRegister(slave_id, 8, VF_GAIN);
  return (j == 1) ? 1 : 0;
}

long int RMCS2303::Position_Feedback(byte slave_id)
{
  String result1, x;
  char a[9];
  char *endptr;
  unsigned long int q;
  long int p;
  result1 = ReadSingleRegister(slave_id, 20, 2);
  x = result1.substring(7, 15);
  x.toCharArray(a, 9);
  for (int i = 0; i < 4; i++)
  {
    char c = a[i];
    a[i] = a[i + 4];
    a[i + 4] = c;
  }
  q = strtoul(a, &endptr, 16);
  if (q > 2147483647)
    p = q - 4294967295;
  else
    p = q;
  return (p);
}

byte RMCS2303::WriteSingleRegister(byte slave_id, int address, unsigned int value)
{
  String response, res;
  byte j;
  String result = Modbus_string(slave_id, 6, address, value);

  // Simplified: Only support Hardware Serial (port)
  port->print(result);
  delay(50);

  while (port->available())
  {
    response = port->readStringUntil('\n');
  }

  if (response == "\0" || response == "")
  {
    Serial.println("CHECK YOUR SLAVE_ID / CONNECTION");
    return 0;
  }

  res = (response.substring(1, 16));
  j = LRC(res, 6);

  return (j == 1) ? 1 : 0;
}

String RMCS2303::Modbus_string(byte slave_id, byte FC, int address, unsigned int data)
{
  byte sum = 0;
  byte AddLoByte, AddHiByte, dataLoByte, dataHiByte;
  AddLoByte = (address & 0x00FF);
  AddHiByte = ((address & 0xFF00) >> 8);
  dataLoByte = (data & 0x00FF);
  dataHiByte = ((data & 0xFF00) >> 8);

  sum = 0;
  sum -= slave_id;
  sum -= FC;
  sum -= AddLoByte;
  sum -= AddHiByte;
  sum -= dataLoByte;
  sum -= dataHiByte;

  String SLAVE = print_hex(slave_id);
  String FCN = print_hex(FC);
  String AddHi = print_hex(AddHiByte);
  String AddLo = print_hex(AddLoByte);
  String dataHi = print_hex(dataHiByte);
  String dataLo = print_hex(dataLoByte);
  String LRC = print_hex(sum);
  String result = ":" + SLAVE + FCN + AddHi + AddLo + dataHi + dataLo + LRC + "\r\n";

  return result;
}

String RMCS2303::print_hex(byte number)
{
  String value = String(number, HEX);
  value.toUpperCase();
  if (value.length() == 1)
  {
    value = "0" + value;
  }
  return (value);
}

String RMCS2303::ReadSingleRegister(byte slave_id, int address, unsigned int No_register)
{
  String response, res;
  byte j;
  String result = Modbus_string(slave_id, 3, address, No_register);

  // Simplified: Only support Hardware Serial
  port->print(result);
  delay(20);

  while (port->available() > 0)
  {
    response = (port->readStringUntil('\n'));
    Serial.print("<<"); // Echo to debug
  }

  if (response.length() < 18)
    return "0"; // Safety check

  res = response.substring(1, 18);
  j = LRC(res, 7);

  if (j == 1)
    return (response);
  else
    return "0";
}

byte RMCS2303::WRITE_PARAMETER(byte slave_id, int INP_CONTROL_MODE, int PP_GAIN, int PI_GAIN, int VF_GAIN, int LPR, int acceleration, int SPEED)
{
  byte W1, W3, W4, W5, W6, W7, W8;

  W1 = WriteSingleRegister(slave_id, 2, INP_CONTROL_MODE);
  if (W1 == 1)
  {
    Serial.println("INP_CONTROL :          DONE");
    Serial.println("INP_MODE :             DONE");
  }
  else
  {
    Serial.print("error in writing parameters");
    return 0;
  }

  W3 = ProportionalGain(slave_id, PP_GAIN);
  if (W3 == 1)
    Serial.println("PP_GAIN :              DONE");

  W4 = IntegralGain(slave_id, PI_GAIN);
  if (W4 == 1)
    Serial.println("PI_GAIN :              DONE");

  W5 = FeedforwardGain(slave_id, VF_GAIN);
  if (W5 == 1)
    Serial.println("VF_GAIN :              DONE");

  W6 = EncoderPosition_LPR(slave_id, LPR);
  if (W6 == 1)
    Serial.println("LPR :                  DONE");

  W7 = Acceleration(slave_id, acceleration);
  if (W7 == 1)
    Serial.println("ACCELERATION :         DONE");

  W8 = Speed(slave_id, SPEED);
  if (W8 == 1)
    Serial.println("SPEED :                DONE");

  if (W1 && W3 && W4 && W5 && W6 && W7 && W8)
  {
    byte c;
    c = SAVE(slave_id);
    if (c == 1)
    {
      Serial.println("ALL PERAMETER SET");
      Serial.print("\n");
    }
    else
      Serial.print("NOT SAVED");
  }
  return 1;
}

byte RMCS2303::READ_PARAMETER(byte slave_id)
{
  long int R1, R2, R3, R4, R5, R6, R7, R8, R9;

  R1 = READ_DEVICE_MODBUS_ADDRESS(slave_id);
  Serial.print("DEVICE_MODBUS_ADDRESS: ");
  Serial.println(R1);

  R2 = READ_INP_CONTROL_BYTE(slave_id);
  Serial.print("INP_CONTROL_BYTE: ");
  Serial.println(R2);

  R3 = READ_INP_MODE_BYTE(slave_id);
  Serial.print("INP_MODE_BYTE: ");
  Serial.println(R3);

  R4 = READ_PP_GAIN_BYTE(slave_id);
  Serial.print("PP_GAIN_BYTE: ");
  Serial.println(R4);

  R5 = READ_PI_GAIN_BYTE(slave_id);
  Serial.print("VI_GAIN_BYTE: ");
  Serial.println(R5);

  R6 = READ_VF_GAIN_BYTE(slave_id);
  Serial.print("VF_GAIN_BYTE: ");
  Serial.println(R6);

  R7 = READ_LINES_PER_ROT(slave_id);
  Serial.print("LINES_PER_ROT: ");
  Serial.println(R7);

  R8 = READ_TRP_ACL_WORD(slave_id);
  Serial.print("TRP_ACL_WORD: ");
  Serial.println(R8);

  R9 = READ_TRP_SPD_WORD(slave_id);
  Serial.print("TRP_SPD_WORD: ");
  Serial.println(R9);

  return 1;
}

// ... Keep all READ helper functions (READ_DEVICE_MODBUS_ADDRESS, etc.) same as original ...
long int RMCS2303::READ_DEVICE_MODBUS_ADDRESS(byte slave_id)
{
  String result, x;
  char a[9];
  char *endptr;
  result = ReadSingleRegister(slave_id, 1, 1);
  if (result == "0")
    return -1;
  x = result.substring(9, 11);
  x.toCharArray(a, 3);
  return strtoul(a, &endptr, 16);
}
long int RMCS2303::READ_INP_CONTROL_BYTE(byte slave_id)
{
  String result, x;
  char a[9];
  char *endptr;
  result = ReadSingleRegister(slave_id, 2, 1);
  if (result == "0")
    return -1;
  x = result.substring(9, 11);
  x.toCharArray(a, 3);
  return strtoul(a, &endptr, 16);
}
long int RMCS2303::READ_INP_MODE_BYTE(byte slave_id)
{
  String result, x;
  char a[9];
  char *endptr;
  result = ReadSingleRegister(slave_id, 3, 1);
  if (result == "0")
    return -1;
  x = result.substring(9, 11);
  x.toCharArray(a, 3);
  return strtoul(a, &endptr, 16);
}
long int RMCS2303::READ_PP_GAIN_BYTE(byte slave_id)
{
  String result;
  result = ReadSingleRegister(slave_id, 4, 1);
  return value(result);
}
long int RMCS2303::READ_PI_GAIN_BYTE(byte slave_id)
{
  String result;
  result = ReadSingleRegister(slave_id, 6, 1);
  return value(result);
}
long int RMCS2303::READ_VF_GAIN_BYTE(byte slave_id)
{
  String result;
  result = ReadSingleRegister(slave_id, 8, 1);
  return value(result);
}
long int RMCS2303::READ_LINES_PER_ROT(byte slave_id)
{
  String result;
  result = ReadSingleRegister(slave_id, 10, 1);
  return value(result);
}
long int RMCS2303::READ_TRP_ACL_WORD(byte slave_id)
{
  String result;
  result = ReadSingleRegister(slave_id, 12, 1);
  return value(result);
}
long int RMCS2303::READ_TRP_SPD_WORD(byte slave_id)
{
  String result;
  result = ReadSingleRegister(slave_id, 14, 1);
  return value(result);
}

byte RMCS2303::SAVE(byte slave_id)
{
  byte j;
  int save = ((slave_id << 8) | 0xFF);
  j = WriteSingleRegister(slave_id, 0, save);
  return (j == 1) ? 1 : 0;
}
byte RMCS2303::RESET(byte slave_id)
{
  byte j, k;
  j = WriteSingleRegister(slave_id, 4, 0);
  if (j == 1)
    k = SAVE(slave_id);
  return (k == 1) ? 1 : 0;
}
byte RMCS2303::ESTOP(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 0, 1792);
  return (j == 1) ? 1 : 0;
}
byte RMCS2303::STOP(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 0, 1793);
  return (j == 1) ? 1 : 0;
}
byte RMCS2303::SET_HOME(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 0, 2048);
  return (j == 1) ? 1 : 0;
}
byte RMCS2303::Restart(byte slave_id)
{
  byte j;
  j = WriteSingleRegister(slave_id, 0, 2304);
  return (j == 1) ? 1 : 0;
}

void RMCS2303::hexToBytes(byte *byteArray, char *hexString)
{
  byte currentByte = 0;
  byte byteIndex = 0;

  for (byte charIndex = 0; charIndex < strlen(hexString); charIndex++)
  {
    bool oddCharIndex = charIndex & 1;
    if (!oddCharIndex) // Even char
    {
      currentByte = nibble(hexString[charIndex]) << 4;
    }
    else
    {
      currentByte |= nibble(hexString[charIndex]);
      byteArray[byteIndex++] = currentByte;
      currentByte = 0;
    }
  }
}

byte RMCS2303::nibble(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return 0;
}

byte RMCS2303::LRC(String s, byte len)
{
  char *ary = const_cast<char *>(s.c_str());
  byte byteArray[MaxByteArraySize] = {0};
  hexToBytes(byteArray, ary);
  byte sum = 0;
  for (int i = 0; i < len; i++)
  {
    sum += byteArray[i];
  }
  byte LRC = (~sum) + 1;

  if (LRC == byteArray[len])
    return 1;
  else
  {
    // Only hardware port handling
    // port->end(); // Removed this line. Resetting serial on checksum error is usually bad practice, just return 0.
    return 0;
  }
}

long int RMCS2303::value(String input)
{
  String x;
  char a[9];
  int len;
  char *endptr;
  long int p;
  len = input.length();
  if (len < 4)
    return 0; // Safety
  x = input.substring(7, len - 3);
  x.toCharArray(a, len - 9);
  p = strtoul(a, &endptr, 16);
  return p;
}