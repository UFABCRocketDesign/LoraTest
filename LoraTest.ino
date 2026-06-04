// =======================================================
// URD LoRa Change Tester
// Simulates the FlightComputer side for frequency change tests.
// It also forwards Serial commands to LoRa.
// =======================================================

// =======================================================
// CONFIGURATION
// Adjust these pins for your board.
// =======================================================
#define LORA_MANAGER_E32 1 
#define LORA_MANAGER_E22 0

#define LORA_FREQUENCY 900


// ----------------------- INCLUDES ----------------------
#include "src/lib/config.h" 
#include "src/lib/pinout.h"

#if LORA_FREQUENCY == 900
  #define FREQUENCY_900
  #define LORA_MAX_CHANNEL 69
#elif LORA_FREQUENCY == 433
  #define FREQUENCY_433
  #define LORA_MAX_CHANNEL 0
  #error "433Mhz Not implemented yet."
#else
  #error "Invalid LoRa Frequency. Use 900 or 433 MHz."
#endif

#if LORA_MANAGER_E32
  #include <LoRa_E32.h>
#elif LORA_MANAGER_E22
  #include <LoRa_E22.h>
#else
  #error "No LoRa Manager defined. Define LORA_MANAGER_E32 or LORA_MANAGER_E22."
#endif

#include "src/lib/URDCore/URDCore.h"
#include "src/lib/LoRaManager/LoRaManager.h"

#if (LORA_MANAGER_E32 && LORA_MANAGER_E22)
  #error "Multiple LoRa Managers defined. Define only one of LORA_MANAGER_E32 or LORA_MANAGER_E22."
#endif
// ----------------------- SERIAL ------------------------
#define SERIAL_BAUD_RATE 115200

// ------------------------ LoRa -------------------------
#define LORA_BAUD_RATE 9600

#if defined(ARDUINO_ARCH_ESP32)

// ESP32 / ESP32-S3
HardwareSerial LoRaSerial(1);

#else

// Arduino Mega (Placa Octa)
// SoftwareSerial LoRaSerial(LORA_RX, LORA_TX);
HardwareSerial& LoRaSerial = Serial2;

#endif

// =======================================================
// OBJECTS
// =======================================================
#if (LORA_MANAGER)
LoRa_E32 e32Module(
    &LoRaSerial,
    AUX_LORA_PIN,
    M0_LORA_PIN,
    M1_LORA_PIN
);

LoRaManager loraManager(LoRaSerial);
#endif
// =======================================================
// HELPERS
// =======================================================

bool readSerialLine(String& line)
{
    if (!Serial.available())
    {
        return false;
    }

    line = Serial.readStringUntil('\n');
    line.trim();

    return line.length() > 0;
}


bool isFrequencyCommand(const String& line)
{
    String command = line;
    command.trim();
    command.toUpperCase();

    return command.startsWith("FREQ");
}

void processSerialCommand(const String& line)
{
    String command = line;
    command.trim();

    if (command.length() == 0)
    {
        return;
    }

    String upperCommand = command;
    upperCommand.toUpperCase();

    if (upperCommand == "HELP")
    {
        printHelp();
        return;
    }

    if (upperCommand == "M0:1")
    {
        digitalWrite(M0_LORA_PIN, HIGH);
        return;
    }

    if (upperCommand == "M0:0")
    {
        digitalWrite(M0_LORA_PIN, LOW);
        return;
    }

    if (upperCommand == "M1:1")
    {
        digitalWrite(M1_LORA_PIN, HIGH);
        return;
    }

    if (upperCommand == "M1:0")
    {
        digitalWrite(M1_LORA_PIN, LOW);
        return;
    }

    uint8_t chan = 0;
    uint8_t addh = 0;
    uint8_t addl = 0;

    if (decodeFrequency(command, chan, addh, addl))
    {
        applyFrequency(chan, addh, addl);
        return;
    }

    Serial.print(F("[SERIAL > LORA] "));
    Serial.println(command);

    loraManager.sendLine(command);
}

void processLoRaCommand(const String& packet)
{
    String command = packet;
    command.trim();

    if (command.length() == 0)
    {
        return;
    }

    // Tudo que chega do LoRa aparece no terminal
    Serial.print(F("[LORA > SERIAL] "));
    Serial.println(command);

    // if (isFrequencyCommand(command))
    // {
    //     Serial.println(F("[REMOTE CONFIG] Frequency command received through LoRa."));
    //     loraManager.changeFrequency(command);
    //     return;
    // }

}

void printHelp()
{
    Serial.println();
    Serial.println(F("=== URD LoRa Direct Tester HELP ==="));
    Serial.println();

    Serial.println(F("Local pin commands:"));
    Serial.println(F("  Send M0:X M1:Y to change the M0 and M1 pins (only one pin per message)"));
    Serial.println(F("  EX- M1:0"));

    Serial.println();

    Serial.println(F("LoRa config command:"));
    Serial.println(F("  Write this for changing LoRa Frequency and Address."));
    Serial.println(F("  CHAN44_0110"));
    Serial.println(F("  CHAN = decimal channel, from 00 to 69 for changing frequency"));
    Serial.println(F("  ADDRESS = hexadecimal address of 4 digits"));

    Serial.println(F("Raw LoRa message:"));
    Serial.println(F("  Any other text typed here is sent through LoRa."));
    Serial.println();

    Serial.println(F("LoRa receive behavior:"));
    Serial.println(F("  Any received LoRa packet is printed on Serial."));
    Serial.println();
}

bool isHexText(const String& text)
{
    if (text.length() == 0)
    {
        return false;
    }

    for (unsigned int i = 0; i < text.length(); i++)
    {
        char c = text[i];

        if (!((c >= '0' && c <= '9') ||
              (c >= 'A' && c <= 'F') ||
              (c >= 'a' && c <= 'f')))
        {
            return false;
        }
    }

    return true;
}

bool isDecimalText(const String& text) {
    if (text.length() == 0) return false;

    for (size_t i = 0; i < text.length(); i++) {
        if (!isDigit(text[i])) {
            return false;
        }
    }

    return true;
}

bool decodeFrequency(
    const String& command,
    uint8_t& chan,
    uint8_t& addh,
    uint8_t& addl
)
{
    String msg = command;
    msg.trim();
    msg.toUpperCase();

    // Formato aceito:
    // CHANXX_A1B2

    if (!msg.startsWith("CHAN"))
    {
        return false;
    }

    int separatorIndex = msg.indexOf('_');

    if (separatorIndex < 0)
    {
        return false;
    }

    String chanText = msg.substring(4, separatorIndex);
    String addressText = msg.substring(separatorIndex + 1);

    chanText.trim();
    addressText.trim();

    if (chanText.length() != 2)
    {
        return false;
    }

    if (addressText.length() != 4)
    {
        return false;
    }

    if (!isDecimalText(chanText))
    {
        return false;
    }

    if (!isHexText(addressText))
    {
        return false;
    }

    unsigned long chanValue = strtoul(chanText.c_str(), nullptr, 10);
    unsigned long addressValue = strtoul(addressText.c_str(), nullptr, 16);

    if (chanValue > LORA_MAX_CHANNEL)
    {
        return false;
    }

    if (addressValue > 0xFFFFUL)
    {
        return false;
    }

    chan = static_cast<uint8_t>(chanValue);
    addh = static_cast<uint8_t>((addressValue >> 8) & 0xFF);
    addl = static_cast<uint8_t>(addressValue & 0xFF);

    return true;
}

bool applyFrequency(
    uint8_t chan,
    uint8_t addh,
    uint8_t addl
)
{
    Serial.print(F("[APPLYING] CHAN="));
    Serial.print(chan);

    Serial.print(F(" ADDH=0x"));
    if (addh < 0x10) Serial.print('0');
    Serial.print(addh, HEX);

    Serial.print(F(" ADDL=0x"));
    if (addl < 0x10) Serial.print('0');
    Serial.println(addl, HEX);

    bool success = loraManager.changeFrequency(chan, addh, addl);

    Serial.println(success ? F("SUCCESS") : F("ERROR"));

    return success;
}
// =======================================================
// SETUP
// =======================================================

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    Serial.println();
    Serial.println(F("=== URD LoRa Tester ==="));
#if (LORA_MANAGER)
#if defined(ARDUINO_ARCH_ESP32)
    LoRaSerial.begin(
        LORA_BAUD_RATE,
        SERIAL_8N1,
        LORA_TX,
        LORA_RX
    );
#else
    LoRaSerial.begin(LORA_BAUD_RATE);
#endif

    pinMode(AUX_LORA_PIN, INPUT);
    pinMode(M0_LORA_PIN, OUTPUT);
    pinMode(M1_LORA_PIN, OUTPUT);
    digitalWrite(M0_LORA_PIN, LOW);
    digitalWrite(M1_LORA_PIN, LOW);

    loraManager.setPrint(true);
    loraManager.setDebug(true);

    loraManager.initE32(e32Module); // The LoRa Manager external lib initialization is inside this method

#endif

    Serial.println(F("Ready."));
    Serial.println(F("Type HELP for commands."));
}

// =======================================================
// LOOP
// =======================================================

void loop()
{
    String serialLine;

    while (readSerialLine(serialLine))
    {
        processSerialCommand(serialLine);
    }

    String loraPacket;

    while (loraManager.readLine(loraPacket))
    {
        processLoRaCommand(loraPacket);
    }
}
