#include "UrdGroundStation.h"

#if URD_GROUNDSTATION_ENABLE


UrdGroundStation::UrdGroundStation(int baudRate)
    : UrdBase(baudRate)
{
    gsStarted = false;

    printEnabled = false;
    debugEnabled = false;

#if LORA_MANAGER
    loraManager = nullptr;
    loraChangeRequested = false;
    loraChangeCommand = "";
#endif

#if SD_CARD
    sdManager = nullptr;
#endif

#if URD_GPS_ENABLE
    gpsSerial = nullptr;
    gpsManager = nullptr;

    gpsAvailable = false;
    gpsReceiving = false;
    gpsEverReceivedData = false;

    gpsLatitude = "~";
    gpsLongitude = "~";

    lastGpsFixTime = 0;
    lastGpsCharTime = 0;

    gpsFixTimeoutMs = 3000UL;
    gpsNoDataTimeoutMs = 5000UL;
#endif
}

void UrdGroundStation::setPrint(bool enable)
{
    printEnabled = enable;
}

void UrdGroundStation::setDebug(bool enable)
{
    debugEnabled = enable;
}

void UrdGroundStation::printInfo(const __FlashStringHelper* message)
{
    if (printEnabled)
    {
        Serial.print(F("[GS] "));
        Serial.println(message);
    }
}

void UrdGroundStation::debugInfo(const __FlashStringHelper* message)
{
    if (debugEnabled)
    {
        Serial.print(F("[DEBUG][GS] "));
        Serial.println(message);
    }
}

void UrdGroundStation::debugInfo(const String& message)
{
    if (debugEnabled)
    {
        Serial.print(F("[DEBUG][GS] "));
        Serial.println(message);
    }
}

void UrdGroundStation::startHandshake()
{
    appStarted = false;
    readyReceived = false;
    gsStarted = false;

    unsigned long timer = millis();

    Serial.println(F("Starting URD Ground Station..."));

    while (!appStarted)
    {
#if URD_BUZZER_ENABLE
        if (buzzerEnabled && !readyReceived && ((millis() - timer) >= 5000UL))
        {
            playDot();
            timer = millis();
        }
#endif

#if URD_GPS_ENABLE
        updateGps();
#endif

        String message;

        if (readAppMessage(message))
        {
            if (processReady(message)) { continue; }
            if (processReset(message)) { continue; }
            if (processGpsCoords(message)) { continue; }

            processUnknownMessage(message);
        }

        delay(10);
    }

    ledOn();
}

#if LORA_MANAGER

void UrdGroundStation::initLoRaManager(LoRaManager& manager)
{
    loraManager = &manager;
}

bool UrdGroundStation::processLoraChangeFrequency(const String& message)
{
    if (loraManager == nullptr)
    {
        debugInfo(F("LoRaManager is null."));
        return false;
    }

    if (message == UrdProtocol::LORA_CHANGE_REQUEST_APP)
    {
        loraChangeRequested = true;
        loraChangeCommand = "";

        printInfo(F("LoRa frequency change requested by app."));
        debugInfo(F("Waiting for VALS:FREQXXX<TAB>ADDRESS."));

        return true;
    }

    if (message.startsWith(UrdProtocol::LORA_VALUE_PREFIX))
    {
        if (!loraChangeRequested)
        {
            debugInfo(F("Received VALS without MUDAR_FREQUENCIA."));
            Serial.println(UrdProtocol::LORA_CHANGE_REQUEST_ERROR_APP);
            return true;
        }

        loraChangeCommand = message.substring(strlen(UrdProtocol::LORA_VALUE_PREFIX));
        loraChangeCommand.trim();

        if (!isValidLoraChangeCommand(loraChangeCommand))
        {
            debugInfo(F("Invalid LoRa change values."));
            Serial.println(UrdProtocol::LORA_CHANGE_REQUEST_ERROR_APP);

            resetLoraChangeState();
            return true;
        }

        debugInfo(String("LoRa command received: ") + loraChangeCommand);

        Serial.println(UrdProtocol::LORA_CHANGE_REQUEST_CONFIRMATION_APP);

        loraManager->sendLine(UrdProtocol::LORA_CHANGE_REQUEST_FLIGHT);

        printInfo(F("LoRa change request sent to flight computer."));

        return true;
    }

    if (message == UrdProtocol::LORA_CHANGE_REQUEST_CONFIRMATION_FLIGHT)
    {
        if (!loraChangeRequested || loraChangeCommand.length() == 0)
        {
            debugInfo(F("Received MUD4R_OK but no pending LoRa change."));

            resetLoraChangeState();
            return false;
        }

        printInfo(F("Flight computer confirmed LoRa change request."));

        String valuePacket = String(UrdProtocol::LORA_VALUE_PREFIX) + loraChangeCommand;

        loraManager->sendLine(valuePacket);

        debugInfo(String("Sent LoRa values to flight computer: ") + valuePacket);

        delay(100);

        if (!loraManager->changeFrequency(loraChangeCommand))
        {
            debugInfo(F("Ground station failed to change LoRa frequency."));

            Serial.println(UrdProtocol::LORA_CHANGE_DONE_ERROR_ON_GS_APP);

            resetLoraChangeState();
            return true;
        }

        printInfo(F("Ground station changed LoRa frequency. Waiting flight confirmation."));

        if (loraManager->waitForMessage(
                UrdProtocol::LORA_CHANGE_DONE_FLIGHT,
                UrdProtocol::LORA_CHANGE_TIMEOUT_MS
            ))
        {
            printInfo(F("Flight computer confirmed final LoRa change."));
            Serial.println(UrdProtocol::LORA_CHANGE_DONE_APP);

            resetLoraChangeState();
            return true;
        }

        debugInfo(F("Timeout waiting for MUD4R_CERTO. Rolling back."));

        Serial.println(UrdProtocol::LORA_CHANGE_DONE_ERROR_APP);

        loraManager->changeFrequency(String(UrdProtocol::GS_LORA_DEFAULT_FREQUENCY));

        resetLoraChangeState();
        return true;
    }

    if (message == UrdProtocol::LORA_CHANGE_DONE_ERROR_FLIGHT)
    {
        debugInfo(F("Flight computer reported LoRa change error."));

        Serial.println(UrdProtocol::LORA_CHANGE_DONE_ERROR_APP);

        loraManager->changeFrequency(String(UrdProtocol::GS_LORA_DEFAULT_FREQUENCY));

        resetLoraChangeState();
        return true;
    }

    return false;
}

bool UrdGroundStation::isValidLoraChangeCommand(const String& command)
{
    String msg = command;
    msg.trim();

    msg.replace("\\t", "\t");

    int tabIndex = msg.indexOf('\t');

    if (tabIndex < 0)
    {
        int spaceIndex = msg.indexOf(' ');

        if (spaceIndex >= 0)
        {
            tabIndex = spaceIndex;
        }
    }

    if (tabIndex < 0)
    {
        debugInfo(F("LoRa command has no tab separator."));
        return false;
    }

    String frequencyText = msg.substring(0, tabIndex);
    String addressText = msg.substring(tabIndex + 1);

    frequencyText.trim();
    addressText.trim();
    frequencyText.toUpperCase();

    if (!frequencyText.startsWith("FREQ"))
    {
        debugInfo(F("LoRa frequency does not start with FREQ."));
        return false;
    }

    String frequencyNumberText = frequencyText.substring(4);

    if (!isDecimalText(frequencyNumberText))
    {
        debugInfo(F("LoRa frequency number is not decimal."));
        return false;
    }

    int frequencyValue = frequencyNumberText.toInt();

    if (frequencyValue < 862 || frequencyValue > 931)
    {
        debugInfo(F("LoRa frequency outside 862-931."));
        return false;
    }

    if (!isDecimalText(addressText))
    {
        debugInfo(F("LoRa address is not decimal."));
        return false;
    }

    long addressValue = addressText.toInt();

    if (addressValue < 0 || addressValue > 65535L)
    {
        debugInfo(F("LoRa address outside 0-65535."));
        return false;
    }

    return true;
}

bool UrdGroundStation::isDecimalText(const String& text)
{
    if (text.length() == 0)
    {
        return false;
    }

    for (unsigned int i = 0; i < text.length(); i++)
    {
        char c = text[i];

        if (c < '0' || c > '9')
        {
            return false;
        }
    }

    return true;
}

void UrdGroundStation::resetLoraChangeState()
{
    loraChangeRequested = false;
    loraChangeCommand = "";
}

#endif

#if SD_CARD

void UrdGroundStation::initSDManager(SDManager& manager)
{
    sdManager = &manager;
}

#endif

#if URD_GPS_ENABLE

void UrdGroundStation::initGps(
    Stream& gpsStream,
    TinyGPSPlus& gps,
    unsigned long fixTimeoutMs,
    unsigned long noDataTimeoutMs
)
{
    gpsSerial = &gpsStream;
    gpsManager = &gps;

    gpsFixTimeoutMs = fixTimeoutMs;
    gpsNoDataTimeoutMs = noDataTimeoutMs;

    clearGpsCoordinates();

    gpsReceiving = false;
    gpsEverReceivedData = false;
    lastGpsCharTime = 0;
    lastGpsFixTime = 0;
}

void UrdGroundStation::setGpsCoordinates(const String& latitude, const String& longitude)
{
    gpsLatitude = latitude;
    gpsLongitude = longitude;
    gpsAvailable = true;
    lastGpsFixTime = millis();
}

void UrdGroundStation::clearGpsCoordinates()
{
    gpsLatitude = "~";
    gpsLongitude = "~";
    gpsAvailable = false;
}

bool UrdGroundStation::updateGps()
{
    if (gpsSerial == nullptr || gpsManager == nullptr)
    {
        gpsReceiving = false;
        gpsAvailable = false;
        return false;
    }

    bool receivedCharNow = false;

    while (gpsSerial->available() > 0)
    {
        char c = static_cast<char>(gpsSerial->read());

        gpsManager->encode(c);

        receivedCharNow = true;
        gpsEverReceivedData = true;
        lastGpsCharTime = millis();
    }

    if (receivedCharNow)
    {
        gpsReceiving = true;
    }
    else if (gpsNoDataTimeoutMs > 0 && lastGpsCharTime > 0)
    {
        gpsReceiving = ((millis() - lastGpsCharTime) <= gpsNoDataTimeoutMs);
    }
    else
    {
        gpsReceiving = false;
    }

    bool locationValid = gpsManager->location.isValid();
    bool locationFresh = false;

    if (locationValid)
    {
        unsigned long age = gpsManager->location.age();
        locationFresh = (gpsFixTimeoutMs == 0UL || age <= gpsFixTimeoutMs);
    }

    if (gpsManager->location.isUpdated() && locationValid)
    {
        gpsLatitude = String(gpsManager->location.lat(), 6);
        gpsLongitude = String(gpsManager->location.lng(), 6);
        lastGpsFixTime = millis();
    }

    gpsAvailable = (gpsReceiving && locationValid && locationFresh);

    if (!gpsAvailable)
    {
        gpsLatitude = "~";
        gpsLongitude = "~";
    }

    return gpsAvailable;
}

bool UrdGroundStation::hasGpsFix() const
{
    return gpsAvailable;
}

bool UrdGroundStation::isGpsReceiving() const
{
    return gpsReceiving;
}

bool UrdGroundStation::hasGpsEverReceivedData() const
{
    return gpsEverReceivedData;
}

String UrdGroundStation::getGpsLatitude() const
{
    return gpsLatitude;
}

String UrdGroundStation::getGpsLongitude() const
{
    return gpsLongitude;
}

unsigned long UrdGroundStation::getLastGpsFixTime() const
{
    return lastGpsFixTime;
}

unsigned long UrdGroundStation::getLastGpsCharTime() const
{
    return lastGpsCharTime;
}

unsigned long UrdGroundStation::getGpsCharsProcessed() const
{
    if (gpsManager == nullptr)
    {
        return 0;
    }

    return gpsManager->charsProcessed();
}

unsigned long UrdGroundStation::getGpsSentencesWithFix() const
{
    if (gpsManager == nullptr)
    {
        return 0;
    }

    return gpsManager->sentencesWithFix();
}

unsigned long UrdGroundStation::getGpsFailedChecksum() const
{
    if (gpsManager == nullptr)
    {
        return 0;
    }

    return gpsManager->failedChecksum();
}

unsigned long UrdGroundStation::getGpsLocationAge() const
{
    if (gpsManager == nullptr || !gpsManager->location.isValid())
    {
        return ULONG_MAX;
    }

    return gpsManager->location.age();
}

uint32_t UrdGroundStation::getGpsSatellites() const
{
    if (gpsManager == nullptr || !gpsManager->satellites.isValid())
    {
        return 0;
    }

    return gpsManager->satellites.value();
}

float UrdGroundStation::getGpsHdop() const
{
    if (gpsManager == nullptr || !gpsManager->hdop.isValid())
    {
        return -1.0f;
    }

    return gpsManager->hdop.hdop();
}

void UrdGroundStation::printGpsStatus(Stream& out)
{
    out.print(F("GPS receiving: "));
    out.print(gpsReceiving ? F("YES") : F("NO"));

    out.print(F(" | fix: "));
    out.print(gpsAvailable ? F("YES") : F("NO"));

    out.print(F(" | lat: "));
    out.print(gpsLatitude);

    out.print(F(" | lon: "));
    out.print(gpsLongitude);

    out.print(F(" | sats: "));
    out.print(getGpsSatellites());

    out.print(F(" | hdop: "));
    out.print(getGpsHdop(), 1);

    out.print(F(" | chars: "));
    out.print(getGpsCharsProcessed());

    out.print(F(" | fix sentences: "));
    out.print(getGpsSentencesWithFix());

    out.print(F(" | checksum fail: "));
    out.println(getGpsFailedChecksum());
}

#endif

bool UrdGroundStation::processGpsCoords(const String& message)
{
    if (message != UrdProtocol::GPS_COORDS)
    {
        return false;
    }

#if URD_GPS_ENABLE
    updateGps();

    if (hasGpsFix())
    {
        Serial.print(gpsLatitude);
        Serial.print('\t');
        Serial.println(gpsLongitude);
    }
    else
#endif
    {
        Serial.println(UrdProtocol::NO_GPS_COORDS);
    }

    if (!gsStarted)
    {
        gsStarted = true;
        markAppStarted();
        Serial.println(F("---Ground Station UFABC Rocket Design---"));
    }

#if URD_BUZZER_ENABLE
    if (buzzerEnabled)
    {
        playDash();
    }
#endif

    return true;
}

void UrdGroundStation::processAppSerialMessages()
{
    String message;

    while (readAppMessage(message))
    {
        message.trim();

        if (message.length() == 0)
        {
            continue;
        }

        if (processReady(message))
        {
            continue;
        }

        if (processReset(message))
        {
            continue;
        }

        if (processGpsCoords(message))
        {
            continue;
        }

#if LORA_MANAGER
        if (processLoraChangeFrequency(message))
        {
            continue;
        }
#endif

        if (debugEnabled)
        {
            Serial.print(F("[DEBUG][GS] Unknown app message: "));
            Serial.println(message);
        }
    }
}

void UrdGroundStation::processLoraMessages()
{
#if LORA_MANAGER
    if (loraManager == nullptr)
    {
        return;
    }

    String packet;

    while (loraManager->readLine(packet))
    {
        packet.trim();

        if (packet.length() == 0)
        {
            continue;
        }

        if (processLoraChangeFrequency(packet))
        {
            continue;
        }

        processLoraPacket(packet);
    }
#endif
}

void UrdGroundStation::update()
{
#if URD_GPS_ENABLE
    updateGps();
#endif

    if (!appStarted)
    {
        return;
    }

    processAppSerialMessages();

    processLoraMessages();
}


void UrdGroundStation::processLoraPacket(const String& packet)
{
    if (packet.length() == 0)
    {
        return;
    }

#if URD_BUZZER_ENABLE
    if (buzzerEnabled)
    {
        playDot();
    }
#endif

    Serial.println(packet);

#if SD_CARD
    if (sdManager != nullptr && sdManager->isReady())
    {
        if (sdManager->writeLine(packet))
        {
#if URD_BUZZER_ENABLE
            if (buzzerEnabled)
            {
                playDot();
            }
#endif
        }
        else
        {
            debugInfo(F("Failed to save LoRa packet to SD."));
        }
    }
#endif
}

bool UrdGroundStation::isGsStarted() const
{
    return gsStarted;
}

#endif
