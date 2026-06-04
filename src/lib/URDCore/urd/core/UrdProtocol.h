#ifndef URD_PROTOCOL_H
#define URD_PROTOCOL_H

#pragma once

#if URD_CORE_ENABLE
namespace UrdProtocol
{
    static const char READY[] = "READY";
    static const char OK[] = "OK";

    static const char RESET[] = "RST";
    static const char RESETTING[] = "RESETTING";

    static const char GPS_COORDS[] = "GPS_COORDS";
    static const char NO_GPS_COORDS[] = "~\t~";

    static const char SIMULATION[] = "SIMULATION";
    static const char STARTED[] = "STARTED";

    static const char LORA_VALUE_PREFIX[] = "VALS:";
    static const char LORA_CHANGE_REQUEST_APP[] = "MUDAR_FREQUENCIA";
    static const char LORA_CHANGE_REQUEST_FLIGHT[] = "MUD4R_FR3Q";
    static const char LORA_CHANGE_REQUEST_ERROR_APP[] = "MUDAR_FREQUENCIA_ERROR";
    static const char LORA_CHANGE_REQUEST_CONFIRMATION_APP[] = "MUDAR_OK";
    static const char LORA_CHANGE_REQUEST_CONFIRMATION_FLIGHT[] = "MUD4R_OK";
    static const char LORA_CHANGE_DONE_APP[] = "MUDAR_CERTO";
    static const char LORA_CHANGE_DONE_FLIGHT[] = "MUD4R_CERTO";
    static const char LORA_CHANGE_DONE_ERROR_ON_GS_APP[] = "MUDAR_ERRO_GS";
    static const char LORA_CHANGE_DONE_ERROR_APP[] = "MUDAR_ERRO";
    static const char LORA_CHANGE_DONE_ERROR_FLIGHT[] = "MUD4R_ERRO";

    static const char GS_LORA_DEFAULT_FREQUENCY[] = "FREQ903\t44";
    static const unsigned long LORA_CHANGE_TIMEOUT_MS = 10000UL;
}
#endif
#endif

