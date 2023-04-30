/*
 * This file is part of libfreespace.
 * 
 * Copyright (c) 2009-2012 Hillcrest Laboratories, Inc. 
 *
 * libfreespace is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "freespace/freespace_printers.h"
#include <string.h>

static void printUnknown(const char* name, const uint8_t* buffer, int length) {
    int i;
    printf("%s(", name);
    for (i = 0; i < length; ++i) {
        printf("%02x ", (uint8_t) buffer[i]);
    }
    printf(")\n");
}

int freespace_printMessageStr(char* dest, int maxlen, const struct freespace_message* s) {
    switch(s->messageType) {
    case FREESPACE_MESSAGE_PAIRINGMESSAGE:
        return freespace_printPairingMessageStr(dest, maxlen, &(s->pairingMessage));
        break;
    case FREESPACE_MESSAGE_PRODUCTIDREQUEST:
        return freespace_printProductIDRequestStr(dest, maxlen, &(s->productIDRequest));
        break;
    case FREESPACE_MESSAGE_LEDSETREQUEST:
        return freespace_printLEDSetRequestStr(dest, maxlen, &(s->lEDSetRequest));
        break;
    case FREESPACE_MESSAGE_LINKQUALITYREQUEST:
        return freespace_printLinkQualityRequestStr(dest, maxlen, &(s->linkQualityRequest));
        break;
    case FREESPACE_MESSAGE_ALWAYSONREQUEST:
        return freespace_printAlwaysOnRequestStr(dest, maxlen, &(s->alwaysOnRequest));
        break;
    case FREESPACE_MESSAGE_FREQUENCYFIXREQUEST:
        return freespace_printFrequencyFixRequestStr(dest, maxlen, &(s->frequencyFixRequest));
        break;
    case FREESPACE_MESSAGE_SOFTWARERESETMESSAGE:
        return freespace_printSoftwareResetMessageStr(dest, maxlen, &(s->softwareResetMessage));
        break;
    case FREESPACE_MESSAGE_DONGLERFDISABLEMESSAGE:
        return freespace_printDongleRFDisableMessageStr(dest, maxlen, &(s->dongleRFDisableMessage));
        break;
    case FREESPACE_MESSAGE_TXDISABLEMESSAGE:
        return freespace_printTxDisableMessageStr(dest, maxlen, &(s->txDisableMessage));
        break;
    case FREESPACE_MESSAGE_DONGLERFSUPRESSHOMEFREQUENCYMESSAGE:
        return freespace_printDongleRFSupressHomeFrequencyMessageStr(dest, maxlen, &(s->dongleRFSupressHomeFrequencyMessage));
        break;
    case FREESPACE_MESSAGE_FRSHANDHELDREADREQUEST:
        return freespace_printFRSHandheldReadRequestStr(dest, maxlen, &(s->fRSHandheldReadRequest));
        break;
    case FREESPACE_MESSAGE_FRSHANDHELDWRITEREQUEST:
        return freespace_printFRSHandheldWriteRequestStr(dest, maxlen, &(s->fRSHandheldWriteRequest));
        break;
    case FREESPACE_MESSAGE_FRSHANDHELDWRITEDATA:
        return freespace_printFRSHandheldWriteDataStr(dest, maxlen, &(s->fRSHandheldWriteData));
        break;
    case FREESPACE_MESSAGE_FRSDONGLEREADREQUEST:
        return freespace_printFRSDongleReadRequestStr(dest, maxlen, &(s->fRSDongleReadRequest));
        break;
    case FREESPACE_MESSAGE_FRSDONGLEWRITEREQUEST:
        return freespace_printFRSDongleWriteRequestStr(dest, maxlen, &(s->fRSDongleWriteRequest));
        break;
    case FREESPACE_MESSAGE_FRSDONGLEWRITEDATA:
        return freespace_printFRSDongleWriteDataStr(dest, maxlen, &(s->fRSDongleWriteData));
        break;
    case FREESPACE_MESSAGE_FRSEFLASHREADREQUEST:
        return freespace_printFRSEFlashReadRequestStr(dest, maxlen, &(s->fRSEFlashReadRequest));
        break;
    case FREESPACE_MESSAGE_FRSEFLASHWRITEREQUEST:
        return freespace_printFRSEFlashWriteRequestStr(dest, maxlen, &(s->fRSEFlashWriteRequest));
        break;
    case FREESPACE_MESSAGE_FRSEFLASHWRITEDATA:
        return freespace_printFRSEFlashWriteDataStr(dest, maxlen, &(s->fRSEFlashWriteData));
        break;
    case FREESPACE_MESSAGE_DONGLERFENABLEMESSAGE:
        return freespace_printDongleRFEnableMessageStr(dest, maxlen, &(s->dongleRFEnableMessage));
        break;
    case FREESPACE_MESSAGE_DATAMODEREQUEST:
        return freespace_printDataModeRequestStr(dest, maxlen, &(s->dataModeRequest));
        break;
    case FREESPACE_MESSAGE_BUTTONTESTMODEREQUEST:
        return freespace_printButtonTestModeRequestStr(dest, maxlen, &(s->buttonTestModeRequest));
        break;
    case FREESPACE_MESSAGE_PAIRINGRESPONSE:
        return freespace_printPairingResponseStr(dest, maxlen, &(s->pairingResponse));
        break;
    case FREESPACE_MESSAGE_PRODUCTIDRESPONSE:
        return freespace_printProductIDResponseStr(dest, maxlen, &(s->productIDResponse));
        break;
    case FREESPACE_MESSAGE_LINKSTATUS:
        return freespace_printLinkStatusStr(dest, maxlen, &(s->linkStatus));
        break;
    case FREESPACE_MESSAGE_ALWAYSONRESPONSE:
        return freespace_printAlwaysOnResponseStr(dest, maxlen, &(s->alwaysOnResponse));
        break;
    case FREESPACE_MESSAGE_FRSHANDHELDREADRESPONSE:
        return freespace_printFRSHandheldReadResponseStr(dest, maxlen, &(s->fRSHandheldReadResponse));
        break;
    case FREESPACE_MESSAGE_FRSHANDHELDWRITERESPONSE:
        return freespace_printFRSHandheldWriteResponseStr(dest, maxlen, &(s->fRSHandheldWriteResponse));
        break;
    case FREESPACE_MESSAGE_FRSDONGLEREADRESPONSE:
        return freespace_printFRSDongleReadResponseStr(dest, maxlen, &(s->fRSDongleReadResponse));
        break;
    case FREESPACE_MESSAGE_FRSDONGLEWRITERESPONSE:
        return freespace_printFRSDongleWriteResponseStr(dest, maxlen, &(s->fRSDongleWriteResponse));
        break;
    case FREESPACE_MESSAGE_FRSEFLASHREADRESPONSE:
        return freespace_printFRSEFlashReadResponseStr(dest, maxlen, &(s->fRSEFlashReadResponse));
        break;
    case FREESPACE_MESSAGE_FRSEFLASHWRITERESPONSE:
        return freespace_printFRSEFlashWriteResponseStr(dest, maxlen, &(s->fRSEFlashWriteResponse));
        break;
    case FREESPACE_MESSAGE_DATAMODERESPONSE:
        return freespace_printDataModeResponseStr(dest, maxlen, &(s->dataModeResponse));
        break;
    case FREESPACE_MESSAGE_BUTTONTESTMODERESPONSE:
        return freespace_printButtonTestModeResponseStr(dest, maxlen, &(s->buttonTestModeResponse));
        break;
    case FREESPACE_MESSAGE_BATTERYLEVELREQUEST:
        return freespace_printBatteryLevelRequestStr(dest, maxlen, &(s->batteryLevelRequest));
        break;
    case FREESPACE_MESSAGE_BATTERYLEVEL:
        return freespace_printBatteryLevelStr(dest, maxlen, &(s->batteryLevel));
        break;
    case FREESPACE_MESSAGE_BODYFRAME:
        return freespace_printBodyFrameStr(dest, maxlen, &(s->bodyFrame));
        break;
    case FREESPACE_MESSAGE_USERFRAME:
        return freespace_printUserFrameStr(dest, maxlen, &(s->userFrame));
        break;
    case FREESPACE_MESSAGE_DATAMOTIONCONTROL:
        return freespace_printDataMotionControlStr(dest, maxlen, &(s->dataMotionControl));
        break;
    case FREESPACE_MESSAGE_FRSWRITERESPONSE:
        return freespace_printFRSWriteResponseStr(dest, maxlen, &(s->fRSWriteResponse));
        break;
    case FREESPACE_MESSAGE_DATAMODECONTROLV2RESPONSE:
        return freespace_printDataModeControlV2ResponseStr(dest, maxlen, &(s->dataModeControlV2Response));
        break;
    case FREESPACE_MESSAGE_SENSORPERIODRESPONSE:
        return freespace_printSensorPeriodResponseStr(dest, maxlen, &(s->sensorPeriodResponse));
        break;
    case FREESPACE_MESSAGE_FRSREADRESPONSE:
        return freespace_printFRSReadResponseStr(dest, maxlen, &(s->fRSReadResponse));
        break;
    case FREESPACE_MESSAGE_PERRESPONSE:
        return freespace_printPerResponseStr(dest, maxlen, &(s->perResponse));
        break;
    case FREESPACE_MESSAGE_FRSWRITEREQUEST:
        return freespace_printFRSWriteRequestStr(dest, maxlen, &(s->fRSWriteRequest));
        break;
    case FREESPACE_MESSAGE_FRSWRITEDATA:
        return freespace_printFRSWriteDataStr(dest, maxlen, &(s->fRSWriteData));
        break;
    case FREESPACE_MESSAGE_FRSREADREQUEST:
        return freespace_printFRSReadRequestStr(dest, maxlen, &(s->fRSReadRequest));
        break;
    case FREESPACE_MESSAGE_PERREQUEST:
        return freespace_printPerRequestStr(dest, maxlen, &(s->perRequest));
        break;
    case FREESPACE_MESSAGE_ACTIVITYCLASSIFICATIONNOTIFICATION:
        return freespace_printActivityClassificationNotificationStr(dest, maxlen, &(s->activityClassificationNotification));
        break;
    case FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST:
        return freespace_printDataModeControlV2RequestStr(dest, maxlen, &(s->dataModeControlV2Request));
        break;
    case FREESPACE_MESSAGE_REORIENTATIONREQUEST:
        return freespace_printReorientationRequestStr(dest, maxlen, &(s->reorientationRequest));
        break;
    case FREESPACE_MESSAGE_SENSORPERIODREQUEST:
        return freespace_printSensorPeriodRequestStr(dest, maxlen, &(s->sensorPeriodRequest));
        break;
    case FREESPACE_MESSAGE_BMSREQUEST:
        return freespace_printBmsRequestStr(dest, maxlen, &(s->bmsRequest));
        break;
    case FREESPACE_MESSAGE_PRODUCTIDRESPONSEBLE:
        return freespace_printProductIDResponseBLEStr(dest, maxlen, &(s->productIDResponseBLE));
        break;
    case FREESPACE_MESSAGE_FRSREADRESPONSEBLE:
        return freespace_printFRSReadResponseBLEStr(dest, maxlen, &(s->fRSReadResponseBLE));
        break;
    case FREESPACE_MESSAGE_BODYUSERFRAME:
        return freespace_printBodyUserFrameStr(dest, maxlen, &(s->bodyUserFrame));
        break;
    case FREESPACE_MESSAGE_MOTIONENGINEOUTPUT:
        return freespace_printMotionEngineOutputStr(dest, maxlen, &(s->motionEngineOutput));
        break;
    case FREESPACE_MESSAGE_DCEOUTV2:
        return freespace_printDceOutV2Str(dest, maxlen, &(s->dceOutV2));
        break;
    case FREESPACE_MESSAGE_DCEOUTV3:
        return freespace_printDceOutV3Str(dest, maxlen, &(s->dceOutV3));
        break;
    case FREESPACE_MESSAGE_DCEOUTV4T0:
        return freespace_printDceOutV4T0Str(dest, maxlen, &(s->dceOutV4T0));
        break;
    case FREESPACE_MESSAGE_DCEOUTV4T1:
        return freespace_printDceOutV4T1Str(dest, maxlen, &(s->dceOutV4T1));
        break;
    default:
        return -1;
    }
}

void freespace_printMessage(FILE* fp, const struct freespace_message * s) {
    char buf[1024];
    int rc = freespace_printMessageStr(buf, sizeof(buf), s);
    
    if (rc < 0) {
        fprintf(fp, "invalid messages\n");
        return;
    }
    fprintf(fp, "%s\n", buf);
}


LIBFREESPACE_API int freespace_printPairingMessageStr(char* dest, int maxlen, const struct freespace_PairingMessage* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "PairingMessage()");
#else
    n = snprintf(dest, maxlen, "PairingMessage()");
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printPairingMessage(FILE* fp, const struct freespace_PairingMessage* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printPairingMessageStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printProductIDRequestStr(char* dest, int maxlen, const struct freespace_ProductIDRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "ProductIDRequest(Format=%d)", s->Format);
#else
    n = snprintf(dest, maxlen, "ProductIDRequest(Format=%d)", s->Format);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printProductIDRequest(FILE* fp, const struct freespace_ProductIDRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printProductIDRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printLEDSetRequestStr(char* dest, int maxlen, const struct freespace_LEDSetRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "LEDSetRequest(onOff=%d selectLED=%d)", s->onOff, s->selectLED);
#else
    n = snprintf(dest, maxlen, "LEDSetRequest(onOff=%d selectLED=%d)", s->onOff, s->selectLED);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printLEDSetRequest(FILE* fp, const struct freespace_LEDSetRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printLEDSetRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printLinkQualityRequestStr(char* dest, int maxlen, const struct freespace_LinkQualityRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "LinkQualityRequest(enable=%d)", s->enable);
#else
    n = snprintf(dest, maxlen, "LinkQualityRequest(enable=%d)", s->enable);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printLinkQualityRequest(FILE* fp, const struct freespace_LinkQualityRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printLinkQualityRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printAlwaysOnRequestStr(char* dest, int maxlen, const struct freespace_AlwaysOnRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "AlwaysOnRequest()");
#else
    n = snprintf(dest, maxlen, "AlwaysOnRequest()");
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printAlwaysOnRequest(FILE* fp, const struct freespace_AlwaysOnRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printAlwaysOnRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFrequencyFixRequestStr(char* dest, int maxlen, const struct freespace_FrequencyFixRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FrequencyFixRequest(channel0=%d channel1=%d channel2=%d channel3=%d channel4=%d device=%d)", s->channel0, s->channel1, s->channel2, s->channel3, s->channel4, s->device);
#else
    n = snprintf(dest, maxlen, "FrequencyFixRequest(channel0=%d channel1=%d channel2=%d channel3=%d channel4=%d device=%d)", s->channel0, s->channel1, s->channel2, s->channel3, s->channel4, s->device);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFrequencyFixRequest(FILE* fp, const struct freespace_FrequencyFixRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFrequencyFixRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printSoftwareResetMessageStr(char* dest, int maxlen, const struct freespace_SoftwareResetMessage* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "SoftwareResetMessage(device=%d)", s->device);
#else
    n = snprintf(dest, maxlen, "SoftwareResetMessage(device=%d)", s->device);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printSoftwareResetMessage(FILE* fp, const struct freespace_SoftwareResetMessage* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printSoftwareResetMessageStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDongleRFDisableMessageStr(char* dest, int maxlen, const struct freespace_DongleRFDisableMessage* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DongleRFDisableMessage()");
#else
    n = snprintf(dest, maxlen, "DongleRFDisableMessage()");
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDongleRFDisableMessage(FILE* fp, const struct freespace_DongleRFDisableMessage* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDongleRFDisableMessageStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printTxDisableMessageStr(char* dest, int maxlen, const struct freespace_TxDisableMessage* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "TxDisableMessage()");
#else
    n = snprintf(dest, maxlen, "TxDisableMessage()");
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printTxDisableMessage(FILE* fp, const struct freespace_TxDisableMessage* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printTxDisableMessageStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDongleRFSupressHomeFrequencyMessageStr(char* dest, int maxlen, const struct freespace_DongleRFSupressHomeFrequencyMessage* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DongleRFSupressHomeFrequencyMessage(low=%d high=%d)", s->low, s->high);
#else
    n = snprintf(dest, maxlen, "DongleRFSupressHomeFrequencyMessage(low=%d high=%d)", s->low, s->high);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDongleRFSupressHomeFrequencyMessage(FILE* fp, const struct freespace_DongleRFSupressHomeFrequencyMessage* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDongleRFSupressHomeFrequencyMessageStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSHandheldReadRequestStr(char* dest, int maxlen, const struct freespace_FRSHandheldReadRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSHandheldReadRequest(wordOffset=%d FRStype=%d BlockSize=%d)", s->wordOffset, s->FRStype, s->BlockSize);
#else
    n = snprintf(dest, maxlen, "FRSHandheldReadRequest(wordOffset=%d FRStype=%d BlockSize=%d)", s->wordOffset, s->FRStype, s->BlockSize);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSHandheldReadRequest(FILE* fp, const struct freespace_FRSHandheldReadRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSHandheldReadRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSHandheldWriteRequestStr(char* dest, int maxlen, const struct freespace_FRSHandheldWriteRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSHandheldWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSHandheldWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSHandheldWriteRequest(FILE* fp, const struct freespace_FRSHandheldWriteRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSHandheldWriteRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSHandheldWriteDataStr(char* dest, int maxlen, const struct freespace_FRSHandheldWriteData* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSHandheldWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#else
    n = snprintf(dest, maxlen, "FRSHandheldWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSHandheldWriteData(FILE* fp, const struct freespace_FRSHandheldWriteData* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSHandheldWriteDataStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSDongleReadRequestStr(char* dest, int maxlen, const struct freespace_FRSDongleReadRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSDongleReadRequest(wordOffset=%d FRStype=%d BlockSize=%d)", s->wordOffset, s->FRStype, s->BlockSize);
#else
    n = snprintf(dest, maxlen, "FRSDongleReadRequest(wordOffset=%d FRStype=%d BlockSize=%d)", s->wordOffset, s->FRStype, s->BlockSize);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSDongleReadRequest(FILE* fp, const struct freespace_FRSDongleReadRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSDongleReadRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSDongleWriteRequestStr(char* dest, int maxlen, const struct freespace_FRSDongleWriteRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSDongleWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSDongleWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSDongleWriteRequest(FILE* fp, const struct freespace_FRSDongleWriteRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSDongleWriteRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSDongleWriteDataStr(char* dest, int maxlen, const struct freespace_FRSDongleWriteData* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSDongleWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#else
    n = snprintf(dest, maxlen, "FRSDongleWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSDongleWriteData(FILE* fp, const struct freespace_FRSDongleWriteData* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSDongleWriteDataStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSEFlashReadRequestStr(char* dest, int maxlen, const struct freespace_FRSEFlashReadRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSEFlashReadRequest(wordOffset=%d FRStype=%d BlockSize=%d)", s->wordOffset, s->FRStype, s->BlockSize);
#else
    n = snprintf(dest, maxlen, "FRSEFlashReadRequest(wordOffset=%d FRStype=%d BlockSize=%d)", s->wordOffset, s->FRStype, s->BlockSize);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSEFlashReadRequest(FILE* fp, const struct freespace_FRSEFlashReadRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSEFlashReadRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSEFlashWriteRequestStr(char* dest, int maxlen, const struct freespace_FRSEFlashWriteRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSEFlashWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSEFlashWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSEFlashWriteRequest(FILE* fp, const struct freespace_FRSEFlashWriteRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSEFlashWriteRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSEFlashWriteDataStr(char* dest, int maxlen, const struct freespace_FRSEFlashWriteData* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSEFlashWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#else
    n = snprintf(dest, maxlen, "FRSEFlashWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSEFlashWriteData(FILE* fp, const struct freespace_FRSEFlashWriteData* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSEFlashWriteDataStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDongleRFEnableMessageStr(char* dest, int maxlen, const struct freespace_DongleRFEnableMessage* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DongleRFEnableMessage()");
#else
    n = snprintf(dest, maxlen, "DongleRFEnableMessage()");
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDongleRFEnableMessage(FILE* fp, const struct freespace_DongleRFEnableMessage* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDongleRFEnableMessageStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDataModeRequestStr(char* dest, int maxlen, const struct freespace_DataModeRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DataModeRequest(enableBodyMotion=%d enableUserPosition=%d inhibitPowerManager=%d enableMouseMovement=%d disableFreespace=%d SDA=%d status=%d aggregate=%d)", s->enableBodyMotion, s->enableUserPosition, s->inhibitPowerManager, s->enableMouseMovement, s->disableFreespace, s->SDA, s->status, s->aggregate);
#else
    n = snprintf(dest, maxlen, "DataModeRequest(enableBodyMotion=%d enableUserPosition=%d inhibitPowerManager=%d enableMouseMovement=%d disableFreespace=%d SDA=%d status=%d aggregate=%d)", s->enableBodyMotion, s->enableUserPosition, s->inhibitPowerManager, s->enableMouseMovement, s->disableFreespace, s->SDA, s->status, s->aggregate);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDataModeRequest(FILE* fp, const struct freespace_DataModeRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDataModeRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printButtonTestModeRequestStr(char* dest, int maxlen, const struct freespace_ButtonTestModeRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "ButtonTestModeRequest(enable=%d)", s->enable);
#else
    n = snprintf(dest, maxlen, "ButtonTestModeRequest(enable=%d)", s->enable);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printButtonTestModeRequest(FILE* fp, const struct freespace_ButtonTestModeRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printButtonTestModeRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printPairingResponseStr(char* dest, int maxlen, const struct freespace_PairingResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "PairingResponse(pairing=%d autoPairing=%d success=%d)", s->pairing, s->autoPairing, s->success);
#else
    n = snprintf(dest, maxlen, "PairingResponse(pairing=%d autoPairing=%d success=%d)", s->pairing, s->autoPairing, s->success);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printPairingResponse(FILE* fp, const struct freespace_PairingResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printPairingResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printProductIDResponseStr(char* dest, int maxlen, const struct freespace_ProductIDResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "ProductIDResponse(swPartNumber=%d swBuildNumber=%d swicn=%d swVersionPatch=%d swVersionMinor=%d swVersionMajor=%d serialNumber=%d deviceClass=%d invalidNS=%d startup=%d)", s->swPartNumber, s->swBuildNumber, s->swicn, s->swVersionPatch, s->swVersionMinor, s->swVersionMajor, s->serialNumber, s->deviceClass, s->invalidNS, s->startup);
#else
    n = snprintf(dest, maxlen, "ProductIDResponse(swPartNumber=%d swBuildNumber=%d swicn=%d swVersionPatch=%d swVersionMinor=%d swVersionMajor=%d serialNumber=%d deviceClass=%d invalidNS=%d startup=%d)", s->swPartNumber, s->swBuildNumber, s->swicn, s->swVersionPatch, s->swVersionMinor, s->swVersionMajor, s->serialNumber, s->deviceClass, s->invalidNS, s->startup);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printProductIDResponse(FILE* fp, const struct freespace_ProductIDResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printProductIDResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printLinkStatusStr(char* dest, int maxlen, const struct freespace_LinkStatus* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "LinkStatus(status=%d mode=%d resetStatus=%d txDisabled=%d)", s->status, s->mode, s->resetStatus, s->txDisabled);
#else
    n = snprintf(dest, maxlen, "LinkStatus(status=%d mode=%d resetStatus=%d txDisabled=%d)", s->status, s->mode, s->resetStatus, s->txDisabled);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printLinkStatus(FILE* fp, const struct freespace_LinkStatus* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printLinkStatusStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printAlwaysOnResponseStr(char* dest, int maxlen, const struct freespace_AlwaysOnResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "AlwaysOnResponse()");
#else
    n = snprintf(dest, maxlen, "AlwaysOnResponse()");
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printAlwaysOnResponse(FILE* fp, const struct freespace_AlwaysOnResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printAlwaysOnResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSHandheldReadResponseStr(char* dest, int maxlen, const struct freespace_FRSHandheldReadResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSHandheldReadResponse(wordOffset=%d  status=%d dataLength=%d FRStype=%d)", s->wordOffset, s->status, s->dataLength, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSHandheldReadResponse(wordOffset=%d  status=%d dataLength=%d FRStype=%d)", s->wordOffset, s->status, s->dataLength, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSHandheldReadResponse(FILE* fp, const struct freespace_FRSHandheldReadResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSHandheldReadResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSHandheldWriteResponseStr(char* dest, int maxlen, const struct freespace_FRSHandheldWriteResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSHandheldWriteResponse(wordOffset=%d status=%d)", s->wordOffset, s->status);
#else
    n = snprintf(dest, maxlen, "FRSHandheldWriteResponse(wordOffset=%d status=%d)", s->wordOffset, s->status);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSHandheldWriteResponse(FILE* fp, const struct freespace_FRSHandheldWriteResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSHandheldWriteResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSDongleReadResponseStr(char* dest, int maxlen, const struct freespace_FRSDongleReadResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSDongleReadResponse(wordOffset=%d  status=%d dataLength=%d FRStype=%d)", s->wordOffset, s->status, s->dataLength, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSDongleReadResponse(wordOffset=%d  status=%d dataLength=%d FRStype=%d)", s->wordOffset, s->status, s->dataLength, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSDongleReadResponse(FILE* fp, const struct freespace_FRSDongleReadResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSDongleReadResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSDongleWriteResponseStr(char* dest, int maxlen, const struct freespace_FRSDongleWriteResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSDongleWriteResponse(wordOffset=%d status=%d)", s->wordOffset, s->status);
#else
    n = snprintf(dest, maxlen, "FRSDongleWriteResponse(wordOffset=%d status=%d)", s->wordOffset, s->status);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSDongleWriteResponse(FILE* fp, const struct freespace_FRSDongleWriteResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSDongleWriteResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSEFlashReadResponseStr(char* dest, int maxlen, const struct freespace_FRSEFlashReadResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSEFlashReadResponse(wordOffset=%d  status=%d dataLength=%d FRStype=%d)", s->wordOffset, s->status, s->dataLength, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSEFlashReadResponse(wordOffset=%d  status=%d dataLength=%d FRStype=%d)", s->wordOffset, s->status, s->dataLength, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSEFlashReadResponse(FILE* fp, const struct freespace_FRSEFlashReadResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSEFlashReadResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSEFlashWriteResponseStr(char* dest, int maxlen, const struct freespace_FRSEFlashWriteResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSEFlashWriteResponse(wordOffset=%d status=%d)", s->wordOffset, s->status);
#else
    n = snprintf(dest, maxlen, "FRSEFlashWriteResponse(wordOffset=%d status=%d)", s->wordOffset, s->status);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSEFlashWriteResponse(FILE* fp, const struct freespace_FRSEFlashWriteResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSEFlashWriteResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDataModeResponseStr(char* dest, int maxlen, const struct freespace_DataModeResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DataModeResponse(enableBodyMotion=%d enableUserPosition=%d inhibitPowerManager=%d enableMouseMovement=%d disableFreespace=%d SDA=%d aggregate=%d)", s->enableBodyMotion, s->enableUserPosition, s->inhibitPowerManager, s->enableMouseMovement, s->disableFreespace, s->SDA, s->aggregate);
#else
    n = snprintf(dest, maxlen, "DataModeResponse(enableBodyMotion=%d enableUserPosition=%d inhibitPowerManager=%d enableMouseMovement=%d disableFreespace=%d SDA=%d aggregate=%d)", s->enableBodyMotion, s->enableUserPosition, s->inhibitPowerManager, s->enableMouseMovement, s->disableFreespace, s->SDA, s->aggregate);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDataModeResponse(FILE* fp, const struct freespace_DataModeResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDataModeResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printButtonTestModeResponseStr(char* dest, int maxlen, const struct freespace_ButtonTestModeResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "ButtonTestModeResponse(status=%d button=%d press=%d)", s->status, s->button, s->press);
#else
    n = snprintf(dest, maxlen, "ButtonTestModeResponse(status=%d button=%d press=%d)", s->status, s->button, s->press);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printButtonTestModeResponse(FILE* fp, const struct freespace_ButtonTestModeResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printButtonTestModeResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printBatteryLevelRequestStr(char* dest, int maxlen, const struct freespace_BatteryLevelRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "BatteryLevelRequest()");
#else
    n = snprintf(dest, maxlen, "BatteryLevelRequest()");
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printBatteryLevelRequest(FILE* fp, const struct freespace_BatteryLevelRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printBatteryLevelRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printBatteryLevelStr(char* dest, int maxlen, const struct freespace_BatteryLevel* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "BatteryLevel(batteryStrength=%d)", s->batteryStrength);
#else
    n = snprintf(dest, maxlen, "BatteryLevel(batteryStrength=%d)", s->batteryStrength);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printBatteryLevel(FILE* fp, const struct freespace_BatteryLevel* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printBatteryLevelStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printBodyFrameStr(char* dest, int maxlen, const struct freespace_BodyFrame* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "BodyFrame(button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaX=%d deltaY=%d deltaWheel=%d sequenceNumber=%d linearAccelX=%d linearAccelY=%d linearAccelZ=%d angularVelX=%d angularVelY=%d angularVelZ=%d)", s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaX, s->deltaY, s->deltaWheel, s->sequenceNumber, s->linearAccelX, s->linearAccelY, s->linearAccelZ, s->angularVelX, s->angularVelY, s->angularVelZ);
#else
    n = snprintf(dest, maxlen, "BodyFrame(button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaX=%d deltaY=%d deltaWheel=%d sequenceNumber=%d linearAccelX=%d linearAccelY=%d linearAccelZ=%d angularVelX=%d angularVelY=%d angularVelZ=%d)", s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaX, s->deltaY, s->deltaWheel, s->sequenceNumber, s->linearAccelX, s->linearAccelY, s->linearAccelZ, s->angularVelX, s->angularVelY, s->angularVelZ);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printBodyFrame(FILE* fp, const struct freespace_BodyFrame* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printBodyFrameStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printUserFrameStr(char* dest, int maxlen, const struct freespace_UserFrame* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "UserFrame(button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaX=%d deltaY=%d deltaWheel=%d sequenceNumber=%d linearPosX=%d linearPosY=%d linearPosZ=%d angularPosA=%d angularPosB=%d angularPosC=%d angularPosD=%d)", s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaX, s->deltaY, s->deltaWheel, s->sequenceNumber, s->linearPosX, s->linearPosY, s->linearPosZ, s->angularPosA, s->angularPosB, s->angularPosC, s->angularPosD);
#else
    n = snprintf(dest, maxlen, "UserFrame(button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaX=%d deltaY=%d deltaWheel=%d sequenceNumber=%d linearPosX=%d linearPosY=%d linearPosZ=%d angularPosA=%d angularPosB=%d angularPosC=%d angularPosD=%d)", s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaX, s->deltaY, s->deltaWheel, s->sequenceNumber, s->linearPosX, s->linearPosY, s->linearPosZ, s->angularPosA, s->angularPosB, s->angularPosC, s->angularPosD);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printUserFrame(FILE* fp, const struct freespace_UserFrame* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printUserFrameStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDataMotionControlStr(char* dest, int maxlen, const struct freespace_DataMotionControl* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DataMotionControl(enableBodyMotion=%d enableUserPosition=%d inhibitPowerManager=%d enableMouseMovement=%d disableFreespace=%d)", s->enableBodyMotion, s->enableUserPosition, s->inhibitPowerManager, s->enableMouseMovement, s->disableFreespace);
#else
    n = snprintf(dest, maxlen, "DataMotionControl(enableBodyMotion=%d enableUserPosition=%d inhibitPowerManager=%d enableMouseMovement=%d disableFreespace=%d)", s->enableBodyMotion, s->enableUserPosition, s->inhibitPowerManager, s->enableMouseMovement, s->disableFreespace);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDataMotionControl(FILE* fp, const struct freespace_DataMotionControl* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDataMotionControlStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSWriteResponseStr(char* dest, int maxlen, const struct freespace_FRSWriteResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSWriteResponse(status=%d wordOffset=%d)", s->status, s->wordOffset);
#else
    n = snprintf(dest, maxlen, "FRSWriteResponse(status=%d wordOffset=%d)", s->status, s->wordOffset);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSWriteResponse(FILE* fp, const struct freespace_FRSWriteResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSWriteResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDataModeControlV2ResponseStr(char* dest, int maxlen, const struct freespace_DataModeControlV2Response* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DataModeControlV2Response(operatingStatus=%d mode=%d outputStatus=%d modeActual=%d packetSelect=%d formatSelect=%d ff0=%d ff1=%d ff2=%d ff3=%d ff4=%d ff5=%d ff6=%d ff7=%d)", s->operatingStatus, s->mode, s->outputStatus, s->modeActual, s->packetSelect, s->formatSelect, s->ff0, s->ff1, s->ff2, s->ff3, s->ff4, s->ff5, s->ff6, s->ff7);
#else
    n = snprintf(dest, maxlen, "DataModeControlV2Response(operatingStatus=%d mode=%d outputStatus=%d modeActual=%d packetSelect=%d formatSelect=%d ff0=%d ff1=%d ff2=%d ff3=%d ff4=%d ff5=%d ff6=%d ff7=%d)", s->operatingStatus, s->mode, s->outputStatus, s->modeActual, s->packetSelect, s->formatSelect, s->ff0, s->ff1, s->ff2, s->ff3, s->ff4, s->ff5, s->ff6, s->ff7);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDataModeControlV2Response(FILE* fp, const struct freespace_DataModeControlV2Response* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDataModeControlV2ResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printSensorPeriodResponseStr(char* dest, int maxlen, const struct freespace_SensorPeriodResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "SensorPeriodResponse(sensor=%d period=%d)", s->sensor, s->period);
#else
    n = snprintf(dest, maxlen, "SensorPeriodResponse(sensor=%d period=%d)", s->sensor, s->period);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printSensorPeriodResponse(FILE* fp, const struct freespace_SensorPeriodResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printSensorPeriodResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSReadResponseStr(char* dest, int maxlen, const struct freespace_FRSReadResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSReadResponse(status=%d dataLength=%d wordOffset=%d  FRStype=%d)", s->status, s->dataLength, s->wordOffset, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSReadResponse(status=%d dataLength=%d wordOffset=%d  FRStype=%d)", s->status, s->dataLength, s->wordOffset, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSReadResponse(FILE* fp, const struct freespace_FRSReadResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSReadResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printPerResponseStr(char* dest, int maxlen, const struct freespace_PerResponse* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "PerResponse(count=%d msError=%d smError=%d frError=%d)", s->count, s->msError, s->smError, s->frError);
#else
    n = snprintf(dest, maxlen, "PerResponse(count=%d msError=%d smError=%d frError=%d)", s->count, s->msError, s->smError, s->frError);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printPerResponse(FILE* fp, const struct freespace_PerResponse* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printPerResponseStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSWriteRequestStr(char* dest, int maxlen, const struct freespace_FRSWriteRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSWriteRequest(length=%d FRStype=%d)", s->length, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSWriteRequest(FILE* fp, const struct freespace_FRSWriteRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSWriteRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSWriteDataStr(char* dest, int maxlen, const struct freespace_FRSWriteData* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#else
    n = snprintf(dest, maxlen, "FRSWriteData(wordOffset=%d data=%d)", s->wordOffset, s->data);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSWriteData(FILE* fp, const struct freespace_FRSWriteData* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSWriteDataStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSReadRequestStr(char* dest, int maxlen, const struct freespace_FRSReadRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSReadRequest(readOffset=%d FRStype=%d BlockSize=%d)", s->readOffset, s->FRStype, s->BlockSize);
#else
    n = snprintf(dest, maxlen, "FRSReadRequest(readOffset=%d FRStype=%d BlockSize=%d)", s->readOffset, s->FRStype, s->BlockSize);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSReadRequest(FILE* fp, const struct freespace_FRSReadRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSReadRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printPerRequestStr(char* dest, int maxlen, const struct freespace_PerRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "PerRequest(op=%d )", s->op);
#else
    n = snprintf(dest, maxlen, "PerRequest(op=%d )", s->op);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printPerRequest(FILE* fp, const struct freespace_PerRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printPerRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printActivityClassificationNotificationStr(char* dest, int maxlen, const struct freespace_ActivityClassificationNotification* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "ActivityClassificationNotification(classification=%d)", s->classification);
#else
    n = snprintf(dest, maxlen, "ActivityClassificationNotification(classification=%d)", s->classification);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printActivityClassificationNotification(FILE* fp, const struct freespace_ActivityClassificationNotification* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printActivityClassificationNotificationStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDataModeControlV2RequestStr(char* dest, int maxlen, const struct freespace_DataModeControlV2Request* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DataModeControlV2Request(operatingStatus=%d mode=%d outputStatus=%d packetSelect=%d formatSelect=%d ff0=%d ff1=%d ff2=%d ff3=%d ff4=%d ff5=%d ff6=%d ff7=%d)", s->operatingStatus, s->mode, s->outputStatus, s->packetSelect, s->formatSelect, s->ff0, s->ff1, s->ff2, s->ff3, s->ff4, s->ff5, s->ff6, s->ff7);
#else
    n = snprintf(dest, maxlen, "DataModeControlV2Request(operatingStatus=%d mode=%d outputStatus=%d packetSelect=%d formatSelect=%d ff0=%d ff1=%d ff2=%d ff3=%d ff4=%d ff5=%d ff6=%d ff7=%d)", s->operatingStatus, s->mode, s->outputStatus, s->packetSelect, s->formatSelect, s->ff0, s->ff1, s->ff2, s->ff3, s->ff4, s->ff5, s->ff6, s->ff7);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDataModeControlV2Request(FILE* fp, const struct freespace_DataModeControlV2Request* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDataModeControlV2RequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printReorientationRequestStr(char* dest, int maxlen, const struct freespace_ReorientationRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "ReorientationRequest(select=%d commit=%d quaternionParameter1=%d quaternionParameter2=%d)", s->select, s->commit, s->quaternionParameter1, s->quaternionParameter2);
#else
    n = snprintf(dest, maxlen, "ReorientationRequest(select=%d commit=%d quaternionParameter1=%d quaternionParameter2=%d)", s->select, s->commit, s->quaternionParameter1, s->quaternionParameter2);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printReorientationRequest(FILE* fp, const struct freespace_ReorientationRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printReorientationRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printSensorPeriodRequestStr(char* dest, int maxlen, const struct freespace_SensorPeriodRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "SensorPeriodRequest(commit=%d get=%d sensor=%d period=%d)", s->commit, s->get, s->sensor, s->period);
#else
    n = snprintf(dest, maxlen, "SensorPeriodRequest(commit=%d get=%d sensor=%d period=%d)", s->commit, s->get, s->sensor, s->period);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printSensorPeriodRequest(FILE* fp, const struct freespace_SensorPeriodRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printSensorPeriodRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printBmsRequestStr(char* dest, int maxlen, const struct freespace_BmsRequest* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "BmsRequest(bmsRequest=%d)", s->bmsRequest);
#else
    n = snprintf(dest, maxlen, "BmsRequest(bmsRequest=%d)", s->bmsRequest);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printBmsRequest(FILE* fp, const struct freespace_BmsRequest* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printBmsRequestStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printProductIDResponseBLEStr(char* dest, int maxlen, const struct freespace_ProductIDResponseBLE* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "ProductIDResponseBLE(deviceClass=%d startup=%d invalidNS=%d swVersionMajor=%d swVersionMinor=%d swPartNumber=%d serialNumber=%d swBuildNumber=%d swVersionPatch=%d)", s->deviceClass, s->startup, s->invalidNS, s->swVersionMajor, s->swVersionMinor, s->swPartNumber, s->serialNumber, s->swBuildNumber, s->swVersionPatch);
#else
    n = snprintf(dest, maxlen, "ProductIDResponseBLE(deviceClass=%d startup=%d invalidNS=%d swVersionMajor=%d swVersionMinor=%d swPartNumber=%d serialNumber=%d swBuildNumber=%d swVersionPatch=%d)", s->deviceClass, s->startup, s->invalidNS, s->swVersionMajor, s->swVersionMinor, s->swPartNumber, s->serialNumber, s->swBuildNumber, s->swVersionPatch);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printProductIDResponseBLE(FILE* fp, const struct freespace_ProductIDResponseBLE* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printProductIDResponseBLEStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printFRSReadResponseBLEStr(char* dest, int maxlen, const struct freespace_FRSReadResponseBLE* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "FRSReadResponseBLE(status=%d dataLength=%d wordOffset=%d  reserved=%d FRStype=%d)", s->status, s->dataLength, s->wordOffset, s->reserved, s->FRStype);
#else
    n = snprintf(dest, maxlen, "FRSReadResponseBLE(status=%d dataLength=%d wordOffset=%d  reserved=%d FRStype=%d)", s->status, s->dataLength, s->wordOffset, s->reserved, s->FRStype);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printFRSReadResponseBLE(FILE* fp, const struct freespace_FRSReadResponseBLE* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printFRSReadResponseBLEStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printBodyUserFrameStr(char* dest, int maxlen, const struct freespace_BodyUserFrame* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "BodyUserFrame(button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaX=%d deltaY=%d deltaWheel=%d sequenceNumber=%d linearAccelX=%d linearAccelY=%d linearAccelZ=%d angularVelX=%d angularVelY=%d angularVelZ=%d linearPosX=%d linearPosY=%d linearPosZ=%d angularPosB=%d angularPosC=%d angularPosD=%d angularPosA=%d)", s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaX, s->deltaY, s->deltaWheel, s->sequenceNumber, s->linearAccelX, s->linearAccelY, s->linearAccelZ, s->angularVelX, s->angularVelY, s->angularVelZ, s->linearPosX, s->linearPosY, s->linearPosZ, s->angularPosB, s->angularPosC, s->angularPosD, s->angularPosA);
#else
    n = snprintf(dest, maxlen, "BodyUserFrame(button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaX=%d deltaY=%d deltaWheel=%d sequenceNumber=%d linearAccelX=%d linearAccelY=%d linearAccelZ=%d angularVelX=%d angularVelY=%d angularVelZ=%d linearPosX=%d linearPosY=%d linearPosZ=%d angularPosB=%d angularPosC=%d angularPosD=%d angularPosA=%d)", s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaX, s->deltaY, s->deltaWheel, s->sequenceNumber, s->linearAccelX, s->linearAccelY, s->linearAccelZ, s->angularVelX, s->angularVelY, s->angularVelZ, s->linearPosX, s->linearPosY, s->linearPosZ, s->angularPosB, s->angularPosC, s->angularPosD, s->angularPosA);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printBodyUserFrame(FILE* fp, const struct freespace_BodyUserFrame* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printBodyUserFrameStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printMotionEngineOutputStr(char* dest, int maxlen, const struct freespace_MotionEngineOutput* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "MotionEngineOutput(formatSelect=%d ff0=%d ff1=%d ff2=%d ff3=%d ff4=%d ff5=%d ff6=%d ff7=%d sequenceNumber=%d )", s->formatSelect, s->ff0, s->ff1, s->ff2, s->ff3, s->ff4, s->ff5, s->ff6, s->ff7, s->sequenceNumber);
#else
    n = snprintf(dest, maxlen, "MotionEngineOutput(formatSelect=%d ff0=%d ff1=%d ff2=%d ff3=%d ff4=%d ff5=%d ff6=%d ff7=%d sequenceNumber=%d )", s->formatSelect, s->ff0, s->ff1, s->ff2, s->ff3, s->ff4, s->ff5, s->ff6, s->ff7, s->sequenceNumber);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printMotionEngineOutput(FILE* fp, const struct freespace_MotionEngineOutput* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printMotionEngineOutputStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDceOutV2Str(char* dest, int maxlen, const struct freespace_DceOutV2* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DceOutV2(sampleBase=%d ax=%d ay=%d az=%d rx=%d ry=%d rz=%d mx=%d my=%d mz=%d temperature=%d flags=%d button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaWheel=%d)", s->sampleBase, s->ax, s->ay, s->az, s->rx, s->ry, s->rz, s->mx, s->my, s->mz, s->temperature, s->flags, s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaWheel);
#else
    n = snprintf(dest, maxlen, "DceOutV2(sampleBase=%d ax=%d ay=%d az=%d rx=%d ry=%d rz=%d mx=%d my=%d mz=%d temperature=%d flags=%d button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaWheel=%d)", s->sampleBase, s->ax, s->ay, s->az, s->rx, s->ry, s->rz, s->mx, s->my, s->mz, s->temperature, s->flags, s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaWheel);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDceOutV2(FILE* fp, const struct freespace_DceOutV2* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDceOutV2Str(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDceOutV3Str(char* dest, int maxlen, const struct freespace_DceOutV3* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DceOutV3(sampleBase=%d button1=%d button2=%d button3=%d button4=%d ff1=%d ff2=%d ff3=%d ff4=%d ax=%d ay=%d az=%d rx=%d ry=%d rz=%d temperature=%d)", s->sampleBase, s->button1, s->button2, s->button3, s->button4, s->ff1, s->ff2, s->ff3, s->ff4, s->ax, s->ay, s->az, s->rx, s->ry, s->rz, s->temperature);
#else
    n = snprintf(dest, maxlen, "DceOutV3(sampleBase=%d button1=%d button2=%d button3=%d button4=%d ff1=%d ff2=%d ff3=%d ff4=%d ax=%d ay=%d az=%d rx=%d ry=%d rz=%d temperature=%d)", s->sampleBase, s->button1, s->button2, s->button3, s->button4, s->ff1, s->ff2, s->ff3, s->ff4, s->ax, s->ay, s->az, s->rx, s->ry, s->rz, s->temperature);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDceOutV3(FILE* fp, const struct freespace_DceOutV3* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDceOutV3Str(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDceOutV4T0Str(char* dest, int maxlen, const struct freespace_DceOutV4T0* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DceOutV4T0(sampleBase=%d ax=%d ay=%d az=%d rx=%d ry=%d rz=%d temperature=%d)", s->sampleBase, s->ax, s->ay, s->az, s->rx, s->ry, s->rz, s->temperature);
#else
    n = snprintf(dest, maxlen, "DceOutV4T0(sampleBase=%d ax=%d ay=%d az=%d rx=%d ry=%d rz=%d temperature=%d)", s->sampleBase, s->ax, s->ay, s->az, s->rx, s->ry, s->rz, s->temperature);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDceOutV4T0(FILE* fp, const struct freespace_DceOutV4T0* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDceOutV4T0Str(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}


LIBFREESPACE_API int freespace_printDceOutV4T1Str(char* dest, int maxlen, const struct freespace_DceOutV4T1* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "DceOutV4T1(sampleBase=%d mx=%d my=%d mz=%d flag1=%d flag2=%d flag3=%d flag4=%d ff1=%d ff2=%d ff3=%d ff4=%d button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaWheel=%d)", s->sampleBase, s->mx, s->my, s->mz, s->flag1, s->flag2, s->flag3, s->flag4, s->ff1, s->ff2, s->ff3, s->ff4, s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaWheel);
#else
    n = snprintf(dest, maxlen, "DceOutV4T1(sampleBase=%d mx=%d my=%d mz=%d flag1=%d flag2=%d flag3=%d flag4=%d ff1=%d ff2=%d ff3=%d ff4=%d button1=%d button2=%d button3=%d button4=%d button5=%d button6=%d button7=%d button8=%d deltaWheel=%d)", s->sampleBase, s->mx, s->my, s->mz, s->flag1, s->flag2, s->flag3, s->flag4, s->ff1, s->ff2, s->ff3, s->ff4, s->button1, s->button2, s->button3, s->button4, s->button5, s->button6, s->button7, s->button8, s->deltaWheel);
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}

LIBFREESPACE_API int freespace_printDceOutV4T1(FILE* fp, const struct freespace_DceOutV4T1* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_printDceOutV4T1Str(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%s\n", str);
}

