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

#include "freespace/freespace_codecs.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifdef _WIN32
#define STRICT_DECODE_LENGTH 0
#else
#define STRICT_DECODE_LENGTH 0
#endif

#undef CODECS_PRINTF
//#define CODECS_DEBUG
#ifdef CODECS_DEBUG
#define CODECS_PRINTF printf
#else
#define CODECS_PRINTF(...)
#endif


static uint32_t toUint32(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return ((((uint32_t) a[3])) << 24) | ((((uint32_t) a[2])) << 16) | ((((uint32_t) a[1])) << 8) | (uint32_t) a[0];
#else
    return ((((uint32_t) a[0])) << 24) | ((((uint32_t) a[1])) << 16) | ((((uint32_t) a[2])) << 8) | (uint32_t) a[3];
#endif
}

static uint16_t toUint16(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return ((((uint16_t) a[1])) << 8) | (uint16_t) a[0];
#else
    return ((((uint16_t) a[0])) << 8) | (uint16_t) a[1];
#endif
}

static uint8_t toUint8(const uint8_t * a) {
    return (uint8_t) *a;
}

static int32_t toInt32(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return (int32_t) (((((int32_t) a[3])) << 24) | ((((uint32_t) a[2])) << 16) | ((((uint32_t) a[1])) << 8) | (uint32_t) a[0]);
#else
    return (int32_t) (((((int32_t) a[0])) << 24) | ((((uint32_t) a[1])) << 16) | ((((uint32_t) a[2])) << 8) | (uint32_t) a[3]);
#endif
}

static int16_t toInt16(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return (((int16_t) a[1]) << 8) | a[0];
#else
    return (((int16_t) a[0]) << 8) | a[1];
#endif
}

static int8_t toInt8(const uint8_t * a) {
    return (int8_t) *a;
}

static uint8_t getBit(uint8_t a, uint16_t whichBit) {
    return (uint8_t) (a >> whichBit) & 0x01;
}

static uint8_t getNibble(uint8_t a, uint16_t whichNibble) {
    return (uint8_t) (a >> (whichNibble*4)) & 0x0F;
}

static uint8_t byteFromNibbles(uint8_t lsn, uint8_t msn) {
    return lsn | (msn << 4);
}


LIBFREESPACE_API int freespace_encodePairingMessage(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_PairingMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 13;
			return 7 + offset;
		case 2:
			if (maxlength < 5) {
				CODECS_PRINTF("freespace_PairingMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 2;
			message[1] = 1 + offset;
			return 1 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeProductIDRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_ProductIDRequest* s = &(m->productIDRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_ProductIDRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 32;
			return 7 + offset;
		case 2:
			if (maxlength < 11) {
				CODECS_PRINTF("freespace_ProductIDRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 9;
			message[1 + offset] = s->Format >> 0;
			message[1] = 7 + offset;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeLEDSetRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_LEDSetRequest* s = &(m->lEDSetRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_LEDSetRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 34;
			message[1 + offset] = s->onOff >> 0;
			message[2 + offset] = s->selectLED >> 0;
			return 7 + offset;
		case 2:
			if (maxlength < 11) {
				CODECS_PRINTF("freespace_LEDSetRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 10;
			message[1 + offset] = s->onOff >> 0;
			message[2 + offset] = s->selectLED >> 0;
			message[1] = 7 + offset;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeLinkQualityRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_LinkQualityRequest* s = &(m->linkQualityRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_LinkQualityRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 48;
			message[1 + offset] = s->enable >> 0;
			return 7 + offset;
		case 2:
			if (maxlength < 11) {
				CODECS_PRINTF("freespace_LinkQualityRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 3;
			message[1 + offset] = s->enable >> 0;
			message[1] = 7 + offset;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeAlwaysOnRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_AlwaysOnRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 49;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFrequencyFixRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FrequencyFixRequest* s = &(m->frequencyFixRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FrequencyFixRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 50;
			message[1 + offset] = s->channel0 >> 0;
			message[2 + offset] = s->channel1 >> 0;
			message[3 + offset] = s->channel2 >> 0;
			message[4 + offset] = s->channel3 >> 0;
			message[5 + offset] = s->channel4 >> 0;
			message[6 + offset] = s->device >> 0;
			return 7 + offset;
		case 2:
			if (maxlength < 10) {
				CODECS_PRINTF("freespace_FrequencyFixRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 11;
			message[1 + offset] = s->channel0 >> 0;
			message[2 + offset] = s->channel1 >> 0;
			message[3 + offset] = s->channel2 >> 0;
			message[4 + offset] = s->channel3 >> 0;
			message[5 + offset] = s->channel4 >> 0;
			message[1] = 6 + offset;
			return 6 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeSoftwareResetMessage(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_SoftwareResetMessage* s = &(m->softwareResetMessage);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_SoftwareResetMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 51;
			message[1 + offset] = s->device >> 0;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeDongleRFDisableMessage(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_DongleRFDisableMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 52;
			return 7 + offset;
		case 2:
			if (maxlength < 5) {
				CODECS_PRINTF("freespace_DongleRFDisableMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 13;
			message[1] = 1 + offset;
			return 1 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeTxDisableMessage(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	switch(m->ver) {
		case 1:
			if (maxlength < 2) {
				CODECS_PRINTF("freespace_TxDisableMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 80;
			return 1 + offset;
		case 2:
			if (maxlength < 5) {
				CODECS_PRINTF("freespace_TxDisableMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 14;
			message[1] = 1 + offset;
			return 1 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeDongleRFSupressHomeFrequencyMessage(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_DongleRFSupressHomeFrequencyMessage* s = &(m->dongleRFSupressHomeFrequencyMessage);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_DongleRFSupressHomeFrequencyMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 53;
			message[1 + offset] = s->low >> 0;
			message[2 + offset] = s->high >> 0;
			return 7 + offset;
		case 2:
			if (maxlength < 11) {
				CODECS_PRINTF("freespace_DongleRFSupressHomeFrequencyMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 15;
			message[1 + offset] = s->low >> 0;
			message[2 + offset] = s->high >> 0;
			message[1] = 7 + offset;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSHandheldReadRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSHandheldReadRequest* s = &(m->fRSHandheldReadRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSHandheldReadRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 58;
			message[1 + offset] = s->wordOffset >> 0;
			message[2 + offset] = s->wordOffset >> 8;
			message[3 + offset] = s->FRStype >> 0;
			message[4 + offset] = s->FRStype >> 8;
			message[5 + offset] = s->BlockSize >> 0;
			message[6 + offset] = s->BlockSize >> 8;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSHandheldWriteRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSHandheldWriteRequest* s = &(m->fRSHandheldWriteRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSHandheldWriteRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 61;
			message[1 + offset] = s->length >> 0;
			message[2 + offset] = s->length >> 8;
			message[3 + offset] = s->FRStype >> 0;
			message[4 + offset] = s->FRStype >> 8;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSHandheldWriteData(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSHandheldWriteData* s = &(m->fRSHandheldWriteData);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSHandheldWriteData encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 63;
			message[1 + offset] = s->wordOffset >> 0;
			message[2 + offset] = s->wordOffset >> 8;
			message[3 + offset] = s->data >> 0;
			message[4 + offset] = s->data >> 8;
			message[5 + offset] = s->data >> 16;
			message[6 + offset] = s->data >> 24;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSDongleReadRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSDongleReadRequest* s = &(m->fRSDongleReadRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSDongleReadRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 59;
			message[1 + offset] = s->wordOffset >> 0;
			message[2 + offset] = s->wordOffset >> 8;
			message[3 + offset] = s->FRStype >> 0;
			message[4 + offset] = s->FRStype >> 8;
			message[5 + offset] = s->BlockSize >> 0;
			message[6 + offset] = s->BlockSize >> 8;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSDongleWriteRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSDongleWriteRequest* s = &(m->fRSDongleWriteRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSDongleWriteRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 62;
			message[1 + offset] = s->length >> 0;
			message[2 + offset] = s->length >> 8;
			message[3 + offset] = s->FRStype >> 0;
			message[4 + offset] = s->FRStype >> 8;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSDongleWriteData(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSDongleWriteData* s = &(m->fRSDongleWriteData);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSDongleWriteData encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 64;
			message[1 + offset] = s->wordOffset >> 0;
			message[2 + offset] = s->wordOffset >> 8;
			message[3 + offset] = s->data >> 0;
			message[4 + offset] = s->data >> 8;
			message[5 + offset] = s->data >> 16;
			message[6 + offset] = s->data >> 24;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSEFlashReadRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSEFlashReadRequest* s = &(m->fRSEFlashReadRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSEFlashReadRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 65;
			message[1 + offset] = s->wordOffset >> 0;
			message[2 + offset] = s->wordOffset >> 8;
			message[3 + offset] = s->FRStype >> 0;
			message[4 + offset] = s->FRStype >> 8;
			message[5 + offset] = s->BlockSize >> 0;
			message[6 + offset] = s->BlockSize >> 8;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSEFlashWriteRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSEFlashWriteRequest* s = &(m->fRSEFlashWriteRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSEFlashWriteRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 66;
			message[1 + offset] = s->length >> 0;
			message[2 + offset] = s->length >> 8;
			message[3 + offset] = s->FRStype >> 0;
			message[4 + offset] = s->FRStype >> 8;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSEFlashWriteData(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSEFlashWriteData* s = &(m->fRSEFlashWriteData);

	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_FRSEFlashWriteData encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 67;
			message[1 + offset] = s->wordOffset >> 0;
			message[2 + offset] = s->wordOffset >> 8;
			message[3 + offset] = s->data >> 0;
			message[4 + offset] = s->data >> 8;
			message[5 + offset] = s->data >> 16;
			message[6 + offset] = s->data >> 24;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeDongleRFEnableMessage(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	switch(m->ver) {
		case 1:
			if (maxlength < 8) {
				CODECS_PRINTF("freespace_DongleRFEnableMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 71;
			return 7 + offset;
		case 2:
			if (maxlength < 5) {
				CODECS_PRINTF("freespace_DongleRFEnableMessage encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 12;
			message[1] = 1 + offset;
			return 1 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeDataModeRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_DataModeRequest* s = &(m->dataModeRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 3) {
				CODECS_PRINTF("freespace_DataModeRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 73;
			message[1 + offset] = (((s->enableBodyMotion & 0x1) << 0)
								|  ((s->enableUserPosition & 0x1) << 1)
								|  ((s->inhibitPowerManager & 0x1) << 2)
								|  ((s->enableMouseMovement & 0x1) << 3)
								|  ((s->disableFreespace & 0x1) << 4)
								|  ((s->SDA & 0x1) << 5)
								|  ((s->status & 0x1) << 7));
			return 2 + offset;
		case 2:
			if (maxlength < 6) {
				CODECS_PRINTF("freespace_DataModeRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 4;
			message[1 + offset] = (((s->enableBodyMotion & 0x1) << 0)
								|  ((s->enableUserPosition & 0x1) << 1)
								|  ((s->inhibitPowerManager & 0x1) << 2)
								|  ((s->enableMouseMovement & 0x1) << 3)
								|  ((s->disableFreespace & 0x1) << 4)
								|  ((s->SDA & 0x1) << 5)
								|  ((s->aggregate & 0x1) << 6)
								|  ((s->status & 0x1) << 7));
			message[1] = 2 + offset;
			return 2 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeButtonTestModeRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_ButtonTestModeRequest* s = &(m->buttonTestModeRequest);

	switch(m->ver) {
		case 1:
			if (maxlength < 3) {
				CODECS_PRINTF("freespace_ButtonTestModeRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[0 + offset] = (uint8_t) 81;
			message[1 + offset] = s->enable >> 0;
			return 2 + offset;
		case 2:
			if (maxlength < 6) {
				CODECS_PRINTF("freespace_ButtonTestModeRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 17;
			message[1 + offset] = s->enable >> 0;
			message[1] = 2 + offset;
			return 2 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_decodePairingResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_PairingResponse* s = &(m->pairingResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "PairingResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 13) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->pairing = (uint8_t) ((message[1 + offset] >> 0) & 0x01);
			s->autoPairing = (uint8_t) ((message[1 + offset] >> 1) & 0x01);
			s->success = (uint8_t) ((message[1 + offset] >> 2) & 0x01);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 12) || (!STRICT_DECODE_LENGTH && length < 12)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "PairingResponse", 12, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 2) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->pairing = (uint8_t) ((message[1 + offset] >> 0) & 0x01);
			s->autoPairing = (uint8_t) ((message[1 + offset] >> 1) & 0x01);
			s->success = (uint8_t) ((message[1 + offset] >> 2) & 0x01);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeProductIDResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_ProductIDResponse* s = &(m->productIDResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "ProductIDResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 32) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->swPartNumber = toUint32(&message[1 + offset]);
			s->swBuildNumber = toUint32(&message[5 + offset]);
			s->swicn = toUint32(&message[9 + offset]);
			s->swVersionPatch = toUint16(&message[13 + offset]);
			s->swVersionMinor = toUint8(&message[15 + offset]);
			s->swVersionMajor = toUint8(&message[16 + offset]);
			s->serialNumber = toUint32(&message[19 + offset]);
			s->deviceClass = (uint8_t) ((message[23 + offset] >> 0) & 0x7F);
			s->invalidNS = getBit(message[23 + offset], 7);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 22) || (!STRICT_DECODE_LENGTH && length < 22)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "ProductIDResponse", 22, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 6) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 9) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->deviceClass = (uint8_t) ((message[1 + offset] >> 0) & 0x3F);
			s->startup = getBit(message[1 + offset], 7);
			s->invalidNS = getBit(message[1 + offset], 8);
			s->swVersionMajor = toUint8(&message[2 + offset]);
			s->swVersionMinor = toUint8(&message[3 + offset]);
			s->swPartNumber = toUint32(&message[4 + offset]);
			s->swBuildNumber = toUint32(&message[8 + offset]);
			s->serialNumber = toUint32(&message[12 + offset]);
			s->swVersionPatch = toUint16(&message[16 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeLinkStatus(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_LinkStatus* s = &(m->linkStatus);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "LinkStatus", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 48) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->status = toUint8(&message[1 + offset]);
			s->mode = toUint8(&message[2 + offset]);
			s->resetStatus = toUint8(&message[3 + offset]);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 31) || (!STRICT_DECODE_LENGTH && length < 31)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "LinkStatus", 31, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 3) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->status = toUint8(&message[1 + offset]);
			s->mode = toUint8(&message[2 + offset]);
			s->resetStatus = toUint8(&message[3 + offset]);
			s->txDisabled = toUint8(&message[4 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeAlwaysOnResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "AlwaysOnResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 49) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSHandheldReadResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSHandheldReadResponse* s = &(m->fRSHandheldReadResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSHandheldReadResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 58) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->wordOffset = toUint16(&message[1 + offset]);
			s->data[0] = toUint32(&message[3 + offset]);
			s->data[1] = toUint32(&message[7 + offset]);
			s->data[2] = toUint32(&message[11 + offset]);
			s->data[3] = toUint32(&message[15 + offset]);
			s->data[4] = toUint32(&message[19 + offset]);
    s->status = getNibble(message[23 + offset], 0);
    s->dataLength = getNibble(message[23 + offset], 1);
			s->FRStype = toUint16(&message[24 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSHandheldWriteResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSHandheldWriteResponse* s = &(m->fRSHandheldWriteResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSHandheldWriteResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 61) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->wordOffset = toUint16(&message[1 + offset]);
			s->status = toUint8(&message[3 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSDongleReadResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSDongleReadResponse* s = &(m->fRSDongleReadResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSDongleReadResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 59) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->wordOffset = toUint16(&message[1 + offset]);
			s->data[0] = toUint32(&message[3 + offset]);
			s->data[1] = toUint32(&message[7 + offset]);
			s->data[2] = toUint32(&message[11 + offset]);
			s->data[3] = toUint32(&message[15 + offset]);
			s->data[4] = toUint32(&message[19 + offset]);
    s->status = getNibble(message[23 + offset], 0);
    s->dataLength = getNibble(message[23 + offset], 1);
			s->FRStype = toUint16(&message[24 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSDongleWriteResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSDongleWriteResponse* s = &(m->fRSDongleWriteResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSDongleWriteResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 62) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->wordOffset = toUint16(&message[1 + offset]);
			s->status = toUint8(&message[3 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSEFlashReadResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSEFlashReadResponse* s = &(m->fRSEFlashReadResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSEFlashReadResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 65) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->wordOffset = toUint16(&message[1 + offset]);
			s->data[0] = toUint32(&message[3 + offset]);
			s->data[1] = toUint32(&message[7 + offset]);
			s->data[2] = toUint32(&message[11 + offset]);
			s->data[3] = toUint32(&message[15 + offset]);
			s->data[4] = toUint32(&message[19 + offset]);
    s->status = getNibble(message[23 + offset], 0);
    s->dataLength = getNibble(message[23 + offset], 1);
			s->FRStype = toUint16(&message[24 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSEFlashWriteResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSEFlashWriteResponse* s = &(m->fRSEFlashWriteResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 27) || (!STRICT_DECODE_LENGTH && length < 27)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSEFlashWriteResponse", 27, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 66) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->wordOffset = toUint16(&message[1 + offset]);
			s->status = toUint8(&message[3 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeDataModeResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_DataModeResponse* s = &(m->dataModeResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 3) || (!STRICT_DECODE_LENGTH && length < 3)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "DataModeResponse", 3, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 73) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->enableBodyMotion = getBit(message[1 + offset], 0);
			s->enableUserPosition = getBit(message[1 + offset], 1);
			s->inhibitPowerManager = getBit(message[1 + offset], 2);
			s->enableMouseMovement = getBit(message[1 + offset], 3);
			s->disableFreespace = getBit(message[1 + offset], 4);
			s->SDA = getBit(message[1 + offset], 5);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 6) || (!STRICT_DECODE_LENGTH && length < 6)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "DataModeResponse", 6, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 4) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->enableBodyMotion = getBit(message[1 + offset], 0);
			s->enableUserPosition = getBit(message[1 + offset], 1);
			s->inhibitPowerManager = getBit(message[1 + offset], 2);
			s->enableMouseMovement = getBit(message[1 + offset], 3);
			s->disableFreespace = getBit(message[1 + offset], 4);
			s->SDA = getBit(message[1 + offset], 5);
			s->aggregate = getBit(message[1 + offset], 6);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeButtonTestModeResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_ButtonTestModeResponse* s = &(m->buttonTestModeResponse);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 5) || (!STRICT_DECODE_LENGTH && length < 5)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "ButtonTestModeResponse", 5, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

            if ((uint8_t) message[offset] != 81) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->status = toUint8(&message[1 + offset]);
			s->button = toUint8(&message[2 + offset]);
			s->press = toUint8(&message[3 + offset]);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 8) || (!STRICT_DECODE_LENGTH && length < 8)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "ButtonTestModeResponse", 8, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 17) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->status = toUint8(&message[1 + offset]);
			s->button = toUint8(&message[2 + offset]);
			s->press = toUint8(&message[3 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_encodeBatteryLevelRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	switch(m->ver) {
		case 1:
			if (maxlength < 2) {
				CODECS_PRINTF("freespace_BatteryLevelRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 9;
			return 1 + offset;
		case 2:
			if (maxlength < 5) {
				CODECS_PRINTF("freespace_BatteryLevelRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 5;
			message[1] = 1 + offset;
			return 1 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_decodeBatteryLevel(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_BatteryLevel* s = &(m->batteryLevel);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 4) || (!STRICT_DECODE_LENGTH && length < 4)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "BatteryLevel", 4, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 10) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->batteryStrength = toUint8(&message[0 + offset]);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 6) || (!STRICT_DECODE_LENGTH && length < 6)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "BatteryLevel", 6, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->batteryStrength = toUint8(&message[1 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeBodyFrame(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_BodyFrame* s = &(m->bodyFrame);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 21) || (!STRICT_DECODE_LENGTH && length < 21)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "BodyFrame", 21, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 32) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->button1 = getBit(message[0 + offset], 0);
			s->button2 = getBit(message[0 + offset], 1);
			s->button3 = getBit(message[0 + offset], 2);
			s->button4 = getBit(message[0 + offset], 3);
			s->button5 = getBit(message[0 + offset], 4);
			s->button6 = getBit(message[0 + offset], 5);
			s->button7 = getBit(message[0 + offset], 6);
			s->button8 = getBit(message[0 + offset], 7);
			s->deltaX = toInt8(&message[1 + offset]);
			s->deltaY = toInt8(&message[2 + offset]);
			s->deltaWheel = toInt8(&message[3 + offset]);
			s->sequenceNumber = toUint16(&message[4 + offset]);
			s->linearAccelX = toInt16(&message[8 + offset]);
			s->linearAccelY = toInt16(&message[10 + offset]);
			s->linearAccelZ = toInt16(&message[12 + offset]);
			s->angularVelX = toInt16(&message[14 + offset]);
			s->angularVelY = toInt16(&message[16 + offset]);
			s->angularVelZ = toInt16(&message[18 + offset]);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 22) || (!STRICT_DECODE_LENGTH && length < 22)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "BodyFrame", 22, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 32) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];
			s->button1 = getBit(message[0 + offset], 0);
			s->button2 = getBit(message[0 + offset], 1);
			s->button3 = getBit(message[0 + offset], 2);
			s->button4 = getBit(message[0 + offset], 3);
			s->button5 = getBit(message[0 + offset], 4);
			s->button6 = getBit(message[0 + offset], 5);
			s->button7 = getBit(message[0 + offset], 6);
			s->button8 = getBit(message[0 + offset], 7);
			s->deltaX = toInt8(&message[1 + offset]);
			s->deltaY = toInt8(&message[2 + offset]);
			s->deltaWheel = toInt8(&message[3 + offset]);
			s->sequenceNumber = toUint16(&message[4 + offset]);
			s->linearAccelX = toInt16(&message[6 + offset]);
			s->linearAccelY = toInt16(&message[8 + offset]);
			s->linearAccelZ = toInt16(&message[10 + offset]);
			s->angularVelX = toInt16(&message[12 + offset]);
			s->angularVelY = toInt16(&message[14 + offset]);
			s->angularVelZ = toInt16(&message[16 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeUserFrame(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_UserFrame* s = &(m->userFrame);

	m->ver = ver;

	switch(ver) {
		case 1:
            if ((STRICT_DECODE_LENGTH && length != 23) || (!STRICT_DECODE_LENGTH && length < 23)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "UserFrame", 23, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 33) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->button1 = getBit(message[0 + offset], 0);
			s->button2 = getBit(message[0 + offset], 1);
			s->button3 = getBit(message[0 + offset], 2);
			s->button4 = getBit(message[0 + offset], 3);
			s->button5 = getBit(message[0 + offset], 4);
			s->button6 = getBit(message[0 + offset], 5);
			s->button7 = getBit(message[0 + offset], 6);
			s->button8 = getBit(message[0 + offset], 7);
			s->deltaX = toInt8(&message[1 + offset]);
			s->deltaY = toInt8(&message[2 + offset]);
			s->deltaWheel = toInt8(&message[3 + offset]);
			s->sequenceNumber = toUint16(&message[4 + offset]);
			s->linearPosX = toInt16(&message[8 + offset]);
			s->linearPosY = toInt16(&message[10 + offset]);
			s->linearPosZ = toInt16(&message[12 + offset]);
			s->angularPosA = toInt16(&message[14 + offset]);
			s->angularPosB = toInt16(&message[16 + offset]);
			s->angularPosC = toInt16(&message[18 + offset]);
			s->angularPosD = toInt16(&message[20 + offset]);
			return FREESPACE_SUCCESS;
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 22) || (!STRICT_DECODE_LENGTH && length < 22)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "UserFrame", 22, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 33) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];
			s->button1 = getBit(message[0 + offset], 0);
			s->button2 = getBit(message[0 + offset], 1);
			s->button3 = getBit(message[0 + offset], 2);
			s->button4 = getBit(message[0 + offset], 3);
			s->button5 = getBit(message[0 + offset], 4);
			s->button6 = getBit(message[0 + offset], 5);
			s->button7 = getBit(message[0 + offset], 6);
			s->button8 = getBit(message[0 + offset], 7);
			s->deltaX = toInt8(&message[1 + offset]);
			s->deltaY = toInt8(&message[2 + offset]);
			s->deltaWheel = toInt8(&message[3 + offset]);
			s->sequenceNumber = toUint16(&message[4 + offset]);
			s->linearPosX = toInt16(&message[6 + offset]);
			s->linearPosY = toInt16(&message[8 + offset]);
			s->linearPosZ = toInt16(&message[10 + offset]);
			s->angularPosB = toInt16(&message[12 + offset]);
			s->angularPosC = toInt16(&message[14 + offset]);
			s->angularPosD = toInt16(&message[16 + offset]);
			s->angularPosA = (int16_t) sqrt(268435456 - ((s->angularPosB * s->angularPosB) + (s->angularPosC * s->angularPosC) + (s->angularPosD * s->angularPosD)));
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_encodeDataMotionControl(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_DataMotionControl* s = &(m->dataMotionControl);

	switch(m->ver) {
		case 1:
			if (maxlength < 2) {
				CODECS_PRINTF("freespace_DataMotionControl encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 34;
			message[0 + offset] = (((s->enableBodyMotion & 0x1) << 0)
								|  ((s->enableUserPosition & 0x1) << 1)
								|  ((s->inhibitPowerManager & 0x1) << 2)
								|  ((s->enableMouseMovement & 0x1) << 3)
								|  ((s->disableFreespace & 0x1) << 4));
			return 1 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_decodeFRSWriteResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSWriteResponse* s = &(m->fRSWriteResponse);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 8) || (!STRICT_DECODE_LENGTH && length < 8)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSWriteResponse", 8, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 6) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->status = toUint8(&message[1 + offset]);
			s->wordOffset = toUint16(&message[2 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeDataModeControlV2Response(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_DataModeControlV2Response* s = &(m->dataModeControlV2Response);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 9) || (!STRICT_DECODE_LENGTH && length < 9)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "DataModeControlV2Response", 9, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 20) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->operatingStatus = getBit(message[1 + offset], 0);
			s->mode = (uint8_t) ((message[1 + offset] >> 1) & 0x07);
			s->outputStatus = getBit(message[1 + offset], 4);
			s->modeActual = (uint8_t) ((message[1 + offset] >> 5) & 0x07);
			s->packetSelect = toUint8(&message[2 + offset]);
			s->formatSelect = toUint8(&message[3 + offset]);
			s->ff0 = getBit(message[4 + offset], 0);
			s->ff1 = getBit(message[4 + offset], 1);
			s->ff2 = getBit(message[4 + offset], 2);
			s->ff3 = getBit(message[4 + offset], 3);
			s->ff4 = getBit(message[4 + offset], 4);
			s->ff5 = getBit(message[4 + offset], 5);
			s->ff6 = getBit(message[4 + offset], 6);
			s->ff7 = getBit(message[4 + offset], 7);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeSensorPeriodResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_SensorPeriodResponse* s = &(m->sensorPeriodResponse);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 12) || (!STRICT_DECODE_LENGTH && length < 12)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "SensorPeriodResponse", 12, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 5) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 23) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->sensor = toUint8(&message[2 + offset]);
			s->period = toUint32(&message[4 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSReadResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSReadResponse* s = &(m->fRSReadResponse);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 22) || (!STRICT_DECODE_LENGTH && length < 22)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSReadResponse", 22, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 6) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
    s->status = getNibble(message[1 + offset], 0);
    s->dataLength = getNibble(message[1 + offset], 1);
			s->wordOffset = toUint16(&message[2 + offset]);
			s->data[0] = toUint32(&message[4 + offset]);
			s->data[1] = toUint32(&message[8 + offset]);
			s->data[2] = toUint32(&message[12 + offset]);
			s->FRStype = toUint16(&message[16 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodePerResponse(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_PerResponse* s = &(m->perResponse);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 22) || (!STRICT_DECODE_LENGTH && length < 22)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "PerResponse", 22, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 6) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 16) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->count = toUint32(&message[2 + offset]);
			s->msError = toUint32(&message[6 + offset]);
			s->smError = toUint32(&message[10 + offset]);
			s->frError = toUint32(&message[14 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_encodeFRSWriteRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSWriteRequest* s = &(m->fRSWriteRequest);

	switch(m->ver) {
		case 2:
			if (maxlength < 10) {
				CODECS_PRINTF("freespace_FRSWriteRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 6;
			message[2 + offset] = s->length >> 0;
			message[3 + offset] = s->length >> 8;
			message[4 + offset] = s->FRStype >> 0;
			message[5 + offset] = s->FRStype >> 8;
			message[1] = 6 + offset;
			return 6 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSWriteData(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSWriteData* s = &(m->fRSWriteData);

	switch(m->ver) {
		case 2:
			if (maxlength < 12) {
				CODECS_PRINTF("freespace_FRSWriteData encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 7;
			message[2 + offset] = s->wordOffset >> 0;
			message[3 + offset] = s->wordOffset >> 8;
			message[4 + offset] = s->data >> 0;
			message[5 + offset] = s->data >> 8;
			message[6 + offset] = s->data >> 16;
			message[7 + offset] = s->data >> 24;
			message[1] = 8 + offset;
			return 8 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeFRSReadRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_FRSReadRequest* s = &(m->fRSReadRequest);

	switch(m->ver) {
		case 2:
			if (maxlength < 12) {
				CODECS_PRINTF("freespace_FRSReadRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 8;
			message[2 + offset] = s->readOffset >> 0;
			message[3 + offset] = s->readOffset >> 8;
			message[4 + offset] = s->FRStype >> 0;
			message[5 + offset] = s->FRStype >> 8;
			message[6 + offset] = s->BlockSize >> 0;
			message[7 + offset] = s->BlockSize >> 8;
			message[1] = 8 + offset;
			return 8 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodePerRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_PerRequest* s = &(m->perRequest);

	switch(m->ver) {
		case 2:
			if (maxlength < 11) {
				CODECS_PRINTF("freespace_PerRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 16;
			message[1 + offset] = s->op >> 0;
			message[2 + offset] = s->payload[0] >> 0;
			message[3 + offset] = s->payload[1] >> 0;
			message[4 + offset] = s->payload[2] >> 0;
			message[5 + offset] = s->payload[3] >> 0;
			message[6 + offset] = s->payload[4] >> 0;
			message[1] = 7 + offset;
			return 7 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeActivityClassificationNotification(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_ActivityClassificationNotification* s = &(m->activityClassificationNotification);

	switch(m->ver) {
		case 2:
			if (maxlength < 6) {
				CODECS_PRINTF("freespace_ActivityClassificationNotification encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 18;
			message[1 + offset] = s->classification >> 0;
			message[1] = 2 + offset;
			return 2 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeDataModeControlV2Request(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_DataModeControlV2Request* s = &(m->dataModeControlV2Request);

	switch(m->ver) {
		case 2:
			if (maxlength < 9) {
				CODECS_PRINTF("freespace_DataModeControlV2Request encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 20;
			message[1 + offset] = (((s->operatingStatus & 0x1) << 0)
								|  ((s->mode & 0x7) << 1)
								|  ((s->outputStatus & 0x1) << 4));
			message[2 + offset] = s->packetSelect >> 0;
			message[3 + offset] = s->formatSelect >> 0;
			message[4 + offset] = (((s->ff0 & 0x1) << 0)
								|  ((s->ff1 & 0x1) << 1)
								|  ((s->ff2 & 0x1) << 2)
								|  ((s->ff3 & 0x1) << 3)
								|  ((s->ff4 & 0x1) << 4)
								|  ((s->ff5 & 0x1) << 5)
								|  ((s->ff6 & 0x1) << 6)
								|  ((s->ff7 & 0x1) << 7));
			message[1] = 5 + offset;
			return 5 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeReorientationRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_ReorientationRequest* s = &(m->reorientationRequest);

	switch(m->ver) {
		case 2:
			if (maxlength < 10) {
				CODECS_PRINTF("freespace_ReorientationRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 21;
			message[1 + offset] = (((s->select & 0x1) << 6)
								|  ((s->commit & 0x1) << 7));
			message[2 + offset] = s->quaternionParameter1 >> 0;
			message[3 + offset] = s->quaternionParameter1 >> 8;
			message[4 + offset] = s->quaternionParameter2 >> 0;
			message[5 + offset] = s->quaternionParameter2 >> 8;
			message[1] = 6 + offset;
			return 6 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeSensorPeriodRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_SensorPeriodRequest* s = &(m->sensorPeriodRequest);

	switch(m->ver) {
		case 2:
			if (maxlength < 12) {
				CODECS_PRINTF("freespace_SensorPeriodRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 22;
			message[1 + offset] = (((s->commit & 0x1) << 0)
								|  ((s->get & 0x1) << 1));
			message[2 + offset] = s->sensor >> 0;
			message[4 + offset] = s->period >> 0;
			message[5 + offset] = s->period >> 8;
			message[6 + offset] = s->period >> 16;
			message[7 + offset] = s->period >> 24;
			message[1] = 8 + offset;
			return 8 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_encodeBmsRequest(const struct freespace_message* m, uint8_t* message, int maxlength) {

	uint8_t offset = 1;
	const struct freespace_BmsRequest* s = &(m->bmsRequest);

	switch(m->ver) {
		case 2:
			if (maxlength < 6) {
				CODECS_PRINTF("freespace_BmsRequest encode(<INVALID LENGTH>)\n");
				return FREESPACE_ERROR_BUFFER_TOO_SMALL;
			}
			message[0] = (uint8_t) 7;
			message[2] = m->dest;
			message[3] = m->src;
			offset = 4;
			message[0 + offset] = (uint8_t) 23;
			message[1 + offset] = s->bmsRequest >> 0;
			message[1] = 2 + offset;
			return 2 + offset;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}}

LIBFREESPACE_API int freespace_decodeProductIDResponseBLE(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_ProductIDResponseBLE* s = &(m->productIDResponseBLE);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 20) || (!STRICT_DECODE_LENGTH && length < 20)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "ProductIDResponseBLE", 20, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 22) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->deviceClass = (uint8_t) ((message[1 + offset] >> 0) & 0x3F);
			s->startup = getBit(message[1 + offset], 6);
			s->invalidNS = getBit(message[1 + offset], 7);
			s->swVersionMajor = toUint8(&message[2 + offset]);
			s->swVersionMinor = toUint8(&message[3 + offset]);
			s->swPartNumber = toUint32(&message[4 + offset]);
			s->serialNumber = toUint32(&message[8 + offset]);
			s->swBuildNumber = toUint16(&message[12 + offset]);
			s->swVersionPatch = toUint16(&message[14 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeFRSReadResponseBLE(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_FRSReadResponseBLE* s = &(m->fRSReadResponseBLE);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 20) || (!STRICT_DECODE_LENGTH && length < 20)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "FRSReadResponseBLE", 20, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 8) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 21) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
    s->status = getNibble(message[1 + offset], 0);
    s->dataLength = getNibble(message[1 + offset], 1);
			s->wordOffset = toUint16(&message[2 + offset]);
			s->data[0] = toUint32(&message[4 + offset]);
			s->data[1] = toUint32(&message[8 + offset]);
			s->reserved = toUint16(&message[12 + offset]);
			s->FRStype = toUint16(&message[14 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeBodyUserFrame(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_BodyUserFrame* s = &(m->bodyUserFrame);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 36) || (!STRICT_DECODE_LENGTH && length < 36)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "BodyUserFrame", 36, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 34) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];
			s->button1 = getBit(message[0 + offset], 0);
			s->button2 = getBit(message[0 + offset], 1);
			s->button3 = getBit(message[0 + offset], 2);
			s->button4 = getBit(message[0 + offset], 3);
			s->button5 = getBit(message[0 + offset], 4);
			s->button6 = getBit(message[0 + offset], 5);
			s->button7 = getBit(message[0 + offset], 6);
			s->button8 = getBit(message[0 + offset], 7);
			s->deltaX = toInt8(&message[1 + offset]);
			s->deltaY = toInt8(&message[2 + offset]);
			s->deltaWheel = toInt8(&message[3 + offset]);
			s->sequenceNumber = toUint16(&message[4 + offset]);
			s->linearAccelX = toInt16(&message[6 + offset]);
			s->linearAccelY = toInt16(&message[8 + offset]);
			s->linearAccelZ = toInt16(&message[10 + offset]);
			s->angularVelX = toInt16(&message[12 + offset]);
			s->angularVelY = toInt16(&message[14 + offset]);
			s->angularVelZ = toInt16(&message[16 + offset]);
			s->linearPosX = toInt16(&message[18 + offset]);
			s->linearPosY = toInt16(&message[20 + offset]);
			s->linearPosZ = toInt16(&message[22 + offset]);
			s->angularPosB = toInt16(&message[24 + offset]);
			s->angularPosC = toInt16(&message[26 + offset]);
			s->angularPosD = toInt16(&message[28 + offset]);
			s->angularPosA = toInt16(&message[30 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeMotionEngineOutput(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_MotionEngineOutput* s = &(m->motionEngineOutput);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 54) || (!STRICT_DECODE_LENGTH && length < 54)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "MotionEngineOutput", 54, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 38) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];
			s->formatSelect = toUint8(&message[0 + offset]);
			s->ff0 = getBit(message[1 + offset], 0);
			s->ff1 = getBit(message[1 + offset], 1);
			s->ff2 = getBit(message[1 + offset], 2);
			s->ff3 = getBit(message[1 + offset], 3);
			s->ff4 = getBit(message[1 + offset], 4);
			s->ff5 = getBit(message[1 + offset], 5);
			s->ff6 = getBit(message[1 + offset], 6);
			s->ff7 = getBit(message[1 + offset], 7);
			s->sequenceNumber = toUint32(&message[2 + offset]);
			s->meData[0] = toUint8(&message[6 + offset]);
			s->meData[1] = toUint8(&message[7 + offset]);
			s->meData[2] = toUint8(&message[8 + offset]);
			s->meData[3] = toUint8(&message[9 + offset]);
			s->meData[4] = toUint8(&message[10 + offset]);
			s->meData[5] = toUint8(&message[11 + offset]);
			s->meData[6] = toUint8(&message[12 + offset]);
			s->meData[7] = toUint8(&message[13 + offset]);
			s->meData[8] = toUint8(&message[14 + offset]);
			s->meData[9] = toUint8(&message[15 + offset]);
			s->meData[10] = toUint8(&message[16 + offset]);
			s->meData[11] = toUint8(&message[17 + offset]);
			s->meData[12] = toUint8(&message[18 + offset]);
			s->meData[13] = toUint8(&message[19 + offset]);
			s->meData[14] = toUint8(&message[20 + offset]);
			s->meData[15] = toUint8(&message[21 + offset]);
			s->meData[16] = toUint8(&message[22 + offset]);
			s->meData[17] = toUint8(&message[23 + offset]);
			s->meData[18] = toUint8(&message[24 + offset]);
			s->meData[19] = toUint8(&message[25 + offset]);
			s->meData[20] = toUint8(&message[26 + offset]);
			s->meData[21] = toUint8(&message[27 + offset]);
			s->meData[22] = toUint8(&message[28 + offset]);
			s->meData[23] = toUint8(&message[29 + offset]);
			s->meData[24] = toUint8(&message[30 + offset]);
			s->meData[25] = toUint8(&message[31 + offset]);
			s->meData[26] = toUint8(&message[32 + offset]);
			s->meData[27] = toUint8(&message[33 + offset]);
			s->meData[28] = toUint8(&message[34 + offset]);
			s->meData[29] = toUint8(&message[35 + offset]);
			s->meData[30] = toUint8(&message[36 + offset]);
			s->meData[31] = toUint8(&message[37 + offset]);
			s->meData[32] = toUint8(&message[38 + offset]);
			s->meData[33] = toUint8(&message[39 + offset]);
			s->meData[34] = toUint8(&message[40 + offset]);
			s->meData[35] = toUint8(&message[41 + offset]);
			s->meData[36] = toUint8(&message[42 + offset]);
			s->meData[37] = toUint8(&message[43 + offset]);
			s->meData[38] = toUint8(&message[44 + offset]);
			s->meData[39] = toUint8(&message[45 + offset]);
			s->meData[40] = toUint8(&message[46 + offset]);
			s->meData[41] = toUint8(&message[47 + offset]);
			s->meData[42] = toUint8(&message[48 + offset]);
			s->meData[43] = toUint8(&message[49 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeDceOutV2(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_DceOutV2* s = &(m->dceOutV2);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 31) || (!STRICT_DECODE_LENGTH && length < 31)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "DceOutV2", 31, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 39) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];
			s->sampleBase = toUint32(&message[0 + offset]);
			s->ax = toInt16(&message[4 + offset]);
			s->ay = toInt16(&message[6 + offset]);
			s->az = toInt16(&message[8 + offset]);
			s->rx = toInt16(&message[10 + offset]);
			s->ry = toInt16(&message[12 + offset]);
			s->rz = toInt16(&message[14 + offset]);
			s->mx = toInt16(&message[16 + offset]);
			s->my = toInt16(&message[18 + offset]);
			s->mz = toInt16(&message[20 + offset]);
			s->temperature = toInt16(&message[22 + offset]);
			s->flags = toInt8(&message[24 + offset]);
			s->button1 = getBit(message[25 + offset], 0);
			s->button2 = getBit(message[25 + offset], 1);
			s->button3 = getBit(message[25 + offset], 2);
			s->button4 = getBit(message[25 + offset], 3);
			s->button5 = getBit(message[25 + offset], 4);
			s->button6 = getBit(message[25 + offset], 5);
			s->button7 = getBit(message[25 + offset], 6);
			s->button8 = getBit(message[25 + offset], 7);
			s->deltaWheel = toInt8(&message[26 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeDceOutV3(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_DceOutV3* s = &(m->dceOutV3);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 20) || (!STRICT_DECODE_LENGTH && length < 20)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "DceOutV3", 20, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 40) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];
			s->sampleBase = toUint8(&message[0 + offset]);
			s->button1 = getBit(message[1 + offset], 0);
			s->button2 = getBit(message[1 + offset], 1);
			s->button3 = getBit(message[1 + offset], 2);
			s->button4 = getBit(message[1 + offset], 3);
			s->ff1 = getBit(message[1 + offset], 4);
			s->ff2 = getBit(message[1 + offset], 5);
			s->ff3 = getBit(message[1 + offset], 6);
			s->ff4 = getBit(message[1 + offset], 7);
			s->ax = toInt16(&message[2 + offset]);
			s->ay = toInt16(&message[4 + offset]);
			s->az = toInt16(&message[6 + offset]);
			s->rx = toInt16(&message[8 + offset]);
			s->ry = toInt16(&message[10 + offset]);
			s->rz = toInt16(&message[12 + offset]);
			s->temperature = toInt16(&message[14 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeDceOutV4T0(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_DceOutV4T0* s = &(m->dceOutV4T0);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 20) || (!STRICT_DECODE_LENGTH && length < 20)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "DceOutV4T0", 20, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 41) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 0) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->sampleBase = toUint8(&message[1 + offset]);
			s->ax = toInt16(&message[2 + offset]);
			s->ay = toInt16(&message[4 + offset]);
			s->az = toInt16(&message[6 + offset]);
			s->rx = toInt16(&message[8 + offset]);
			s->ry = toInt16(&message[10 + offset]);
			s->rz = toInt16(&message[12 + offset]);
			s->temperature = toInt16(&message[14 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}

LIBFREESPACE_API int freespace_decodeDceOutV4T1(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {
	uint8_t offset = 1;
	struct freespace_DceOutV4T1* s = &(m->dceOutV4T1);

	m->ver = ver;

	switch(ver) {
		case 2:
            if ((STRICT_DECODE_LENGTH && length != 15) || (!STRICT_DECODE_LENGTH && length < 15)) {
                CODECS_PRINTF("Length mismatch for %s.  Expected %d.  Got %d.\n", "DceOutV4T1", 15, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != 41) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			offset = 4;
			m->len = message[1];
			m->dest = message[2];
			m->src = message[3];

            if ((uint8_t) message[offset] != 1) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
			s->sampleBase = toUint8(&message[1 + offset]);
			s->mx = toInt16(&message[2 + offset]);
			s->my = toInt16(&message[4 + offset]);
			s->mz = toInt16(&message[6 + offset]);
			s->flag1 = getBit(message[8 + offset], 0);
			s->flag2 = getBit(message[8 + offset], 1);
			s->flag3 = getBit(message[8 + offset], 2);
			s->flag4 = getBit(message[8 + offset], 3);
			s->ff1 = getBit(message[8 + offset], 4);
			s->ff2 = getBit(message[8 + offset], 5);
			s->ff3 = getBit(message[8 + offset], 6);
			s->ff4 = getBit(message[8 + offset], 7);
			s->button1 = getBit(message[9 + offset], 0);
			s->button2 = getBit(message[9 + offset], 1);
			s->button3 = getBit(message[9 + offset], 2);
			s->button4 = getBit(message[9 + offset], 3);
			s->button5 = getBit(message[9 + offset], 4);
			s->button6 = getBit(message[9 + offset], 5);
			s->button7 = getBit(message[9 + offset], 6);
			s->button8 = getBit(message[9 + offset], 7);
			s->deltaWheel = toInt8(&message[10 + offset]);
			return FREESPACE_SUCCESS;
		default:
			return  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
	}
}


LIBFREESPACE_API int freespace_decode_message(const uint8_t* message, int length, struct freespace_message* s, uint8_t ver) {
    if (length == 0) {
        return -1;
    }

    switch (ver) {
		case 0:
			switch(message[0]) {
                default:
                    return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
		case 1:
			switch(message[0]) {
				case 8:
                    switch (message[1]) {
                        case 13:
                            s->messageType = FREESPACE_MESSAGE_PAIRINGRESPONSE;
                            return freespace_decodePairingResponse(message, length, s, ver);
                        case 32:
                            s->messageType = FREESPACE_MESSAGE_PRODUCTIDRESPONSE;
                            return freespace_decodeProductIDResponse(message, length, s, ver);
                        case 48:
                            s->messageType = FREESPACE_MESSAGE_LINKSTATUS;
                            return freespace_decodeLinkStatus(message, length, s, ver);
                        case 49:
                            s->messageType = FREESPACE_MESSAGE_ALWAYSONRESPONSE;
                            return freespace_decodeAlwaysOnResponse(message, length, s, ver);
                        case 58:
                            s->messageType = FREESPACE_MESSAGE_FRSHANDHELDREADRESPONSE;
                            return freespace_decodeFRSHandheldReadResponse(message, length, s, ver);
                        case 61:
                            s->messageType = FREESPACE_MESSAGE_FRSHANDHELDWRITERESPONSE;
                            return freespace_decodeFRSHandheldWriteResponse(message, length, s, ver);
                        case 59:
                            s->messageType = FREESPACE_MESSAGE_FRSDONGLEREADRESPONSE;
                            return freespace_decodeFRSDongleReadResponse(message, length, s, ver);
                        case 62:
                            s->messageType = FREESPACE_MESSAGE_FRSDONGLEWRITERESPONSE;
                            return freespace_decodeFRSDongleWriteResponse(message, length, s, ver);
                        case 65:
                            s->messageType = FREESPACE_MESSAGE_FRSEFLASHREADRESPONSE;
                            return freespace_decodeFRSEFlashReadResponse(message, length, s, ver);
                        case 66:
                            s->messageType = FREESPACE_MESSAGE_FRSEFLASHWRITERESPONSE;
                            return freespace_decodeFRSEFlashWriteResponse(message, length, s, ver);
                        case 73:
                            s->messageType = FREESPACE_MESSAGE_DATAMODERESPONSE;
                            return freespace_decodeDataModeResponse(message, length, s, ver);
                        case 81:
                            s->messageType = FREESPACE_MESSAGE_BUTTONTESTMODERESPONSE;
                            return freespace_decodeButtonTestModeResponse(message, length, s, ver);
                        default:
                            return FREESPACE_ERROR_MALFORMED_MESSAGE;
                    }
				case 10:
                    s->messageType = FREESPACE_MESSAGE_BATTERYLEVEL;
                    return freespace_decodeBatteryLevel(message, length, s, ver);
				case 32:
                    s->messageType = FREESPACE_MESSAGE_BODYFRAME;
                    return freespace_decodeBodyFrame(message, length, s, ver);
				case 33:
                    s->messageType = FREESPACE_MESSAGE_USERFRAME;
                    return freespace_decodeUserFrame(message, length, s, ver);
                default:
                    return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
		case 2:
			switch(message[0]) {
				case 5:
                    switch (message[4]) {
                        case 2:
                            s->messageType = FREESPACE_MESSAGE_PAIRINGRESPONSE;
                            return freespace_decodePairingResponse(message, length, s, ver);
                        case 3:
                            s->messageType = FREESPACE_MESSAGE_LINKSTATUS;
                            return freespace_decodeLinkStatus(message, length, s, ver);
                        case 4:
                            s->messageType = FREESPACE_MESSAGE_DATAMODERESPONSE;
                            return freespace_decodeDataModeResponse(message, length, s, ver);
                        case 17:
                            s->messageType = FREESPACE_MESSAGE_BUTTONTESTMODERESPONSE;
                            return freespace_decodeButtonTestModeResponse(message, length, s, ver);
                        case 5:
                            s->messageType = FREESPACE_MESSAGE_BATTERYLEVEL;
                            return freespace_decodeBatteryLevel(message, length, s, ver);
                        case 6:
                            s->messageType = FREESPACE_MESSAGE_FRSWRITERESPONSE;
                            return freespace_decodeFRSWriteResponse(message, length, s, ver);
                        case 20:
                            s->messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2RESPONSE;
                            return freespace_decodeDataModeControlV2Response(message, length, s, ver);
                        case 23:
                            s->messageType = FREESPACE_MESSAGE_SENSORPERIODRESPONSE;
                            return freespace_decodeSensorPeriodResponse(message, length, s, ver);
                        default:
                            return FREESPACE_ERROR_MALFORMED_MESSAGE;
                    }
				case 6:
                    switch (message[4]) {
                        case 9:
                            s->messageType = FREESPACE_MESSAGE_PRODUCTIDRESPONSE;
                            return freespace_decodeProductIDResponse(message, length, s, ver);
                        case 8:
                            s->messageType = FREESPACE_MESSAGE_FRSREADRESPONSE;
                            return freespace_decodeFRSReadResponse(message, length, s, ver);
                        case 16:
                            s->messageType = FREESPACE_MESSAGE_PERRESPONSE;
                            return freespace_decodePerResponse(message, length, s, ver);
                        default:
                            return FREESPACE_ERROR_MALFORMED_MESSAGE;
                    }
				case 32:
                    s->messageType = FREESPACE_MESSAGE_BODYFRAME;
                    return freespace_decodeBodyFrame(message, length, s, ver);
				case 33:
                    s->messageType = FREESPACE_MESSAGE_USERFRAME;
                    return freespace_decodeUserFrame(message, length, s, ver);
				case 8:
                    switch (message[4]) {
                        case 22:
                            s->messageType = FREESPACE_MESSAGE_PRODUCTIDRESPONSEBLE;
                            return freespace_decodeProductIDResponseBLE(message, length, s, ver);
                        case 21:
                            s->messageType = FREESPACE_MESSAGE_FRSREADRESPONSEBLE;
                            return freespace_decodeFRSReadResponseBLE(message, length, s, ver);
                        default:
                            return FREESPACE_ERROR_MALFORMED_MESSAGE;
                    }
				case 34:
                    s->messageType = FREESPACE_MESSAGE_BODYUSERFRAME;
                    return freespace_decodeBodyUserFrame(message, length, s, ver);
				case 38:
                    s->messageType = FREESPACE_MESSAGE_MOTIONENGINEOUTPUT;
                    return freespace_decodeMotionEngineOutput(message, length, s, ver);
				case 39:
                    s->messageType = FREESPACE_MESSAGE_DCEOUTV2;
                    return freespace_decodeDceOutV2(message, length, s, ver);
				case 40:
                    s->messageType = FREESPACE_MESSAGE_DCEOUTV3;
                    return freespace_decodeDceOutV3(message, length, s, ver);
				case 41:
                    switch (message[4]) {
                        case 0:
                            s->messageType = FREESPACE_MESSAGE_DCEOUTV4T0;
                            return freespace_decodeDceOutV4T0(message, length, s, ver);
                        case 1:
                            s->messageType = FREESPACE_MESSAGE_DCEOUTV4T1;
                            return freespace_decodeDceOutV4T1(message, length, s, ver);
                        default:
                            return FREESPACE_ERROR_MALFORMED_MESSAGE;
                    }
                default:
                    return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }

    default:
        return FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
    }
}

LIBFREESPACE_API int freespace_encode_message(struct freespace_message* message, uint8_t* msgBuf, int maxlength) {
    message->src = 0; // Force source to 0, since this is coming from the system host.
    switch (message->messageType) {
        case FREESPACE_MESSAGE_PAIRINGMESSAGE:
            return freespace_encodePairingMessage(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_PRODUCTIDREQUEST:
            return freespace_encodeProductIDRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_LEDSETREQUEST:
            return freespace_encodeLEDSetRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_LINKQUALITYREQUEST:
            return freespace_encodeLinkQualityRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_ALWAYSONREQUEST:
            return freespace_encodeAlwaysOnRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FREQUENCYFIXREQUEST:
            return freespace_encodeFrequencyFixRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_SOFTWARERESETMESSAGE:
            return freespace_encodeSoftwareResetMessage(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_DONGLERFDISABLEMESSAGE:
            return freespace_encodeDongleRFDisableMessage(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_TXDISABLEMESSAGE:
            return freespace_encodeTxDisableMessage(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_DONGLERFSUPRESSHOMEFREQUENCYMESSAGE:
            return freespace_encodeDongleRFSupressHomeFrequencyMessage(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSHANDHELDREADREQUEST:
            return freespace_encodeFRSHandheldReadRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSHANDHELDWRITEREQUEST:
            return freespace_encodeFRSHandheldWriteRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSHANDHELDWRITEDATA:
            return freespace_encodeFRSHandheldWriteData(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSDONGLEREADREQUEST:
            return freespace_encodeFRSDongleReadRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSDONGLEWRITEREQUEST:
            return freespace_encodeFRSDongleWriteRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSDONGLEWRITEDATA:
            return freespace_encodeFRSDongleWriteData(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSEFLASHREADREQUEST:
            return freespace_encodeFRSEFlashReadRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSEFLASHWRITEREQUEST:
            return freespace_encodeFRSEFlashWriteRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSEFLASHWRITEDATA:
            return freespace_encodeFRSEFlashWriteData(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_DONGLERFENABLEMESSAGE:
            return freespace_encodeDongleRFEnableMessage(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_DATAMODEREQUEST:
            return freespace_encodeDataModeRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_BUTTONTESTMODEREQUEST:
            return freespace_encodeButtonTestModeRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_BATTERYLEVELREQUEST:
            return freespace_encodeBatteryLevelRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_DATAMOTIONCONTROL:
            return freespace_encodeDataMotionControl(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSWRITEREQUEST:
            return freespace_encodeFRSWriteRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSWRITEDATA:
            return freespace_encodeFRSWriteData(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_FRSREADREQUEST:
            return freespace_encodeFRSReadRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_PERREQUEST:
            return freespace_encodePerRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_ACTIVITYCLASSIFICATIONNOTIFICATION:
            return freespace_encodeActivityClassificationNotification(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST:
            return freespace_encodeDataModeControlV2Request(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_REORIENTATIONREQUEST:
            return freespace_encodeReorientationRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_SENSORPERIODREQUEST:
            return freespace_encodeSensorPeriodRequest(message, msgBuf, maxlength);
        case FREESPACE_MESSAGE_BMSREQUEST:
            return freespace_encodeBmsRequest(message, msgBuf, maxlength);
        default:
            return -1;
        }
}