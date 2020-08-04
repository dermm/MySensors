/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * -------------------------------------------------------------------------------
 *
 * Copyright (c) 2013, Majenko Technologies and S.J.Hoeksma
 * Copyright (c) 2015, LeoDesigner
 * https://github.com/leodesigner/mysensors-serial-transport
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of Majenko Technologies.
 ********************************************************************************/

// transport(Serial,0,-1); // serial port, node, dePin (-1 disabled)

// Serial Transport



#include "driver/EspNowFloodingMesh.h"


#define ESPnow_PACKET_HEADER_VERSION            (1u)			//!< RFM95 packet header version

#define BUFFER_SIZE 16 //1 2 4 8 16 32
#define BUFFER_MASK (BUFFER_SIZE-1)

unsigned char _nodeId;
char _data[MY_ESPnow_MAX_MESSAGE_LENGTH*BUFFER_SIZE];
uint8_t _packet_len[BUFFER_SIZE];
uint8_t _read;
uint8_t _write;
//unsigned char _packet_from;
//bool _packet_received;
uint32_t _replyId;

uint32_t _sendCount;
uint32_t _recCount;
uint32_t _errorCount;
uint32_t _parseCount;


int _bsid = MY_ESPnow_BSID;
unsigned char _secretKey[16] = MY_ESPnow_secretKey;
unsigned char _iv[16] = MY_ESPnow_iv;
const int _ttl = MY_ESPnow_ttl;

static bool _prepareWiFi() {
  wifi_country_t wc;

  wc.cc[0] = 'J';
  wc.cc[1] = 'P';
  wc.cc[2] = '\0';
  wc.schan = 1;
  wc.nchan = 14;
  wc.policy = WIFI_COUNTRY_POLICY_MANUAL;
  #ifdef ESP32
	return esp_wifi_set_country(&wc);
  #else
  	return wifi_set_country(&wc);
  #endif
}

// Reception state machine control and storage variables
//unsigned char _recCommand;
unsigned char _recLen;
unsigned char _recStation;
unsigned char _recSender;



void _parse(const unsigned char *data, int size, uint32_t replyId)
{
  _replyId = replyId;
  //Serial.print("PARSE:");

  if ( (size>13 && size<55) && data[0] == 'M' && data[1] == 'E' && data[2] == 'T' && data[3] == 'T') {
    //Serial.print("PARSE:");
    //Serial.println((const char*)data);

	// Version  = data[0+4];
	_recStation = data[1+4];
	_recSender 	= data[2+4];
	_recLen 	= data[3+4];
	//_recCommand = data[4+4];
	//_replyId 	= data[5,6,7,8];

	if (_recLen >= MY_ESPnow_MAX_MESSAGE_LENGTH) {
#if MY_ESPnow_Debug
		Serial.println("ESP NOW too long");
#endif
		return;
	}

	if ((_recSender == _nodeId)) {
#if MY_ESPnow_Debug
		Serial.println("ESP NOW from me..");
#endif
		//return;
	}

#if MY_ESPnow_Debug
	if (_recStation != _nodeId && _recStation != BROADCAST_ADDRESS) {
		Serial.println("ESP NOW not for me..");
		//return;
	}
#endif
	_parseCount ++;

	uint8_t next = ((_write + 1) & BUFFER_MASK);

	if (_read == next) { // voll
		_read = (_read+1) & BUFFER_MASK; // letzten löschen
#if MY_ESPnow_Debug
		Serial.println("Puffer Voll!");
#endif
	}

	memcpy(_data+(_write*MY_ESPnow_MAX_MESSAGE_LENGTH),data+13,_recLen);
	//_packet_from = _recSender;
	_packet_len[_write] = _recLen;
	//_packet_received = true;

	_write = next;


  } /*else
  {
	  Serial.println("Parse Error!");
	  for(int i = 0; i < size; i++)
	  {
		Serial.print(data[i]);
	  }
	  Serial.println();
  }*/
  return;
}

void _espNowFloodingMeshRecv(const uint8_t *data, int len, uint32_t replyPrt)
{
	_recCount ++;
	if (len > 0) {
		_parse(data, len, replyPrt); //Parse messages
		//Serial.println((const char*)data);
	}
	if (_replyId > 0) {
		char ackMsg [5] = "ACK";
		espNowFloodingMesh_sendReply((uint8_t*)ackMsg, 4, _ttl, _replyId);
		_replyId = 0;
	}
}

/*
Message size 45
0			1				2			3			4				5 .. 8		9	10 .. 50
Version		_recStation		_recSender	_recLen		_controlFlag	replyID		0	Payload
*/

bool transportSend(const uint8_t to, const void* data, const uint8_t len, const bool noACK)
{
	//Serial.println("ESP NOW SEND1");
	//espNowFloodingMesh_loop();
	char senddata[13+MY_ESPnow_MAX_MESSAGE_LENGTH];
	char *p = senddata;
	snprintf(p, sizeof(senddata)-(p-senddata), "METT");

	senddata[0+4] = ESPnow_PACKET_HEADER_VERSION;
	senddata[1+4] = to;
	senddata[2+4] = _nodeId;
	senddata[3+4] = len;
	senddata[4+4] = 0;
	//senddata[5,6,7,8] = replyID
	senddata[9+4] = 0;

	memcpy(senddata+13,data,len);

	_sendCount ++;

	if ((to != 255) && (noACK == 0)) {
		bool status = espNowFloodingMesh_sendAndWaitReply((uint8_t*)senddata, sizeof(senddata), _ttl, 1,NULL , 250, 1 ); //(msg, size, ttl, tryCount, void (*f), timeoutMs, expectedCountOfReplies)
		if (!status) {
			//Send failed, no connection to master??? Reboot ESP???
#if MY_ESPnow_Debug
			Serial.println("Timeout");
#endif
			_errorCount ++;
			return false;
		}
		return status;
	} else {
		espNowFloodingMesh_send((uint8_t*)senddata, sizeof(senddata), _ttl);
#if MY_ESPnow_Debug
		Serial.println("Send w/o ACK");
#endif
	}
	return true;
}

bool transportInit(void)
{
	if (! _prepareWiFi()) {
		Serial.println(F("wifi_set_country() fail!"));
		Serial.flush();
		ESP.deepSleep(0);
	}

#if MY_ESPnow_Debug
	Serial.println("ESP NOW INIT");
#endif
	Serial.flush();

	_read = 0;
	_write = 0;

	_sendCount = 0;
	_recCount = 0;
	_errorCount = 0;
	_parseCount = 0;

	espNowFloodingMesh_secretKey(_secretKey);
	espNowFloodingMesh_setAesInitializationVector(_iv);
#if MY_ESPnow_master 
	espNowFloodingMesh_setToMasterRole(true, _ttl);
#else
	espNowFloodingMesh_setToMasterRole(false, _ttl);
#endif

#if MY_ESPnow_battery
	espNowFloodingMesh_setToBatteryNode(true);
#endif

#if MY_ESPnow_disableTimeDiffCheck
	espNowFloodingMesh_disableTimeDifferenceCheck (true);
#endif

	espNowFloodingMesh_RecvCB(_espNowFloodingMeshRecv);

	espNowFloodingMesh_begin(MY_ESPnow_CHANNEL, _bsid);

	//Ask instant sync from master.
	if (espNowFloodingMesh_syncTimeAndWait(150,15))
	{
#if MY_ESPnow_Debug
		Serial.println("Sync OK");
#endif
	} else
	{
#if MY_ESPnow_Debug
		Serial.println("Sync ERROR");
#endif		
	}
	//espNowFloodingMesh_loop();
	//espNowFloodingMesh_delay (50);

	return true;
}

void transportSetAddress(const uint8_t address)
{
	_nodeId = address;
}

uint8_t transportGetAddress(void)
{
	return _nodeId;
}


bool transportDataAvailable(void)
{
	espNowFloodingMesh_loop();

	if (_replyId > 0) {
		char ackMsg [5] = "ACK";
		espNowFloodingMesh_sendReply((uint8_t*)ackMsg, 4, _ttl, _replyId);
		_replyId = 0;
	}

	if (_read == _write)
		return false;

	return true;
}

bool transportSanityCheck(void)
{
	// not implemented yet
	return true;
}

uint8_t transportReceive(void* data)
{
	espNowFloodingMesh_loop();

	if (_replyId > 0) {
		char ackMsg [5] = "ACK";
		espNowFloodingMesh_sendReply((uint8_t*)ackMsg, 4, _ttl, _replyId);
		_replyId = 0;
	}
	//Serial.println("ESP NOW REC");
	//Serial.flush();

	if (_read == _write)
		return 0;

	uint8_t read = _read;

	memcpy(data,_data+(read*MY_ESPnow_MAX_MESSAGE_LENGTH),_packet_len[read]);

	_read = (read+1) & BUFFER_MASK;

	return _packet_len[read];

}

void transportPowerDown(void)
{
	// Nothing to shut down here
}

void transportPowerUp(void)
{
	// not implemented
}

void transportSleep(void)
{
	// not implemented
}

void transportStandBy(void)
{
	// not implemented
}

int16_t transportGetSendingRSSI(void)
{
	// not implemented
	return INVALID_RSSI;
}

int16_t transportGetReceivingRSSI(void)
{
	// not implemented
	return INVALID_RSSI;
}

int16_t transportGetSendingSNR(void)
{
	// not implemented
	return INVALID_SNR;
}

int16_t transportGetReceivingSNR(void)
{
	// not implemented
	return INVALID_SNR;
}

int16_t transportGetTxPowerPercent(void)
{
	// not implemented
	return static_cast<int16_t>(100);
}

int16_t transportGetTxPowerLevel(void)
{
	// not implemented
	return static_cast<int16_t>(100);
}

bool transportSetTxPowerPercent(const uint8_t powerPercent)
{
	// not possible
	(void)powerPercent;
	return false;
}
