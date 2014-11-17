/************ WARNING ************/
/* Disconnect Serial Port 0      */
/* Before attempting to upload   */
/* new code                      */
/*********************************/

/***
	Serial Mux for Seeduino Mega V1.22 board with AT1280 processor
	Receives data from each of three serial ports, ports 0, 1 & 2 
	and sends data out to the other two ports respectively
	
	Created  12 Apr. 2011 Version 1 by B Hill 
	Modified 31 Jul. 2012 Version 2 by S Vazquez
	Modified 09 Jan. 2013 Version 3 by S Vazquez/B Hill 
***/

/***
	N803RE uses an AT1280 processor
***/

/***
	Removed vehicle ID validation check
	It reads from corresponding port and 
	passes the packet to the AP. Acceptance
	or rejection of packet is responsibility
	of AP.
***/

//#include "G:\ArduRepo\ArduPlane\mydefines.h"
/*
#define SLV_R1 1
#define SLV_R2 2
#define SLV_R3 3
#define SLV_PAWNEE	4
#define SLV_C1	5
#define SLV_C2	6
#define SLV_C3	7
#define SLV_C4	8
#define SLV_R88 88	// HILSIM Vehicle

// Select Aircraft
#define SLV_AIRCRAFT	SLV_C4
#define SLV_GCS			(255-SLV_AIRCRAFT)
*/

#define DEBUG 1

// Global Variables
uint8_t inByte1;

uint8_t MavID;
uint8_t messLen;
uint8_t messCnt;
uint8_t sysID;
uint8_t ID;
uint8_t messID;

bool messStart;
bool vpacket;
int packetLen;
uint8_t packet[256+8];

//bool slvPort(char* label, HardwareSerial* port, int vsysID)
bool slvPort(char* label, HardwareSerial* port)
{	
	vpacket = false;
	packetLen = 0;
	if (port->available())
	{
		messStart = false;
		int cnt = 0;
		while (port->available() && messStart==false)
		{
			MavID = port->read();
#if DEBUG == 1
			Serial3.println("*");
			Serial3.print(label);
			Serial3.print("[");
			Serial3.print(MavID,DEC);
			Serial3.print(":");
#endif
			if ((MavID == 85) || (MavID == 254))
				messStart = true;
		}
		if (messStart==true)
		{
			cnt = 0;
			while(port->available()<5 && messStart==true)
			{
				delay(1);
				cnt++;
				if (cnt > 100) messStart = false;
			}
			if (messStart)
			{
				messLen = port->read();
				#if DEBUG == 1
					Serial3.print(messLen,DEC);
					Serial3.print(":");
				#endif
				messCnt = port->read();
				#if DEBUG == 1
					Serial3.print(messCnt,DEC);
					Serial3.print(":");
				#endif
				sysID = port->read();
				#if DEBUG == 1
					Serial3.print(sysID,DEC);
					Serial3.print(":");
				#endif
				ID = port->read();
				#if DEBUG == 1
					Serial3.print(ID,DEC);
					Serial3.print(":");
				#endif
				messID = port->read();
				#if DEBUG == 1
					Serial3.print(messID,DEC);
					Serial3.print("]{");
				#endif

			//if (sysID==vsysID && ID==1)
			//if (sysID==vsysID)
			//{
				packet[packetLen++] = MavID;
				packet[packetLen++] = messLen;
				packet[packetLen++] = messCnt;
				packet[packetLen++] = sysID;
				packet[packetLen++] = ID;
				packet[packetLen++] =  messID;
				while (port->available() && packetLen<(messLen+8)) 
				{	
					packet[packetLen++] = port->read();
					delay(1);
				}
				if (packetLen==(messLen+8))
				{
					vpacket = true;
					for (int i=6; i<packetLen; i++)
					{
#if DEBUG == 1
						Serial3.print(packet[i],HEX);
						Serial3.print(":");
#endif
					}
#if DEBUG == 1
					Serial3.print("}");
#endif
				}
				else
					Serial3.println("INCOMPLETE PACKET");
				//}
			}
			else				
				Serial3.println("INVALID HEADER");
		}
	}
	return vpacket;
}

bool slvGetAPpacket()
{
	//return slvPort("    AP",&Serial1,SLV_AIRCRAFT);
	return slvPort("    AP",&Serial1);
}

bool slvGetGCSpacket()
{
	//return slvPort("  XBEE",&Serial,SLV_GCS);
	return slvPort("  XBEE",&Serial);
}

bool slvGetBEAGLEpacket()
{
	//return slvPort("BEAGLE",&Serial2,103);
	return slvPort("BEAGLE",&Serial2);
}

void setup()
{
    delay(10000);
	/*
	Serial.begin (115200); // Initialize port 0: XBEE TELEMETRY
	Serial1.begin(115200); // Initialize port 1: AUTOPILOT
	Serial2.begin(115200); // Initialize port 2: BEAGLEBOARD
	Serial3.begin(115200); // Initialize port 3: DEBUG
	*/
	Serial.begin (57600); // Initialize port 0: XBEE TELEMETRY
	Serial1.begin(57600); // Initialize port 1: AUTOPILOT
	Serial2.begin(57600); // Initialize port 2: BEAGLEBOARD
	Serial3.begin(57600); // Initialize port 3: DEBUG
        
    Serial.flush ();
    Serial1.flush();
    Serial2.flush();
    Serial3.flush();
}

void loop()
{
	// 1ST PRIORITY: read AUTOPILOT data and send to XBEE and BEAGLEBOARD
#if 1
    digitalWrite(13, HIGH);
    while (Serial1.available())
    {
		inByte1 = Serial1.read();
		Serial.write (inByte1); // send to port 0:
		Serial2.write(inByte1); // send to port 2:
	}
	digitalWrite(13,LOW);
#else
	if (slvGetAPpacket())
	{
		for (int i=0; i<packetLen; i++)
		{
			Serial.write (packet[i]);
			Serial2.write(packet[i]);
		}
	}
#endif

	// 2ND PRIORITY: read XBEE data and send to AUTOPILOT
	if (slvGetGCSpacket())
	{
		for (int i=0; i<packetLen; i++)
		{
			Serial1.write(packet[i]);
		}
	}

	// 3RD PRIORITY: read BEAGLEBOARD data and send to AUTOPILOT and XBEE
	if (slvGetBEAGLEpacket())
	{
		for (int i=0; i<packetLen; i++)
		{
			Serial.write (packet[i]);
			Serial1.write(packet[i]);
		}
	}
}

/*
void loop()
{
	// 1ST PRIORITY: read AUTOPILOT data and send to XBEE and BEAGLEBOARD
#if 1
	if (Serial1.available())
	{
        digitalWrite(13, HIGH);
		#if DEBUG == 1
		//Serial3.println("*");
		//Serial3.print("    AP");
		//Serial3.print("[]");
		#endif
        while (Serial1.available())
        {
			inByte1 = Serial1.read();
			Serial.write (inByte1); // send to port 0:
			Serial2.write(inByte1); // send to port 2:
		}
	}
	else
		digitalWrite(13,LOW);
#else
	if (slvGetAPpacket())
	{
		for (int i=0; i<packetLen; i++)
		{
			Serial.write (packet[i]);
			Serial2.write(packet[i]);
		}
	}
#endif

	// 2ND PRIORITY: read XBEE data and send to AUTOPILOT
	if (slvGetGCSpacket())
	{
		for (int i=0; i<packetLen; i++)
		{
			Serial1.write(packet[i]);
		}
	}

	// 3RD PRIORITY: read BEAGLEBOARD data and send to AUTOPILOT
	if (slvGetBEAGLEpacket())
	{
		for (int i=0; i<packetLen; i++)
		{
			Serial.write (packet[i]);
			Serial1.write(packet[i]);
		}
	}
}
*/
