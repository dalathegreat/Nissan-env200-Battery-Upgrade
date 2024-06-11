#include "can.h"

#include "can-bridge-firmware.h"
#include "nissan_can_structs.h"

#include <stdio.h>
#include <string.h>

//General variables
static volatile	uint8_t		battery_soc	= 0; 
static volatile uint16_t	GIDS 				= 0;
static volatile uint8_t	max_charge_80_requested	= 0;
static volatile uint8_t charging_state = 0;

//CAN message templates
static volatile	uint8_t		battery_can_bus			= 2; //keeps track on which CAN bus the battery talks
static	CAN_FRAME		instrumentCluster5E3	= {.ID = 0x5E3, .dlc = 5, .ide = 0, .rtr = 0,  .data = {0x8E,0x00,0x00,0x00,0x80}};
static	CAN_FRAME		ZE1startupMessage603	= {.ID = 0x603, .dlc = 1, .ide = 0, .rtr = 0,  .data = {0x00}};
static	CAN_FRAME		ZE1startupMessage605	= {.ID = 0x605, .dlc = 1, .ide = 0, .rtr = 0,  .data = {0x00}};
static	CAN_FRAME		ZE1message355			= {.ID = 0x355, .dlc = 8, .ide = 0, .rtr = 0,  .data = {0x14,0x0a,0x13,0x97,0x10,0x00,0x40,0x00}}; 
static volatile	uint8_t		ticks10ms				= 0;
static		CAN_FRAME	ZE1message5C5			= {.ID = 0x5C5, .dlc = 8, .ide = 0, .rtr = 0,  .data = {0x84,0x01,0x06,0x79,0x00,0x0C,0x00,0x00}}; 

	void one_second_ping( void )
{
}

static uint8_t	crctable[256] = {0,133,143,10,155,30,20,145,179,54,60,185,40,173,167,34,227,102,108,233,120,253,247,114,80,213,223,90,203,78,68,193,67,198,204,73,216,93,87,210,240,117,127,250,107,238,228,97,160,37,47,170,59,190,180,49,19,150,156,25,136,13,7,130,134,3,9,140,29,152,146,23,53,176,186,63,174,43,33,164,101,224,234,111,254,123,113,244,214,83,89,220,77,200,194,71,197,64,74,207,94,219,209,84,118,243,249,124,237,104,98,231,38,163,169,44,189,56,50,183,149,16,26,159,14,139,129,4,137,12,6,131,18,151,157,24,58,191,181,48,161,36,46,171,106,239,229,96,241,116,126,251,217,92,86,211,66,199,205,72,202,79,69,192,81,212,222,91,121,252,246,115,226,103,109,232,41,172,166,35,178,55,61,184,154,31,21,144,1,132,142,11,15,138,128,5,148,17,27,158,188,57,51,182,39,162,168,45,236,105,99,230,119,242,248,125,95,218,208,85,196,65,75,206,76,201,195,70,215,82,88,221,255,122,112,245,100,225,235,110,175,42,32,165,52,177,187,62,28,153,147,22,135,2,8,141};
void calc_crc8(CAN_FRAME *frame);
	
//recalculates the CRC-8 with 0x85 poly
void calc_crc8(CAN_FRAME *frame)
{
    uint8_t crc = 0;
    
	for(uint8_t j = 0; j < 7; j++)
    {
        crc = crctable[(crc ^ ((int) frame->data[j])) % 256];
    }
    
    frame->data[7] = crc;
}



void can_handler(uint8_t can_bus, CAN_FRAME *frame)
{
		uint16_t temp; // Temporary variable 
    if (1)
    {
        switch (frame->ID)
        {
				case 0x1F2:
					//Collect charging state
					charging_state = frame->data[2];
					//Check if VCM wants to only charge to 80%
					max_charge_80_requested = ((frame->data[0] & 0x80) >> 7);
				
					//Upon reading VCM originating 0x1F2 every 10ms, send the missing message to battery every 40ms
					ticks10ms++;
					if(ticks10ms > 3)
					{
						ticks10ms = 0;
						PushCan(battery_can_bus, CAN_TX, &ZE1message355);
					}

				break;
				case 0x1DB:
					if(max_charge_80_requested)
						{
							if((charging_state == CHARGING_SLOW) && (battery_soc > 80))
							{
								frame->data[1] = (frame->data[1] & 0xE0) | 2; //request charging stop
								frame->data[3] = (frame->data[3] & 0xEF) | 0x10; //full charge completed
							}
						}
					calc_crc8(frame);
				break;
				case 0x55B:
					//Capture SOC% needed for QC_rescaling
					battery_soc = (uint8_t) (((frame->data[0] << 2) | ((frame->data[1] & 0xC0) >> 6)) / 10); 
					
					//Check on what side of the CAN-bridge the battery is connected to
					battery_can_bus = can_bus; 
					
					//Upon reading 0x55B coming from battery every 100ms, send missing messages towards battery
					PushCan(battery_can_bus, CAN_TX, &instrumentCluster5E3);
					PushCan(battery_can_bus, CAN_TX, &ZE1message5C5);

				break;
				case 0x5BC:
					if((frame->data[5] & 0x10) == 0x00)
					{ //LB_MAXGIDS is 0, store GIDS
						GIDS = (uint16_t)((frame->data[0] << 2) | ((frame->data[1] & 0xC0) >> 6));
					}
					
					//Avoid blinking GOM by always writing remaining GIDS
					frame->data[0] = (uint8_t)(GIDS >> 2);
					frame->data[1] = (GIDS << 6) & 0xC0;

				break;
				case 0x59E:   // QC capacity message
					//Calculate new LBC_QC_CapRemaining value
					temp = ((230 * battery_soc)/100); // Crazy advanced math
					frame->data[3] = (frame->data[3] & 0xF0) | ((temp >> 5) & 0xF); // store the new LBC_QC_CapRemaining
					frame->data[4] = (uint8_t)(((temp & 0x1F) <<3) | (frame->data[4] & 0x07)); // to the 59E message out to vehicle
					calc_crc8(frame);
				break;
				case 0x679: //Send missing 2018+ startup messages towards battery
					charging_state	= 0;
					PushCan(battery_can_bus, CAN_TX, &ZE1startupMessage603);
					PushCan(battery_can_bus, CAN_TX, &ZE1startupMessage605);
				break;
        default:
            break;
        }

    
				if (can_bus == 0)
				{
						PushCan(1, CAN_TX, frame);
				}
				else
				{
						PushCan(0, CAN_TX, frame);
				}

    }
}
