/*
MCP2515.cpp - CAN library
Written by Frank Kienast in November, 2010
Modified by Christopher Meyer July, 2011

Connections to MCP2515:
Arduino  MCP2515
11       MOSI
12       MISO
13       SCK
10       CS 

*/


#include "WProgram.h"
#include "SPI.h"
#include "MCP2515.h"

boolean MCP2515::initCAN(int baudConst)
{
    /**
     * Sets the baud rate and puts the MCP2515 in CONFIG mode
     * 
     * Arguments
     * baudConst -- A constant indating the desired baud rate. Possible values:
     *              CAN_BAUD_500K
     *              CAN_BAUD_250K
     *              CAN_BAUD_125K
     *              CAN_BAUD_100K
     * 
     * Returns
     * Boolean indicating whether setting the baud rate and puttign the chip 
     * in CONFIG mode was successful
     */
    
	byte mode;

	SPI.begin();

	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(RESET); //Reset cmd
	digitalWrite(SLAVESELECT,HIGH);
	//Read mode and make sure it is config
	delay(100);
	mode = readReg(CANSTAT) >> 5;
	if(mode != 0b100) {
		return false;
	}
	
	return(setCANBaud(baudConst));

}

boolean MCP2515::setCANBaud(int baudConst)
{
    /**
     * Sets the baud rate
     * 
     * Arguments
     * baudConst -- A constant indating the desired baud rate. Possible values:
     *              CAN_BAUD_500K
     *              CAN_BAUD_250K
     *              CAN_BAUD_125K
     *              CAN_BAUD_100K
     * 
     * Returns
     * Boolean indicating whether setting the baud rate was successful
     */
    
	byte brp;

	//BRP<5:0> = 00h, so divisor (0+1)*2 for 125ns per quantum at 16MHz for 500K   
	//SJW<1:0> = 00h, Sync jump width = 1
	switch(baudConst) 	{
        case CAN_BAUD_500K: 
            brp = 0;
            break;
        case CAN_BAUD_250K:
            brp = 1;
            break;
        case CAN_BAUD_125K:
            brp = 3;
            break;
        case CAN_BAUD_100K:
            brp = 4;
            break;
        default:
            return false;
	}
    
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(WRITE); 
	SPI.transfer(CNF1);
	SPI.transfer(brp & 0b00111111);
	digitalWrite(SLAVESELECT,HIGH);  

	//PRSEG<2:0> = 0x01, 2 time quantum for prop
	//PHSEG<2:0> = 0x06, 7 time constants to PS1 sample
	//SAM = 0, just 1 sampling
	//BTLMODE = 1, PS2 determined by CNF3
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(WRITE); 
	SPI.transfer(CNF2);
	SPI.transfer(0b10110001);
	digitalWrite(SLAVESELECT,HIGH); 

	//PHSEG2<2:0> = 5 for 6 time constants after sample
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(WRITE); 
	SPI.transfer(CNF3);
	SPI.transfer(0x05);
	digitalWrite(SLAVESELECT,HIGH); 

	//SyncSeg + PropSeg + PS1 + PS2 = 1 + 2 + 7 + 6 = 16

	return true;
}

boolean MCP2515::setCANNormalMode()
{
    /**
     * Puts the MCP2515 in normal mode
     * 
     * The CANCTRL bits are set as follows:
     *  REQOP2<2:0> = 000 for normal mode
     *  ABAT = 0, do not abort pending transmission
     *  OSM = 0, not one shot
     *  CLKEN = 1, disable output clock
     *  CLKPRE = 0b11, clk/8
     * 
     */
	byte mode;

	writeReg(CANCTRL,0b00000111);
	//Read mode and make sure it is normal
	mode = readReg(CANSTAT) >> 5;
	if(mode != 0) {
		return false;
	}
	
	return true;

}

boolean MCP2515::setCANReceiveonlyMode()
{
    /**
     * Puts the MCP2515 in receive-only (listener) mode
     * 
     * The CANCTRL bits are set as follows:
     *  REQOP2<2:0> = 011 for receive-only mode
     *  ABAT = 0, do not abort pending transmission
     *  OSM = 0, not one shot
     *  CLKEN = 1, disable output clock
     *  CLKPRE = 0b11, clk/8
     * 
     */
	byte mode;

	writeReg(CANCTRL,0b01100111);
	//Read mode and make sure it is receive-only
	mode = readReg(CANSTAT) >> 5;
	if(mode != 3) {
		return false;
	}
	
	return true;

}

boolean MCP2515::setCANLoopbackMode()
{
    /**
     * Puts the MCP2515 in loopback mode
     * 
     * The CANCTRL bits are set as follows:
     *  REQOP2<2:0> = 010 for loopback mode
     *  ABAT = 0, do not abort pending transmission
     *  OSM = 0, not one shot
     *  CLKEN = 1, disable output clock
     *  CLKPRE = 0b11, clk/8
     * 
     * Added by Christopher Meyer July, 2011
     * 
     */

	byte mode;

	writeReg(CANCTRL,0b01000111);
	//Confirm that chip is in loop back
	mode = readReg(CANSTAT) >> 5;
	if(mode != 2) {
		return false;
	}
	
	return true;

}

boolean MCP2515::setReceiveFilter(unsigned int buffer, FILTER0 bufferFilter0,
	FILTER1 bufferFilter1)
{
		
	/**
	 * Set receive filters and mask for both buffers
	 * 
	 * Only handles standard IDs. It can potentially handle all ID types
     * relatively changes but I don't need that support.
	 * 
     * Arguments
     * buffer -- Which receive buffer's filters should be set? Possible values:
     *              BUFFER_0 - Buffer 0
     *              BUFFER_1 - Buffer 1
     *              BUFFER_ALL - Buffer 0 and Buffer 1
     * bufferFilter0 -- FILTER0 item that contains the mask and filters for
     *                  receive buffer 0. If only buffer 1 is being set, 
     *                  this is ignored.
     * bufferFilter1 -- FILTER1 item that contains the mask and filters for
     *                  receive buffer 1. If only buffer 0 is being set, 
     *                  this is ignored.
     * 
     * Returns
     * Boolean indicating whether the given filters were set
     * 
     * Added by Christopher Meyer July, 2011
     * 
	 */

    unsigned short maskAddr;
    unsigned short mask;
    byte filterAddr;
    unsigned short filterCnt;
    unsigned short activeFilters[4];
    byte bufferCtl;
    unsigned short addressOffset;
    unsigned short i;
    
    if (buffer == BUFFER_ALL) {
        if (!setReceiveFilter(BUFFER_0, bufferFilter0, bufferFilter1)) {
            return false;
        }
        buffer = BUFFER_1;
    }
	    
    if (buffer == BUFFER_0) {
        maskAddr = RXM0SIDH;
        mask = bufferFilter0.mask;
        filterAddr = RXF0SIDH;
        filterCnt = 2;
        activeFilters[0] = bufferFilter0.filters[0];
        activeFilters[1] = bufferFilter0.filters[1];
        bufferCtl = RXB0CTRL;
        
    } else {
        maskAddr = RXM1SIDH;
        mask = bufferFilter1.mask;
        filterAddr = RXF2SIDH;
        filterCnt = 4;
        activeFilters[0] = bufferFilter1.filters[0];
        activeFilters[1] = bufferFilter1.filters[1];
        activeFilters[2] = bufferFilter1.filters[2];
        activeFilters[3] = bufferFilter1.filters[3];
        bufferCtl = RXB1CTRL;
    }

    //Set the mask RXM0
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(WRITE); 
	SPI.transfer(maskAddr);
	SPI.transfer(mask >> 3);
	SPI.transfer((mask & 0xFF) << 5);
	digitalWrite(SLAVESELECT,HIGH);
    
	//Verify mask was set
	if (readReg(maskAddr) != (mask >> 3)) {
		return false;
	}
	
	//Set filters
    digitalWrite(SLAVESELECT,LOW);
    SPI.transfer(WRITE); 
    SPI.transfer(filterAddr);
    for (i = 0; i < filterCnt; i++) {
        if ((buffer == BUFFER_1) && (i == 1)) {
            //The filters for buffer 1 are not contiguous
            digitalWrite(SLAVESELECT,HIGH);
            digitalWrite(SLAVESELECT,LOW);
            SPI.transfer(WRITE); 
            SPI.transfer(RXF3SIDH);
        }
        SPI.transfer(activeFilters[i] >> 3);
        SPI.transfer((activeFilters[i] & 0xFF) << 5);
        // The EID filters are unused so just set them to zero
        SPI.transfer(0);
        SPI.transfer(0);
    }
    digitalWrite(SLAVESELECT,HIGH);
	
	// Verify the filters were set
    addressOffset = 0;
	for (i = 0; i < filterCnt; i++) {
        if ((buffer == BUFFER_1) && i == 1) {
            //Add an offset for filters RXF3 and up (buffer 1 filters 1 and up)
            addressOffset = 4;
        }
        
        if (readReg(filterAddr + 4 * i + addressOffset) != 
            (activeFilters[i] >> 3)) {
            return false;
        }
    }
	
	//Set receive operating mode to SID filter mode
	writeRegBit(bufferCtl, RXM1, 0);
	writeRegBit(bufferCtl, RXM0, 1);
	
	//Verify that RXM in the buffer control register was set to filter SIDs
	if ((readReg(bufferCtl) & 0x60) != 0x20) {
		return false;
	}
	
	return true;
}

boolean MCP2515::setOBDFilters()
{
	/**
     * Set the receive filters to only accept data from those in the OBDII 
     * address range, 0x7E8 - 0x7EF.
     * 
     * Returns
     * A boolean indicating whether setting the filters was successful
     * 
     * Added by Christopher Meyer July, 2011
     * 
     */
    FILTER0 obdFilter0;
    FILTER1 obdFilter1;

	//Mask is 0x7F8. Filter will allow 0x7E8 - 0x7EF. Not sure if 7F0 is a 
	//valid address.
    obdFilter0.mask = 0x7F8;
    obdFilter0.filters[0] = 0x7E8;
    obdFilter0.filters[1] = 0x7E8;
    
    obdFilter1.mask = 0x7F8;
    obdFilter1.filters[0] = 0x7E8;
    obdFilter1.filters[1] = 0x7E8;
    obdFilter1.filters[2] = 0x7E8;
    obdFilter1.filters[3] = 0x7E8;
    
	return (setReceiveFilter(BUFFER_ALL, obdFilter0, obdFilter1));
}

boolean MCP2515::receiveCANMessage(CANMSG *msg, unsigned long timeout)
{
    /** 
     * Checks to see if a message exists in either of the receive buffers and 
     * returns the value message in msg
     *
     * When calling provide a CANMSG that will be overwritten if data exists in
     * either buffer. If not data is in the buffer or received in timeout 
     * seconds, the function will return false. Otherwise true will be returned.
     * 
     * Data in Buffer 0 has priority.
     * 
     * Args
     * msg -- A pointer to the CANMSG item where the CAN message will be stored
     * timeout -- The number of milliseconds the function will wait before 
     *              returning a false
     * 
     * Returns
     * A boolean indicating whether data was successfully received.
     * 
     * Added by Christopher Meyer July, 2011. Based on Frank Kienast's 
     * receiveCANMessage, but now uses the Read RX Buffer SPI instructions. 
     * Also, both RX buffers are checked.
     * 
     */

    unsigned long endTime;
    boolean msgInB0;
    boolean msgInB1;
    boolean msgRxd;
    byte readVal;
    int byteIndex;
    
    
    endTime = millis() + timeout;
    msgRxd = false;
    msgInB0 = false;
    msgInB1 = false;
    
    //Wait for data to be received in either buffer
    while (millis() < endTime) {
        readVal = readReg(CANINTF);
        
        if (bitRead(readVal, RX0IF) == 1) {
            msgRxd = true;
            msgInB0 = true;
            break;
        } else if (bitRead(readVal, RX1IF) == 1) {
            msgRxd = true;
            msgInB1 = true;
            break;
        } 
    }
    
    if (msgRxd) {
        //Was this data requested, is Remote Transmission Request (RTR) set?
        if (msgInB0) {
            readVal = readReg(RXB0CTRL);
        } else {
            readVal = readReg(RXB1CTRL);
        }
        msg->rtr = ((bitRead(readVal,3) == 1) ? true : false);
        
        //Start receiveing buffer 0 from the SID high byte
        digitalWrite(SLAVESELECT,LOW);
        
        if (msgInB0) {
            SPI.transfer(READ_RX_BUFFER | READ_RX_0_SIDH);
        } else {
            SPI.transfer(READ_RX_BUFFER | READ_RX_1_SIDH);
        }
        
        //Get the SID
        msg->adrsValue = 0;
        readVal = SPI.transfer(0);
        msg->adrsValue = (readVal << 3);
        readVal = SPI.transfer(0);
        msg->adrsValue |= (readVal >> 5);
        
        //Get EID if it exists
        msg->isExtendedAdrs = ((bitRead(readVal,EXIDE) == 1) ? true : false);
        msg->extendedAdrsValue = 0;
        if (msg->isExtendedAdrs) {
            msg->extendedAdrsValue = (readVal & 0x03) << 16;
            readVal = SPI.transfer(0);
            msg->extendedAdrsValue |= (readVal << 8);
			readVal = SPI.transfer(0);
			msg->extendedAdrsValue |= readVal;
        } else {
            SPI.transfer(0);
            SPI.transfer(0);
        }
        
        //Get the number of bytes received        
        readVal = SPI.transfer(0);
        msg->dataLength = (readVal & 0x0F); 
        if (msg->dataLength > 8) {
            msg->dataLength = 8;
        }
        
        //Read the actual data
        for (byteIndex = 0; byteIndex < msg->dataLength; byteIndex++) {
            msg->data[byteIndex] = SPI.transfer(0);
        }
        
        //End the communication with the chip. No need to resest the RXIF.
        digitalWrite(SLAVESELECT,HIGH);
    }
    
    return msgRxd;

}

boolean MCP2515::transmitCANMessage(CANMSG msg, unsigned long timeout)
{
	unsigned long startTime, endTime;
	boolean sentMessage;
	unsigned short val;
	int i;

	startTime = millis();
	endTime = startTime + timeout;
	sentMessage = false;

	val = msg.adrsValue >> 3;
	writeReg(TXB0SIDH,val);
	val = msg.adrsValue << 5;
	if(msg.isExtendedAdrs) {
		val |= 1 << EXIDE;
	}
	writeReg(TXB0SIDL,val);
	if(msg.isExtendedAdrs) {
		val = msg.extendedAdrsValue >> 8;
		writeReg(TXB0EID8,val);
		val = msg.extendedAdrsValue;
		writeReg(TXB0EID0,val);
	}

	val = msg.dataLength & 0x0f;
	if(msg.rtr) {
		bitWrite(val,TXRTR,1);
	}
	writeReg(TXB0DLC,val);

	//Message bytes
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(WRITE); 
	SPI.transfer(TXB0D0);
	for(i = 0; i < msg.dataLength; i++) {
		SPI.transfer(msg.data[i]);
	}
	digitalWrite(SLAVESELECT,HIGH);

	//Transmit the message
	writeRegBit(TXB0CTRL,TXREQ,1);

	sentMessage = false;
	while(millis() < endTime) {
		val = readReg(CANINTF);
		if(bitRead(val,TX0IF) == 1) {
			sentMessage = true;
			break;
		}
	}

	//Abort the send if failed
	writeRegBit(TXB0CTRL,TXREQ,0);

	//And clear write interrupt
	writeRegBit(CANINTF,TX0IF,0);

	return sentMessage;

}

byte MCP2515::getCANTxErrCnt()
{
	return(readReg(TEC));
}

byte MCP2515::getCANRxErrCnt()
{
	return(readReg(REC));
}

void MCP2515::writeReg(byte regno, byte val)
{
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(WRITE); 
	SPI.transfer(regno);
	SPI.transfer(val);
	digitalWrite(SLAVESELECT,HIGH);  
}

void MCP2515::writeRegBit(byte regno, byte bitno, byte val)
{
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(BIT_MODIFY); 
	SPI.transfer(regno);
	SPI.transfer(1 << bitno);
	if(val != 0)
	SPI.transfer(0xff);
	else
	SPI.transfer(0x00);
	digitalWrite(SLAVESELECT,HIGH);
}

byte MCP2515::readReg(byte regno)
{
	byte val;

	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(READ); 
	SPI.transfer(regno);
	val = SPI.transfer(0);
	digitalWrite(SLAVESELECT,HIGH);

	return val;  
}  

long MCP2515::queryOBD(byte code)
{
    /**
     * Requests the given PID and returns the received value
     * 
     * Returned value may need further formating, e.g. O2 Sensors return two
     * types of values for each request, coolant temperature is 40C less than
     * what is returned.
     * 
     * Arguments
     * code -- PID to be requested
     * 
     * Returns
     * Value received from the vehicle. When the number of returned data 
     * bytes is:
     *      1: A
     *      2: A * 256 + B
     *      3: A * (2^16) + B * (2^8) + C
     *      4: A * (2^24) + B * (2^16) + C * (2^8) + D
     * 
     * Modified by Christopher Meyer July, 2011
     * 
     */
    
	CANMSG msg;
	long val;
	bool rxSuccess;
	unsigned int noMatch;
    unsigned short i;

	msg.adrsValue = 0x7df;
	msg.isExtendedAdrs = false;
	msg.extendedAdrsValue = 0;
	msg.rtr = false;
	msg.dataLength = 8;
	msg.data[0] = 0x02;
	msg.data[1] = 0x01;
	msg.data[2] = code;
	msg.data[3] = 0;
	msg.data[4] = 0;
	msg.data[5] = 0;
	msg.data[6] = 0;
	msg.data[7] = 0;

	if(!transmitCANMessage(msg,1000)) {
		return 0;
	}

	rxSuccess = receiveCANMessage(&msg,1000);
	if (rxSuccess) {
		//Check if the received PID matches the sent PID. Added by C. Meyer
		while(msg.data[2] != code){
			rxSuccess = receiveCANMessage(&msg,1000);
			noMatch++;
			if ((noMatch >= 100) || (rxSuccess = false)){
				return 0;
			}
		}
	} else {
		return 0;
	}
    
	// Convert the received bytes to a single number. Modified by C. Meyer to 
    // read all four bytes.
    val = 0;
    for (i = 3; i <= msg.data[0]; i++) {
        val = (val << 8) | msg.data[i] ;
    }
	
	return val;
}
