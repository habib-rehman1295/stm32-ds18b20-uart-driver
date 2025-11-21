/*
 * DS18B20.cpp
 *
 *  Created on: Nov, 2025
 *      Author: HABIB_REHMAN
 */

#include "DS18B20.h"

// registry for mapping UART Instances(1,2,3,4) to DS18B20 Object to handle respective RX/TX interrupts

struct UARTonewireRegistry {
    UART_HandleTypeDef* huart;
    DS18B20* instance;
};

static UARTonewireRegistry g_registry[4];		//max no of UARTs supported are 4
static uint8_t g_registryCount = 0;

static DS18B20* FindInstance(UART_HandleTypeDef* huart)
{	//return ptr to DS18B20 instance whose UART raised interrupt
    for (uint8_t i = 0; i < g_registryCount; i++){
        if (g_registry[i].huart == huart)
            return g_registry[i].instance;
    }
    return nullptr;
}
/*********different object of this class should never share the same UART Instance******/
DS18B20::DS18B20(UART_HandleTypeDef* huart) : Uart(huart)
{	//Class created, add RX/TX interrupts and bridge functions call backs in them in main.cpp file
    g_registry[g_registryCount++] = { huart, this };	//store UART instance <--> DS18B20 object
}
DS18B20::~DS18B20() {}

/************************************ROM Commands*******************************************/

D_Status DS18B20::Reset()
{   //function to Reset(followed by Process_Reset) all one wire devices
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
    Rx_Bit=0x00;
    SetBaud(9600);
    setUartTxBusy(true),setUartRxBusy(true);		//set the UART flags
    HAL_UART_Receive_IT(Uart, &Rx_Bit,1);
    HAL_UART_Transmit_IT(Uart, &Tx_Reset, 1);
    return D_Status::IN_PROGRESS;   //followed by a process function
}
D_Status DS18B20::Process_Reset()
{
    //called after Reset() in a loop
    if(uartTxBusy||uartRxBusy) return D_Status::IN_PROGRESS;    //let previous transmission complete
    if(Rx_Bit == Tx_Reset) return D_Status::NO_DEVICE;          //different transmitted values means-device or devices are connected
    SetBaud(115200);                                            //for commands 115200 baudrate is req
    return D_Status::OK;    //DONE
}
D_Status DS18B20::SearchRom(uint8_t addresses[][8], uint8_t *foundDevices, const uint8_t maxDevices)
{   //function to find all addresses((followed by Process_SearchRom)) of connected Devices
/****"D_Status = NO DEVICE" means no device is participating in this Search****/
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;       //let previous transmission complete
    this->deviceAddresses = addresses;
    this->deviceCountPtr  = foundDevices;
    this->maxDeviceCount  = maxDevices;
    currentDevice = 0;
    Last_Discp = 0;
    N_Bit = 0, cmpl_N_Bit = 0;
    this->searchState = SearchState::START_RESET;   //state to initiate search process
    return D_Status::IN_PROGRESS;      //followed by a process function
}
D_Status DS18B20::Process_SearchRom()
{	//called after SearchRom() in a loop
    switch (searchState) {
        case SearchState::START_RESET:
        {
        	if(!uartTxBusy && !uartRxBusy){
                Last_Zero = 0;
                Adr_Bit = 0;							//Adr_Bit is bit index within byte (0–7)
                Adr_Byte = 0;							//Adr_Byte tracks byte index (0–7)
                currentBit = 0;							//currentBit(0–63) keeps track of address BIT being processed
                searchState=SearchState::RESET_WAIT;	//move to next
                return Reset();							//Reset return 1.BUSY /2.IN_PROGRESS
        	}
        	return D_Status::IN_PROGRESS;
        }
        case SearchState::RESET_WAIT:
        {
            D_Status current_status=Process_Reset();       //Process returns 1.IN_PROGRESS /2.NO_DEVICE /3. OK
            if(current_status == D_Status::OK){
                searchState=SearchState::SEND_SEARCH_CMD;  //move to next
                return D_Status::IN_PROGRESS;
            }
            return current_status;      //remain here
        }
        case SearchState::SEND_SEARCH_CMD:
        {
        	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
        	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
            HAL_UART_Transmit_IT(Uart, search_rom, 8);
            searchState = SearchState::READ_N_BIT;			//move to next
            return D_Status::IN_PROGRESS;
        }
        case SearchState::READ_N_BIT:
        {
            if(!uartTxBusy && !uartRxBusy){					//if busy return without updating state
            	setUartTxBusy(true),setUartRxBusy(true);	//set UART flags
                Rx_Bit = 0x00;
                HAL_UART_Receive_IT(Uart, &Rx_Bit, 1);
                HAL_UART_Transmit_IT(Uart, &Tx_Bit_Read, 1);
                searchState = SearchState::READ_cmpl_N_BIT;	//move to next
            }
            return D_Status::IN_PROGRESS;
        }
        case SearchState::READ_cmpl_N_BIT:
        {
            if(!uartTxBusy && !uartRxBusy){					//stay here if UART busy
                N_Bit = Rx_Bit;								//N bit read
                setUartTxBusy(true),setUartRxBusy(true);	//set UART flags
                Rx_Bit = 0x00;
                HAL_UART_Receive_IT(Uart, &Rx_Bit, 1);
                HAL_UART_Transmit_IT(Uart, &Tx_Bit_Read, 1);
                searchState = SearchState::WRITE_BIT;       //move to next
            }
            return D_Status::IN_PROGRESS;
        }
        case SearchState::WRITE_BIT:
        {
            if(!uartTxBusy && !uartRxBusy){      			//stay here if UART busy
                cmpl_N_Bit = Rx_Bit;    					//compliment of N bit read
 /*****address bit logic starts (ref:MAXIM's 1-Wire APPLICATION NOTE 187)*****/
                uint8_t write_Bit;							//Bit to be sent & saved
                if(N_Bit == cmpl_N_Bit)
                {
                	if(N_Bit == ow_1) return D_Status::NO_DEVICE;
                    if((currentBit+1)>Last_Discp){
                        Last_Zero=currentBit+1;
                        write_Bit=0;
                    }
                    else if((currentBit+1)==Last_Discp){
                        write_Bit = 1;
                    }
                    else
                    {
                        write_Bit = (deviceAddresses[currentDevice-1][Adr_Byte] >> Adr_Bit) & 1UL; //prev device address bit copied
                    }
                }
                else
                {
                    write_Bit = (N_Bit == ow_1) ? 1 : 0;
                }
/********MAXIM's algo ends here, now updated bit will be send and saved in address array********/
                if(write_Bit == 1){
                    deviceAddresses[currentDevice][Adr_Byte] |= (1UL << Adr_Bit);	//1 written in address
                	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
                	HAL_UART_Receive_IT(Uart, Rx_Garbage, 1);		//this receive serve no purpose, ignore it
                    HAL_UART_Transmit_IT(Uart, &Tx_Bit_Write1, 1);					//1 sent over one-wire
                }
                else{
                    deviceAddresses[currentDevice][Adr_Byte] &= ~(1UL << Adr_Bit);	//0 written in address
                	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
                	HAL_UART_Receive_IT(Uart, Rx_Garbage, 1);		//this receive serve no purpose, ignore it
                    HAL_UART_Transmit_IT(Uart, &Tx_Bit_Write0, 1);					//0 sent over one-wire
                }
                //following are for looping
                Adr_Byte = (Adr_Bit == 7) ? Adr_Byte + 1 : Adr_Byte;			//Byte++ when Bit=7
                Adr_Bit = (Adr_Bit == 7) ? 0 : Adr_Bit + 1;						//Bit++ after evey iteration
                searchState = (currentBit < 63 ) ? SearchState::READ_N_BIT : SearchState::VERIFY_CRC; //change state if all 64 bits are processed
                currentBit++ ;
            }
            return D_Status::IN_PROGRESS;       //moveS to next
        }
        case SearchState::VERIFY_CRC:
        {
            if(!uartTxBusy && !uartRxBusy){					//stay here if UART busy
                if((CRC8_Maxim(deviceAddresses[currentDevice],7))==deviceAddresses[currentDevice][7])
                {
                    Last_Discp = Last_Zero;					//discrepancies update
                    currentDevice++;						//move to next device
                    *deviceCountPtr = currentDevice;		//no of found devices increment
                    if((currentDevice == maxDeviceCount) | (Last_Discp == 0)) //all devices found | there is no discrepancies(=all found)
                    {
                        searchState = SearchState::DONE;
                        return D_Status::OK;				//Done
                    }
                    else
                    {
                        searchState = SearchState::START_RESET;  //start again for next device
                    }
                }
                else{
                    searchState = SearchState::ERROR;			//CRC ERROR
                }
            }
            return D_Status::IN_PROGRESS;
        }
        case SearchState::ERROR:
        {
            return D_Status::CRC_ERROR;
        }
        default:
        {
            searchState = SearchState::IDLE;
            break;
        }
    }
    return D_Status::ERROR;
}
D_Status DS18B20::ReadRom(uint8_t address[])
{
    //function to read ROM(followed by Process_ReadRom) of a single Device
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
    this->deviceRom = address;
    Adr_Bit = 0;
    Adr_Byte = 0;
    this->readromState = ReadromState::SEND_READROM_CMD;  //state to initiate ReadRom process
    return D_Status::IN_PROGRESS;
}
D_Status DS18B20::Process_ReadRom()
{
    //called after ReadRom() in a loop
    switch (readromState) {
        case ReadromState::SEND_READROM_CMD:
        {
        	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
        	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
            HAL_UART_Transmit_IT(Uart,read_rom, 8);
            readromState = ReadromState::READ_ROM;   //mov to next
            return D_Status::IN_PROGRESS;
        }
        case ReadromState::READ_ROM:
        {
            if(!uartTxBusy && !uartRxBusy){     //stay here if UART busy
                setUartRxBusy(true),setUartTxBusy(true);     //set UART flags
                HAL_UART_Receive_DMA(Uart, Rx_8_Byte, 64);
                HAL_UART_Transmit_DMA(Uart, Tx_Read_Buffer, 64);
                readromState = ReadromState::EXTRACT_ROM;   //move to next
            }
            return D_Status::IN_PROGRESS;
        }
        case DS18B20::ReadromState::EXTRACT_ROM:
        {
            if(!uartTxBusy && !uartRxBusy){     //stay here if UART busy
                for(uint8_t i=0 ; i<64 ; i++)
                {
                    if(Rx_8_Byte[i] == ow_1){
                        deviceRom[Adr_Byte] |= (1UL << Adr_Bit);         //1 written in address
                    }
                    else {
                        deviceRom[Adr_Byte] &= ~(1UL << Adr_Bit);        //0 written in address
                    }
                    //following are for looping
                    Adr_Byte = (Adr_Bit == 7) ? Adr_Byte + 1 : Adr_Byte;        //Byte++ when Bit=7
                    Adr_Bit = (Adr_Bit == 7) ? 0 : Adr_Bit + 1;                 //Bit++ after every iteration
                }
                readromState = ReadromState::VERIFY_CRC; //change state if all 64 bits are processed
            }
            return D_Status::IN_PROGRESS;
        }
        case ReadromState::VERIFY_CRC:
        {
            if((CRC8_Maxim(deviceRom,7))==deviceRom[7])
            {
                readromState = ReadromState::DONE;
                return D_Status::OK;                //Done
            }
            else{
                readromState = ReadromState::ERROR; //CRC ERROR
            }
            return D_Status::IN_PROGRESS;
        }
        case ReadromState::ERROR:
        {
            return D_Status::CRC_ERROR;
        }
        default:
        {
            readromState = ReadromState::IDLE;
            break;
        }
    }
    return D_Status::ERROR;
}
D_Status DS18B20::MatchRom(uint8_t address[])
{
    //function to Match ROM(followed by Process_MatchRom) of a specific Device to address it
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
    this->deviceRom = address;
    Adr_Bit = 0;
    Adr_Byte = 0;
    uint8_t write_Bit = 0;
    for(uint8_t i=0;i<64;i++) //ROM is of 8 bytes= 64 bits
    {   //Tx_8_Byte Buffer is updated according to passed Rom Address Bits
        write_Bit = (deviceRom[Adr_Byte] >> Adr_Bit) & 1UL;    //segregate the bit to send
        Tx_8_Byte[i]= (write_Bit == 1) ? ow_1 : ow_0 ;         //One-Wire high byte for a "1"
        //following are for looping
        Adr_Byte = (Adr_Bit == 7) ? Adr_Byte + 1 : Adr_Byte;        //Byte++ when Bit=7
        Adr_Bit = (Adr_Bit == 7) ? 0 : Adr_Bit + 1;                 //Bit++ after every iteration
    }
    this->matchromState = MatchromState::SEND_MATCHROM_CMD;  //state to initiate ReadRom process
    return D_Status::IN_PROGRESS;
}
D_Status DS18B20::Process_MatchRom()
{
    //called after MatchRom() in a loop
    switch (matchromState) {
        case MatchromState::SEND_MATCHROM_CMD:
        {
        	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
        	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
            HAL_UART_Transmit_IT(Uart,match_rom, 8);
            matchromState = MatchromState::SEND_ADDRESS;   //move to next
            return D_Status::IN_PROGRESS;
        }
        case MatchromState::SEND_ADDRESS:
        {
            if(!uartTxBusy && !uartRxBusy){     //stay here if UART busy
            	setUartRxBusy(true),setUartTxBusy(true);      //set UART flags
                HAL_UART_Receive_DMA(Uart,Rx_Garbage, 64);	  //this receive serve no purpose, ignore it
                HAL_UART_Transmit_DMA(Uart,Tx_8_Byte, 64);
                matchromState = MatchromState::WAIT;   //move to next
            }
            return D_Status::IN_PROGRESS;
        }
        case MatchromState::WAIT:
        {
            if(!uartTxBusy && !uartRxBusy){				//stay here if UART busy
                matchromState = MatchromState::DONE;	//move to next
            }
            return D_Status::IN_PROGRESS;
        }
        case MatchromState::DONE:
        {
            return D_Status::OK;
        }
        default :
        {
            matchromState = MatchromState::IDLE;
            break;
        }
    }
    return D_Status::ERROR;
}
D_Status DS18B20::SkipRom()
{
    //Command function to send Skip Rom cmd to all one wire devices
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
    setUartRxBusy(true),setUartTxBusy(true);				//set UART flags
    HAL_UART_Receive_IT(Uart, Rx_Garbage,8);		//this receive serve no purpose, ignore it
    HAL_UART_Transmit_IT(Uart, skip_rom,8);
    return D_Status::OK;
}
D_Status DS18B20::AlarmSearch(uint8_t addresses[][8], uint8_t *foundDevices, const uint8_t maxDevices)
{   //function to find all devices address
/****"D_Status = NO DEVICE" means no device is participating in this Search****/
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;       //let previous transmission complete
    this->deviceAddresses = addresses;
    this->deviceCountPtr  = foundDevices;
    this->maxDeviceCount  = maxDevices;
    currentDevice = 0;
    Last_Discp = 0;
    N_Bit = 0, cmpl_N_Bit = 0;
    this->alarmsearchState = AlarmSearchState::START_RESET;   //state to initiate search process
    return D_Status::IN_PROGRESS;      //followed by a process function
}
D_Status DS18B20::Process_AlarmSearch()
{
    //called after SearchRom() in a loop
    switch (alarmsearchState) {
        case AlarmSearchState::START_RESET:
        {
            Last_Zero = 0;
            Adr_Bit = 0;							//Adr_Bit is bit index within byte (0–7)
            Adr_Byte = 0;							//Adr_Byte tracks byte index (0–7)
            currentBit = 0;							//currentBit(0–63) keeps track of address BIT being processed
            alarmsearchState=AlarmSearchState::RESET_WAIT;        //move to next
            return Reset();                             //Reset return 1.BUSY /2.IN_PROGRESS
        }
        case AlarmSearchState::RESET_WAIT:
        {
            D_Status current_status=Process_Reset();       //Process returns 1.IN_PROGRESS /2.NO_DEVICE /3. OK
            if(current_status == D_Status::OK){
                alarmsearchState=AlarmSearchState::SEND_SEARCH_CMD;  //mov to next
                return D_Status::IN_PROGRESS;
            }
            return current_status;      //remain here
        }
        case AlarmSearchState::SEND_SEARCH_CMD:
        {
        	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
        	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
            HAL_UART_Transmit_IT(Uart, alarm_search, 8);
            alarmsearchState = AlarmSearchState::READ_N_BIT;   //mov to next
            return D_Status::IN_PROGRESS;
        }
        case AlarmSearchState::READ_N_BIT:
        {
            if(!uartTxBusy && !uartRxBusy){      //if busy return without updating state
                setUartRxBusy(true),setUartTxBusy(true);			//set UART flags
                Rx_Bit = 0x00;
                HAL_UART_Receive_IT(Uart, &Rx_Bit, 1);
                HAL_UART_Transmit_IT(Uart, &Tx_Bit_Read, 1);
                alarmsearchState = AlarmSearchState::READ_cmpl_N_BIT;     //mov to next
            }
            return D_Status::IN_PROGRESS;
        }
        case AlarmSearchState::READ_cmpl_N_BIT:
        {
            if(!uartTxBusy && !uartRxBusy){      //stay here if UART busy
                N_Bit = Rx_Bit;         //N bit read
                setUartRxBusy(true),setUartTxBusy(true);			//set UART flags
                Rx_Bit = 0x00;
                HAL_UART_Receive_IT(Uart, &Rx_Bit, 1);
                HAL_UART_Transmit_IT(Uart, &Tx_Bit_Read, 1);
                alarmsearchState = AlarmSearchState::WRITE_BIT;       //mov to next
            }
            return D_Status::IN_PROGRESS;
        }
        case AlarmSearchState::WRITE_BIT:
        {
            if(!uartTxBusy && !uartRxBusy){      //stay here if UART busy
                cmpl_N_Bit = Rx_Bit;    //compliment of N bit read
                //address bit logic starts (ref:MAXIM's 1-Wire APPLICATION NOTE 187)
                uint8_t write_Bit;      //Bit to be sent & saved
                if(N_Bit == cmpl_N_Bit)
                {
                	if(N_Bit == ow_1) return D_Status::NO_DEVICE;
                    if((currentBit+1)>Last_Discp){
                        Last_Zero=currentBit+1;
                        write_Bit=0;
                    }
                    else if((currentBit+1)==Last_Discp){
                        write_Bit = 1;
                    }
                    else
                    {
                        write_Bit = (deviceAddresses[currentDevice-1][Adr_Byte] >> Adr_Bit) & 1UL; //prev device address bit copied
                    }
                }
                else
                {
                    write_Bit = (N_Bit == ow_1) ? 1 : 0;
                }
                    //MAXIM's algo ends here, now updated bit will be send and saved in address array
                if(write_Bit == 1){
                    deviceAddresses[currentDevice][Adr_Byte] |= (1UL << Adr_Bit);         //1 written in address
                    setUartRxBusy(true),setUartTxBusy(true);			//set UART flag
                    HAL_UART_Receive_IT(Uart, Rx_Garbage, 1);
                    HAL_UART_Transmit_IT(Uart, &Tx_Bit_Write1, 1);    //1 sent over one-wire
                }
                else{
                    deviceAddresses[currentDevice][Adr_Byte] &= ~(1UL << Adr_Bit);        //0 written in address
                    setUartRxBusy(true),setUartTxBusy(true);			//set UART flag
                    HAL_UART_Receive_IT(Uart, Rx_Garbage, 1);
                    HAL_UART_Transmit_IT(Uart, &Tx_Bit_Write0, 1);    //0 sent over one-wire
                }
                //following are for looping
                Adr_Byte = (Adr_Bit == 7) ? Adr_Byte + 1 : Adr_Byte;        //Byte update when Bit=7
                Adr_Bit = (Adr_Bit == 7) ? 0 : Adr_Bit + 1;                 //Bit ++ after evey iteration
                alarmsearchState = (currentBit < 63 ) ? AlarmSearchState::READ_N_BIT : AlarmSearchState::VERIFY_CRC; //change state if all 64 bits are processed
                currentBit++ ;
            }
            return D_Status::IN_PROGRESS;       //mov to next
        }
        case AlarmSearchState::VERIFY_CRC:
        {
            if(!uartTxBusy && !uartRxBusy){       //stay here if UART busy
                if((CRC8_Maxim(deviceAddresses[currentDevice],7))==deviceAddresses[currentDevice][7])
                {
                    Last_Discp = Last_Zero ;            //discp udate
                    currentDevice++;                    //move to next device
                    *deviceCountPtr = currentDevice;    //no of found devices increment
                    if((currentDevice == maxDeviceCount) | (Last_Discp == 0)) //all devices found or there is no discripencies
                    {
                        alarmsearchState = AlarmSearchState::DONE;
                        return D_Status::OK;            //Done
                    }
                    else
                    {
                        alarmsearchState = AlarmSearchState::START_RESET;  //start again for next device
                    }
                }
                else{
                    alarmsearchState = AlarmSearchState::ERROR;       //CRC ERROR
                }
            }
            return D_Status::IN_PROGRESS;
        }
        case AlarmSearchState::ERROR:
        {
            return D_Status::CRC_ERROR;
        }
        default:
        {
            alarmsearchState = AlarmSearchState::IDLE;
            break;
        }
    }
    return D_Status::ERROR;
}

/************************************Functional Commands*******************************************/

D_Status DS18B20::ConvertTemp()
{
    //Command function to initiate Temp conversion, a delay is req after this cmd depending on precision
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
    setUartRxBusy(true),setUartTxBusy(true);				//set UART flags
    HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);				//this receive serve no purpose, ignore it
    HAL_UART_Transmit_IT(Uart, convert_temp, 8);
    return D_Status::OK;
}
D_Status DS18B20::WriteScratchpad(const int8_t TH,const int8_t TL,const uint8_t resolution)
{
    //Function to write(followed by Process_WriteScratchpad) config and TH,TL bits for Alarm,
    //TH and TL are upper and lower limits,resolution = 0,1,2,3 for 0.5, 0.25, 0.125, 0.0625 Precision
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete

    //Tx_8_Byte Buffer is updated according to passed Rom Address Bits
    auto encodeTempByte = [&](int8_t val, uint8_t startIndex)
    {
        uint8_t raw = static_cast<uint8_t>(val);   //casting valid for negative values also(2's complement)
        for (uint8_t i = 0; i < 8; i++){
            Tx_3_Byte[startIndex + i] = ((raw >> i) & 1UL) ? ow_1 : ow_0;
        }
    };
    // Encode TH and TL
    encodeTempByte(TH, 0);   // bits 0–7
    encodeTempByte(TL, 8);   // bits 8–15

    // Encode resolution bits (2 bits → bits 6,7 of 3rd byte)
    for (uint8_t i = 0; i < 2; i++)
    {
        Tx_3_Byte[21+i] = ((resolution >> i) & 1UL) ? ow_1 : ow_0;  //5th and 6th bit of config register
    }
    this->writescratchpadState = WriteScratchpadState::SEND_WRITESCRATCHPAD_CMD;  //state to initiate write ScratchPad process
    return D_Status::IN_PROGRESS;
}
D_Status DS18B20::Process_WriteScratchpad()
{
    // called after WriteScratchpad() in a loop
    switch (writescratchpadState) {
        case WriteScratchpadState::SEND_WRITESCRATCHPAD_CMD:
        {
        	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
        	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
            HAL_UART_Transmit_IT(Uart,write_scratchpad, 8);
            writescratchpadState = WriteScratchpadState::SEND_SCRATCHPAD;   //mov to next
            return D_Status::IN_PROGRESS;
        }
        case WriteScratchpadState::SEND_SCRATCHPAD:
        {
            if(!uartTxBusy && !uartRxBusy){     //stay here if UART busy
                setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
                HAL_UART_Receive_DMA(Uart,Rx_Garbage, 24);		//this receive serve no purpose, ignore it
                HAL_UART_Transmit_DMA(Uart,Tx_3_Byte, 24);
                writescratchpadState = WriteScratchpadState::WAIT;  //mov to next
            }
            return D_Status::IN_PROGRESS;
        }
        case WriteScratchpadState::WAIT:
        {
            if(!uartTxBusy && !uartRxBusy){     //stay here if UART busy
                writescratchpadState = WriteScratchpadState::DONE;   //mov to next
            }
            return D_Status::IN_PROGRESS;
        }
        case WriteScratchpadState::DONE:
        {
            return D_Status::OK;
        }
        default :
        {
            writescratchpadState = WriteScratchpadState::IDLE;
            break;
        }
    }
    return D_Status::ERROR;
}
D_Status DS18B20::ReadScratchpad(uint8_t scratchpad[])
{
    //function to read Scratchpad
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
    this->deviceScratchpad = scratchpad;
    Adr_Bit = 0;
    Adr_Byte = 0;
    this->readscratchpadState = ReadScratchpadState::SEND_READSCRATCHPAD_CMD;  //state to initiate ReadRom process
    return D_Status::IN_PROGRESS;
}
D_Status DS18B20::Process_ReadScratchpad()
{
    // called after ReadScratchpad() in a loop
    switch (readscratchpadState) {
        case ReadScratchpadState::SEND_READSCRATCHPAD_CMD:
        {
        	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
        	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
            HAL_UART_Transmit_IT(Uart,read_scrachpad, 8);
            readscratchpadState = ReadScratchpadState::READ_SCRATCHPAD;   //mov to next
            return D_Status::IN_PROGRESS;
        }
        case ReadScratchpadState::READ_SCRATCHPAD:
        {
            if(!uartTxBusy && !uartRxBusy){     //stay here if UART busy
                setUartTxBusy(true),setUartRxBusy(true);     //set UART flags
                HAL_UART_Receive_DMA(Uart,Rx_9_Byte,72);
                HAL_UART_Transmit_DMA(Uart, Tx_Read_Buffer, 72);
                readscratchpadState = ReadScratchpadState::EXTRACT_SCRATCHPAD;   //mov to next
            }
            return D_Status::IN_PROGRESS;
        }
        case ReadScratchpadState::EXTRACT_SCRATCHPAD:
        {
            if(!uartTxBusy && !uartRxBusy){     //stay here if UART busy
                for(uint8_t i=0 ; i<72 ; i++)
                {
                    if(Rx_9_Byte[i] == ow_1){
                        deviceScratchpad[Adr_Byte] |= (1UL << Adr_Bit);         //1 written in address
                    }
                    else {
                        deviceScratchpad[Adr_Byte] &= ~(1UL << Adr_Bit);        //0 written in address
                    }
                    //following are for looping
                    Adr_Byte = (Adr_Bit == 7) ? Adr_Byte + 1 : Adr_Byte;        //Byte++ when Bit=7
                    Adr_Bit = (Adr_Bit == 7) ? 0 : Adr_Bit + 1;                 //Bit++ after evey iteration
                }
                readscratchpadState = ReadScratchpadState::VERIFY_CRC; //change state if all 72 bits are processed
            }
            return D_Status::IN_PROGRESS;
        }
        case ReadScratchpadState::VERIFY_CRC:
        {
            if((CRC8_Maxim(deviceScratchpad,8))==deviceScratchpad[8])
            {
                readscratchpadState = ReadScratchpadState::DONE;
                return D_Status::OK;                //Done
            }
            else{
                readscratchpadState = ReadScratchpadState::ERROR; //CRC ERROR
            }
            return D_Status::IN_PROGRESS;
        }
        case ReadScratchpadState::ERROR:
        {
            return D_Status::CRC_ERROR;
        }
        default:
        {
            readscratchpadState = ReadScratchpadState::IDLE;
            break;
        }
    }
    return D_Status::ERROR;
}
D_Status DS18B20::CopyScratchpad()
{	//Command function to initiate copy of TH,TL and config reg from Scratchpad to EEPROM
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
    HAL_UART_Transmit_IT(Uart, copy_scrachpad, 8);
    return D_Status::OK;
}
D_Status DS18B20::RecallE2()
{   //Command function to recall the TH,TL and cofig reg from EEPROM to Scratchpad
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
	setUartRxBusy(true),setUartTxBusy(true);		//set UART flags
	HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);		//this receive serve no purpose, ignore it
    HAL_UART_Transmit_IT(Uart, recall_E2, 8);
    return D_Status::OK;
}
D_Status DS18B20::ReadPowerSupply(uint8_t* parasiteMode)
{   //function to check any device running on parasitic power(followed by Process_ReadSupply())
	//returns 0 if no Parasite device
    if(uartTxBusy||uartRxBusy)  return D_Status::BUSY;      //let previous transmission complete
    this->parasiteModePtr = parasiteMode;
    this->readpowersupplyState=ReadPowerSupplyState::SEND_READPOWERSUPPLY_CMD;
    return D_Status::IN_PROGRESS;	//followed by a process function
}
D_Status DS18B20::Process_ReadPowerSupply()
{    //called after SearchRom() in a loop
    switch (readpowersupplyState) {
        case ReadPowerSupplyState::SEND_READPOWERSUPPLY_CMD:
        {
            setUartRxBusy(true),setUartTxBusy(true);				//set UART flags
            HAL_UART_Receive_IT(Uart, Rx_Garbage, 8);				//this receive serve no purpose, ignore it
            HAL_UART_Transmit_IT(Uart, read_powersupply, 8);
            readpowersupplyState = ReadPowerSupplyState::READ_BIT;   //move to next
            return D_Status::IN_PROGRESS;
        }
        case ReadPowerSupplyState::READ_BIT:
        {
            if(!uartTxBusy && !uartRxBusy){      //if busy return without updating state
                setUartTxBusy(true),setUartRxBusy(true);     //set UART flags
                Rx_Bit = 0x00;
                HAL_UART_Receive_IT(Uart, &Rx_Bit, 1);
                HAL_UART_Transmit_IT(Uart, &Tx_Bit_Read, 1);
                readpowersupplyState = ReadPowerSupplyState::CHECK_MODE;     //move to next
            }
            return D_Status::IN_PROGRESS;
        }
        case ReadPowerSupplyState::CHECK_MODE:
        {
            if(!uartTxBusy && !uartRxBusy){      //if busy return without updating state
            	*parasiteModePtr = (Rx_Bit == Tx_Bit_Read)? 0 : 1;     //if same transmission is received, means no parasite
                readpowersupplyState = ReadPowerSupplyState::DONE;     //move to next
            }
            return D_Status::IN_PROGRESS;
        }
        case ReadPowerSupplyState::DONE:
        {
            return D_Status::OK;
        }
        default:
        {
            readpowersupplyState=ReadPowerSupplyState::IDLE;
            break;
        }
    }
    return D_Status::ERROR;
}
/*************************************Helpers functions******************************************/

int32_t DS18B20::GetTemperatureC(const uint8_t* scratchpad) const
{   //output will be in format ±(wwwffff),where w= Whole no, and f= Fractional part
    uint8_t LSB = scratchpad[0] , MSB = scratchpad[1];
    int16_t rawTemp = (static_cast<int16_t>(MSB) << 8) | LSB;
    // Extract resolution bits from configuration register (bits 6 and 5)
    uint8_t config = scratchpad[4];
    uint8_t resBits = (config >> 5) & 0x03;  // 0=9bit, 1=10bit, 2=11bit, 3=12bit
    switch (resBits)
    {
        case 0: rawTemp &= ~0x0007; break;  // 9-bit - clear lowest 3 bits
        case 1: rawTemp &= ~0x0003; break;  // 10-bit - clear lowest 2 bits
        case 2: rawTemp &= ~0x0001; break;  // 11-bit - clear lowest 1 bit
        default: break;                     // 12-bit - no mask
    }
    // Temperature in °C (multiples of 0.0625)
    return static_cast<int32_t>(rawTemp) * 625;
}
int8_t DS18B20::GetTH(const uint8_t* scratchpad) const
{   //upper limit for Alarm condition
    return static_cast<int8_t>(scratchpad[2]);
}
int8_t DS18B20::GetTL(const uint8_t* scratchpad) const
{   //lower limit for Alarm condition
    return static_cast<int8_t>(scratchpad[3]);
}
uint8_t DS18B20::GetResolution(const uint8_t* scratchpad) const
{	// res= 0(9bit), 1(10bit), 2(11bit), 3(12bit)
    uint8_t config = scratchpad[4];
    return ((config >> 5) & 0x03);
}
void DS18B20::GetStringTempC(int32_t tempC, char* stringTempC)
{	//output char Array must be of size= 12, as "±www.ffff C" + /0 format
	char* ptr = stringTempC;
	if (tempC < 0) {	 //if temperature is minus
		*ptr++ = '-';		 //-sign
		tempC = -tempC;	 //absolute value
	}
	uint8_t whole = tempC / 10000;		//whole part is separated
	uint16_t frac  = tempC % 10000;		//fractional part is separated
	char tmp[3];		//temporary buffer and size=3 because max value is "125"
	uint8_t i = 0;
	if (whole == 0) tmp[i++] = '0';
	else {
		while (whole > 0){					//loop till whole part is processed
			tmp[i++] = '0' + (whole % 10);	//ASCII encoding
			whole /= 10;
		}
	}
	while (i--){
		*ptr++ = tmp[i];
	}
	*ptr++ = '.';
	for (uint16_t i = 1000; i > 0; i /= 10) {	//loop till all 4 digits of frac part is processed
		*ptr++ = '0' + (frac / i);				//ASCII encoding
		frac %= i;
	}
	*ptr++ = ' ', *ptr++ = 'C', *ptr++ = '\0';
}

/**************************Bridge functions to be called in UART call backs******************************/
/*
// the call back bridge function must be written in main.cpp or main.c
    extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
        DS18B20::handleUartTxComplete(huart);
    }
    extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
        DS18B20::handleUartRxComplete(huart);
    }
*/
void DS18B20::handleUartTxComplete(UART_HandleTypeDef* huart) {
    // Look up UARTonewireRegistry for object corresponds to this UART Instance
    if (auto inst = FindInstance(huart))
        inst->setUartTxBusy(false);
}
void DS18B20::handleUartRxComplete(UART_HandleTypeDef* huart) {
	// Look up in UARTonewireRegistry for object corresponds to this UART Instance
    if (auto inst = FindInstance(huart))
        inst->setUartRxBusy(false);
}

/*************************************Private member functions******************************************/

void DS18B20::setUartTxBusy(bool state) { uartTxBusy = state; }			//TX flag setter
void DS18B20::setUartRxBusy(bool state) { uartRxBusy  = state; }        //RX flag setter
void DS18B20::SetBaud(uint32_t baud)				//baud rate setter
{
	//set only Baud-rate rest initial parameters of UART are followed
    HAL_UART_DeInit(Uart);
    Uart->Init.BaudRate = baud;
    HAL_UART_Init(Uart);
}
uint8_t DS18B20::CRC8_Maxim(const uint8_t *data, uint8_t length)
{
	//check CRC length must be "-1" than total length
    uint8_t crc = 0;
    while (length--) {
        uint8_t inbyte = *data++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;  // polynomial x^8 + x^5 + x^4 + 1
            inbyte >>= 1;
        }
    }
    return crc;
}


/***Following const data buffers are used to communicate with sensors (LSB) one byte is 1 Bit for One-Wire*****/
const uint8_t DS18B20::Tx_Reset = 0xF0;  //F0h

	//Rom commands
const uint8_t DS18B20::search_rom[8] =   { ow_0, ow_0, ow_0, ow_0, ow_1, ow_1, ow_1, ow_1 };     //F0h
const uint8_t DS18B20::read_rom[8] =     { ow_1, ow_1, ow_0, ow_0, ow_1, ow_1, ow_0, ow_0 };     //33h
const uint8_t DS18B20::match_rom[8] =    { ow_1, ow_0, ow_1, ow_0, ow_1, ow_0, ow_1, ow_0 };     //55h
const uint8_t DS18B20::skip_rom[8] =     { ow_0, ow_0, ow_1, ow_1, ow_0, ow_0, ow_1, ow_1 };     //CCh
const uint8_t DS18B20::alarm_search[8] = { ow_0, ow_0, ow_1, ow_1, ow_0, ow_1, ow_1, ow_1 };     //ECh

	//Functional commands
const uint8_t DS18B20::convert_temp[8] =     { ow_0, ow_0, ow_1, ow_0, ow_0, ow_0, ow_1, ow_0 };	//44h
const uint8_t DS18B20::write_scratchpad[8] = { ow_0, ow_1, ow_1, ow_1, ow_0, ow_0, ow_1, ow_0 };	//4Eh
const uint8_t DS18B20::read_scrachpad[8] =   { ow_0, ow_1, ow_1, ow_1, ow_1, ow_1, ow_0, ow_1 };	//BEh
const uint8_t DS18B20::copy_scrachpad[8] =   { ow_0, ow_0, ow_0, ow_1, ow_0, ow_0, ow_1, ow_0 };	//48h
const uint8_t DS18B20::recall_E2[8] =        { ow_0, ow_0, ow_0, ow_1, ow_1, ow_1, ow_0, ow_1 };	//B8h
const uint8_t DS18B20::read_powersupply[8] = { ow_0, ow_0, ow_1, ow_0, ow_1, ow_1, ow_0, ow_1 };	//B4h

	//Buffers
const uint8_t DS18B20::Tx_Bit_Read = ow_R;			//to read one bit
const uint8_t DS18B20::Tx_Bit_Write0 = ow_0;		//to write 0
const uint8_t DS18B20::Tx_Bit_Write1 = ow_1;		//to write 1
const uint8_t DS18B20::Tx_Read_Buffer[72] = {		//to read Scratchpad
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


