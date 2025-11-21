/*
 * DS18B20.h
 *
 *  Created on: Nov, 2025
 *      Author: HABIB_REHMAN
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_
/*
	In GPIO mode selection the UART RX,TX terminals will be enabled on GPIOs when their respective AF are selected,
	furthermore the RX GPIO Pin works with its default settings in STM32CubeIDE, but the "TX GPIO Pin" should be
	config as "OPEN DRAIN", the other circuit elements are a pull up resistor and a diode,
	connections are as:-
		1. DQ->pullup Resistor,
		2. DQ->RX GPIO Pin,
		3. DQ-> P terminal of Diode
		4. TX GPIO Pin -> N terminal of Diode
        DQ ---- pull-up resistor ---- VDD
        | \
        |  \---- RX pin (AF: UARTx_RX)
        | 
        |----|>|---- TX pin (AF: UARTx_TX, open-drain)
*/

/*
ROM commands

D_Status Reset() / Process_Reset()
D_Status SearchRom(x,x,x) / Process_SearchRom()
D_Status ReadRom(x) / Process_ReadRom()
D_Status MatchRom(x) / Process_MatchRom()
D_Status SkipRom()
D_Status AlarmSearch(x,x,x) / Process_AlarmSearch()

Function commands

D_Status ConvertTemp()
D_Status WriteScratchpad(x,x,x) / Process_WriteScratchpad()
D_Status ReadScratchpad(x) / Process_ReadScratchpad()
D_Status CopyScratchpad()
D_Status RecallE2()
D_Status ReadPowerSupply(x) / Process_ReadPowerSupply()

Helpers

int32_t GetTemperatureC(x)
int8_t GetTH(x)
int8_t GetTL(x)
uint8_t GetResolution(x)
void GetStringTempC(x,x)
*/

//#include "stm32f4xx_hal.h"
#include "main.h"

#define ow_0 0x00
#define ow_1 0xff
#define ow_R 0xff

//DS18B20 class functions return type for operational status
enum class D_Status : uint8_t {
    OK=0,			//operation completed with success
    NO_DEVICE,		//no response or no device
    CRC_ERROR,		//CRC error, user can repeat this task
    BUSY,			//UART is busy with last task
    IN_PROGRESS,	//current task is being carried out
	ERROR			//task is unsuccessful
};

//DS18B20 class
class DS18B20
{	/****these UART Call-Backs must be written in main.cpp****/
	/*extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
        DS18B20::handleUartTxComplete(huart);
    }
	extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
        DS18B20::handleUartRxComplete(huart);
    }*/
    public:
        DS18B20(UART_HandleTypeDef *hUart);
        ~DS18B20();

        D_Status Reset();                                                                       //Primary Function
        D_Status Process_Reset();                                                               //Secondary Function
        D_Status SearchRom(uint8_t addresses[][8], uint8_t *foundDevices,const uint8_t maxDevices);  //Primary Function
        D_Status Process_SearchRom();                                                           //Secondary Function
        D_Status ReadRom(uint8_t address[]);                                                    //Primary Function
        D_Status Process_ReadRom();                                                             //Secondary Function
        D_Status MatchRom(uint8_t address[]);                                                   //Primary Function
        D_Status Process_MatchRom();                                                            //Secondary Function
        D_Status SkipRom();
        D_Status AlarmSearch(uint8_t addresses[][8], uint8_t *foundDevices,const uint8_t maxDevices);//Primary Function
        D_Status Process_AlarmSearch();                                                         //Secondary Function
        D_Status ConvertTemp();
        D_Status WriteScratchpad(const int8_t TH,const int8_t TL,const uint8_t resolution);		//Primary Function
        D_Status Process_WriteScratchpad();                                                     //Secondary Function
        D_Status ReadScratchpad(uint8_t scratchpad[]);											//Primary Function
        D_Status Process_ReadScratchpad();                                                     	//Secondary Function
        D_Status CopyScratchpad();
        D_Status RecallE2();
        D_Status ReadPowerSupply(uint8_t* parasiteMode);										//Primary Function
        D_Status Process_ReadPowerSupply();														//Secondary Function

        int32_t GetTemperatureC(const uint8_t* scratchpad) const;
        int8_t GetTH(const uint8_t* scratchpad) const;
        int8_t GetTL(const uint8_t* scratchpad) const ;
        uint8_t GetResolution(const uint8_t* scratchpad) const;
        void GetStringTempC(int32_t tempC, char* stringTempC);

        static void handleUartTxComplete(UART_HandleTypeDef* hUart);       //call backs dispatcher functions(static because no object is needed)
        static void handleUartRxComplete(UART_HandleTypeDef* hUart);       //call backs dispatcher functions

    private:
        UART_HandleTypeDef* Uart;  				// store the UART used by this handle
        	//UART status flag
        volatile bool uartTxBusy = false;
        volatile bool uartRxBusy = false;
 /***Following const data buffers are used to communicate with sensors (LSB) one byte is 1 Bit for One-Wire*****/

        	//Rom commands
        static const uint8_t Tx_Reset;			//F0h
        static const uint8_t search_rom[8];		//F0h
        static const uint8_t read_rom[8];   	//33h
        static const uint8_t match_rom[8];		//55h
        static const uint8_t skip_rom[8];		//CCh
        static const uint8_t alarm_search[8];	//ECh

            //Functional commands
        static const uint8_t convert_temp[8];		//44h
        static const uint8_t write_scratchpad[8];	//4Eh
        static const uint8_t read_scrachpad[8];		//BEh
        static const uint8_t copy_scrachpad[8];		//48h
        static const uint8_t recall_E2[8];			//B8h
        static const uint8_t read_powersupply[8];	//B4h

            //Buffers
        static const uint8_t Tx_Bit_Read;			//to read one bit
        static const uint8_t Tx_Bit_Write0;			//to write 0
        static const uint8_t Tx_Bit_Write1;			//to write 1
        static const uint8_t Tx_Read_Buffer[72];	//to read Scratchpad
        uint8_t Tx_3_Byte[24] =  {0};				//to write Scratch after updating
        uint8_t Tx_8_Byte[64] =  {0};				//to write ROM after updating
        uint8_t Rx_Bit = 0;
        uint8_t Rx_Garbage[64]={ 0 };				//this buffer store in this buffer is useless,used to compliment
        											//Tx operation while doing, Bit read, send cmd, send 3 bytes of
        											//scratch pad and send ROM.
        uint8_t Rx_8_Byte[64] = { 0 };				//store ROM
        uint8_t Rx_9_Byte[72] = { 0 };				//store Scratchpad

        uint8_t CRC8_Maxim(const uint8_t *data, uint8_t length);    //verify CRC
        void setUartRxBusy(bool state);        //flag setter
        void setUartTxBusy(bool state);        //flag setter
        void SetBaud(uint32_t baud);

        uint8_t currentBit;			//address bit under process
        uint8_t Adr_Bit;            //Address bit(0-7) in a address byte
        uint8_t Adr_Byte;           //Address byte(0-7) in a address aray

        //Search_ROM / Alarm_Search
        uint8_t (*deviceAddresses)[8] = nullptr;  // pointer to external address array
        uint8_t *deviceCountPtr = nullptr;        // pointer to external counter
        uint8_t maxDeviceCount;     // max allowed (e.g. 5)
        uint8_t currentDevice;      //device whose address is under process
        uint8_t Last_Discp;         //discrepancy
        uint8_t Last_Zero;          //zero
        uint8_t N_Bit;              //first bit
        uint8_t cmpl_N_Bit;         //complement of First bit

        //Read_ROM / Match_ROM
        uint8_t *deviceRom = nullptr;
        //Read_Scratchpad
        uint8_t *deviceScratchpad = nullptr;
        //Read_Powersupply
        uint8_t *parasiteModePtr = nullptr;     //pointer to power-supply type flag

        enum class SearchState : uint8_t{
            IDLE = 0, START_RESET, RESET_WAIT, SEND_SEARCH_CMD,
            READ_N_BIT, READ_cmpl_N_BIT, WRITE_BIT,
            VERIFY_CRC, NEXT_DEVICE, DONE, ERROR
        } searchState = SearchState::IDLE;

        enum class ReadromState : uint8_t{
            IDLE = 0, SEND_READROM_CMD, READ_ROM,
            EXTRACT_ROM, VERIFY_CRC, DONE, ERROR
        } readromState = ReadromState::IDLE;

        enum class MatchromState : uint8_t{
            IDLE = 0, SEND_MATCHROM_CMD,
            SEND_ADDRESS, WAIT, DONE
        } matchromState = MatchromState::IDLE;

        enum class AlarmSearchState : uint8_t{
            IDLE = 0, START_RESET, RESET_WAIT, SEND_SEARCH_CMD,
            READ_N_BIT, READ_cmpl_N_BIT, WRITE_BIT,
            VERIFY_CRC, NEXT_DEVICE, DONE, ERROR
        } alarmsearchState = AlarmSearchState::IDLE;

        enum class WriteScratchpadState : uint8_t{
            IDLE = 0, SEND_WRITESCRATCHPAD_CMD,
            SEND_SCRATCHPAD, WAIT, DONE
        } writescratchpadState = WriteScratchpadState::IDLE;

        enum class ReadScratchpadState : uint8_t{
            IDLE = 0, SEND_READSCRATCHPAD_CMD,
            READ_SCRATCHPAD, EXTRACT_SCRATCHPAD,
            VERIFY_CRC, DONE, ERROR
        } readscratchpadState = ReadScratchpadState::IDLE;

        enum class ReadPowerSupplyState : uint8_t{
            IDLE = 0, SEND_READPOWERSUPPLY_CMD,
            READ_BIT, CHECK_MODE, DONE
        } readpowersupplyState = ReadPowerSupplyState::IDLE;
};

#endif /* INC_DS18B20_H_ */
