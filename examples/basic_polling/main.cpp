#include "main.h"
#include "DS18B20.h"

#define totalSensor 2

UART_HandleTypeDef huart1;

DS18B20 mySensor(&huart1);

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    DS18B20::handleUartTxComplete(huart);
}
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    DS18B20::handleUartRxComplete(huart);
}

int main(void)
{
    uint8_t Addresses[totalSensors][8]={0};
    uint8_t FoundSensors;
    uint8_t Scratch1[8]={0};
    uint8_t Scratch2[8]={0};
    char TempCstring1[12]={0};
    char TempCstring2[12]={0};

    D_Status Status = mySensor.Reset();         //enum class D_Status object for current status
    while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_Reset();

    if(Status == D_Status::OK)      //if last function returs OK as Status, it means that process completed succesfully
    {
        Status = mySensor.SearchRom(Addresses, &FoundSensors, totalSensors);
        while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_SearchRom();


        while(1) {
             
            //initiate temperature read
	        do Status = mySensor.Reset();			//carry out doing this till device gets free from prev transmission
                while(Status == D_Status::BUSY);
	        while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_Reset();
	        Status = mySensor.SkipRom();
	        do Status = mySensor.ConvertTemp();	    //carry out doing till last complete
			    while(Status == D_Status::BUSY);

            HAL_Delay(750); // max conversion time depending on resolution

            //reading a scratchpad
            do Status = mySensor.Reset();			                //carry out doing till last complete
                    while(Status == D_Status::BUSY);
            while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_Reset();
            Status = mySensor.MatchRom(Addresses[0]);
            while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_MatchRom();
            do Status = mySensor.ReadScratchpad(Scratch1);			//carry out doing till last complete
                    while(Status == D_Status::BUSY);
            while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_ReadScratchpad();

            //reading other scratchpad
            do Status = mySensor.Reset();			                //carry out doing till last complete
                    while(Status == D_Status::BUSY);
            while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_Reset();
            Status = mySensor.MatchRom(Addresses[1]);
            while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_MatchRom();
            do Status = mySensor.ReadScratchpad(Scratch2);			//carry out doing till last complete
                    while(Status == D_Status::BUSY);
            while(Status == D_Status::IN_PROGRESS) Status=mySensor.Process_ReadScratchpad();

            //valid Data Extraction
            mySensor.GetStringTempC(mySensor.GetTemperatureC(Scratch1), TempCstring1);
            mySensor.GetStringTempC(mySensor.GetTemperatureC(Scratch2), TempCstring2);
        }
    }
    
}
