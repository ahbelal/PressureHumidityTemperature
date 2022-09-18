/* 
 * File:   pht_ms8607.h
 * Author: a.belal
 *
 * Created on November 3, 2020, 8:31 AM
 */

#ifndef PHT_MS8607_H
#define	PHT_MS8607_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "temperature.h"
#include "pressure.h"
#include "humidity.h"
#include "HardwareProfile.h"
    
#ifdef USE_PHT_MS8607
#warning "Subsystem Temperature: PHT MS8607"
#warning "Subsystem Pressure: PHT MS8607"
#warning "Subsystem Humidity: PHT MS8607"

#define SIZE_OF_PHT_DATA                SIZE_OF_PRESSURE_DATA + SIZE_OF_TEMPERATURE_DATA + SIZE_OF_HUMIDITY_DATA

#define I2C_CLOCK_FREQ              (400000)

//LED
#define LED_TRIS                    TRISEbits.TRISE4
#define LED_LAT                     LATEbits.LATE4

// PHT Constants
#define PT_ADDRESS                  0x76        // 0b1110110 Pressure and Temperature P&T address
#define RH_ADDRESS                  0x40        // 0b1000000 Relative Humidity RH address

#define I2C_x_CHNL                 I2C2         /**< I2C communication channel. */

unsigned char  I2CArrayBuffer[4]; //Buffer for receiving multiple bytes
unsigned short int PTPROM[8]; //PT PROM values

//unsigned short int RHPROM[8]; //RH PROM values

unsigned long long Time;

typedef struct 
{
    int32_t dT;		//INT32 //int dT;
    signed int ActualTemperature; 	//INT32 //int ActualTemperature;
    float TEMPfirstorder;	//signed int TEMPfirstorder;
    float TEMPsecondorder; //signed int TEMPsecondorder;
    float PressureFirstOrder;
    float PressureSecondOrder;
    short int ActualRH;
    float RHcompensated;
} PHT_VAR;

PHT_VAR MyPHT_VAR;

typedef struct
{
    unsigned short int SENS_T1; //Pressure sensitivity
    unsigned short int OFF_T1; //Pressure offset 
    unsigned short int TCS; //Temperature coefficient of pressure sensitivity
    unsigned short int TCO; //Temperature coefficient of pressure offset
    unsigned short int TREF; //Reference temperature 
    uint16_t TEMPSENS; //unsigned short int TEMPSENS; //Temperature coefficient of the temperature
} PHT_CONST;

PHT_CONST MyPHT_CONST;

void InitPht ( void );
void I2CInitiate ( I2C_MODULE PHT_I2C_BUS );
unsigned char SendCommandPHT ( I2C_MODULE PHT_I2C_BUS, unsigned char AddressByte, unsigned char CMDByte );
unsigned char ReceiveDataPHT ( I2C_MODULE PHT_I2C_BUS, unsigned char AddressByte, unsigned char NumberofBytes );
unsigned char crc4_PT( unsigned short int n_prom[] );
int InitializePHT ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte, unsigned char RHAddressByte );
operation_status_t GetTemperatureReading ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte );
operation_status_t GetPressureReading ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte );
operation_status_t GetHumidityReading ( I2C_MODULE PHT_I2C_BUS, unsigned char RHAddressByte );

void ProcessTemperatureDataFeedback ( message_t * pMessage, operation_status_t resultOfOperation );
void ProcessPressureDataFeedback ( message_t * pMessage, operation_status_t resultOfOperation );
void ProcessHumidityDataFeedback ( message_t * pMessage, operation_status_t resultOfOperation );

test_result_t ProcessTemperatureTestFeedback ( message_t * pMessage, operation_status_t resultOfOperation );
test_result_t ProcessPressureTestFeedback ( message_t * pMessage, operation_status_t resultOfOperation );
test_result_t ProcessHumidityTestFeedback ( message_t * pMessage, operation_status_t resultOfOperation );
static operation_status_t TestSensor (message_t* pMessage);
int UpdatePHTReadings (I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte, unsigned char RHAddressByte);


BaseType_t phtInterruptNotifyTail;
BaseType_t phtInterruptNotifyHead;
BaseType_t phtMsgNotifyTail;
BaseType_t phtMsgNotifyHead;

#endif // USE_PHT_MS8607

#ifdef	__cplusplus
}
#endif

#endif	/* PHT_MS8607_H */

