
#include "temperature.h"
#include "pressure.h"
#include "humidity.h"

#ifdef USE_PHT_MS8607

#include "pht_ms8607.h"
#include "HelperFunctions.h"
#include "subsystem.h"

/* Macro Definitions */

/* Data Types */

/* Constants */

/* Static Variable Declaration */

/* Interrupts */

/* Public Function Body */
portTASK_FUNCTION ( TemperatureTask, pParams )
{
    /* Variables */
    operation_status_t resultOfOperation ;
    BaseType_t result;
    TickType_t ticksBlockTime ;
    message_t * pMessage ;
    uint32_t recvItem ;
    task_info_t * pTaskInfo ;

    /* Variables Init */
    result = 0;
    resultOfOperation = NO_OPERATION_STATUS;
    ticksBlockTime = portMAX_DELAY ;
    pMessage = NULL ;
    phtMsgNotifyTail = 0 ;
    phtMsgNotifyHead = 0 ;
    phtInterruptNotifyTail = configTASK_NOTIFICATION_ARRAY_ENTRIES / 2 ;
    phtInterruptNotifyHead = configTASK_NOTIFICATION_ARRAY_ENTRIES / 2 ;
    
    /* PHT sensor Initialize Function */
    InitPht ( ) ;

    /* Task Core */
    for ( ; ; )
    {
        /* 
         * Block indefinitely (without a timeout, so no need to check the functionâ€™s
         * return value) to wait for a notification.
         * Bits in this RTOS taskâ€™s notification value are set by the notifying
         * tasks and interrupts to indicate which events have occurred.         
         */
        result = xTaskNotifyWaitIndexed ( phtMsgNotifyHead, // Wait for 0th notification.
                                          0x00, // Donâ€™t clear any notification bits on entry.
                                          ULONG_MAX, // Reset the notification value to 0 on exit.
                                          &recvItem, // Notified value pass out in ulNotifiedValue.
                                          ticksBlockTime ) ; // Block indefinitely. 
        if( result == SUCCESSFUL_OPERATION )
        {
            result = ConsumeTemperatureMessage ( result ) ;
        }
        else
        {
            Nop();
        }

        pMessage = ( message_t * ) recvItem ;
        pTaskInfo = ( task_info_t * ) pMessage->info;
        resultOfOperation = UNKNOWN_ERROR ;
                
        switch ( pMessage->body.command )
        {
            case GET_COMMAND:
                switch ( pMessage->body.peripheralFunction )
                {
                    case TEMPERATURE:
                        resultOfOperation = GetTemperature ( pMessage ) ;
                        resultOfOperation |= AllocateMessageData ( pMessage, SIZE_OF_TEMPERATURE_DATA );
                        ProcessTemperatureDataFeedback ( pMessage, resultOfOperation );                             
                        break;

                    case PRESSURE:
                        resultOfOperation = GetPressure ( pMessage ) ;
                        resultOfOperation |= AllocateMessageData ( pMessage, SIZE_OF_PRESSURE_DATA );
                        ProcessPressureDataFeedback ( pMessage, resultOfOperation );                             
                        break;

                    case HUMIDITY:
                        resultOfOperation = GetHumidity ( pMessage ) ;
                        resultOfOperation |= AllocateMessageData ( pMessage, SIZE_OF_HUMIDITY_DATA );
                        ProcessHumidityDataFeedback ( pMessage, resultOfOperation );                                                     
                        break;

                    case ALL_STATUS:
                        resultOfOperation = GetHumidity ( pMessage ) ;
                        resultOfOperation |= AllocateMessageData ( pMessage, SIZE_OF_TEMPERATURE_DATA + SIZE_OF_PRESSURE_DATA + SIZE_OF_HUMIDITY_DATA );
                        ProcessPressureDataFeedback ( pMessage, resultOfOperation );                             
                        ProcessHumidityDataFeedback ( pMessage, resultOfOperation );                                                     
                        ProcessTemperatureDataFeedback ( pMessage, resultOfOperation );                             
                        break;
                }
                break ;

            case TEST_COMMAND:
                resultOfOperation = TestSensor ( pMessage );
                if ( SUCCESSFUL_OPERATION == AllocateMessageData ( pMessage, 3 * SIZE_OF_TEST_RESULTS ))
                {
                    ProcessTemperatureTestFeedback ( pMessage, resultOfOperation );                             
                    ProcessPressureTestFeedback ( pMessage, resultOfOperation );                             
                    ProcessHumidityTestFeedback ( pMessage, resultOfOperation );  
                    resultOfOperation = SUCCESSFUL_OPERATION;                    
                }
                else
                {
                    resultOfOperation = HEAP_ERROR;                    
                }
                break ;
                
            default:
                break ;
        }
        
        
        if ( NULL != pTaskInfo )
        {
            if ( NULL != pTaskInfo->handle )
            {
                // Handle feedback
                taskENTER_CRITICAL();
                result = xTaskNotifyIndexed (   pTaskInfo->handle, *pTaskInfo->pIndex, 
                                                ( resultOfOperation << SHIFT_3_BYTE ) | 
                                                TEMPERATURE_TASK_BITS, eSetBits );
                resultOfOperation = IsNotifyValid ( result, pTaskInfo->pIndex );
                // Free ( pTaskInfo );
                taskEXIT_CRITICAL();   
            }
            else
            {
            
            }
        }
        else
        {
            
        }
    }
}


void InitPht ( void )
{
    // Initialize I2C channel
    I2CInitiate ( I2C_x_CHNL );
    
    // Initialize PHT sensor 
    InitializePHT ( I2C_x_CHNL, PT_ADDRESS, RH_ADDRESS );
}

operation_status_t GetTemperature (message_t* pMessage)
{
    return GetTemperatureReading (I2C_x_CHNL, PT_ADDRESS);
}

/*********************************************************************
 * Function:       	int GetTemperatureReading ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte )
 *
 * PreCondition:    Declaring SendCommandPHT and ReceiveDataPHT functions
 *                  using InitializePHT function
 *                  Declaring MyPHT_VAR struct
 *
 * Input:           I2C_MODULE PHT_I2C_BUS:
 *                  unsigned char PTAddressByte: PHT sensor PT Address
 *
 * Output:          0 if success
 *                  -1 if failed
 *
 * Side Effects:    None
 *
 * Overview:        This function update temperature values at MyPHT_VAR struct 
 *
 * Note:            None
 ********************************************************************/

operation_status_t GetTemperatureReading ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte )
{
    unsigned char             success = 1;
//    float                     T2 = 0;
	int64_t T2 = 0;
    unsigned int              DigitalTemperature = 0; //Digital temperature value | (unsigned int 32 D2)
    
    //TMIN = -40�C TMAX = 85�C TREF = 20�C    
//    int Tmin = -40, Tmax = 85;
    
    //Read digital temperature data
    //Digital temperature value (unsigned int D2)
    //conversion command sequence 0b01011010 OSR (8192)
    /*
     *********(I2C command to initiate a temperature conversion (OSR=8192, typ=D2))*********
     */
    success = SendCommandPHT ( PHT_I2C_BUS, PTAddressByte, 0x5A ); //conversion command sequence 0b01011010 OSR (8192)
    vTaskDelay ( pdMS_TO_TICKS (20 ) ); //18
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    
    /*
     *********(I2C ADC read sequence)*********
     */
    success = SendCommandPHT ( PHT_I2C_BUS, PTAddressByte, 0x00);     //ADC read command sequence
    //delayMS(20);
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    
    /*
    *********(I2C answer from the ASIC)*********
    */
    success = ReceiveDataPHT ( PHT_I2C_BUS, PTAddressByte, 3 );     //I2C answer from the ASIC 3 Bytes
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    
    //Combine the data into one variable  
    DigitalTemperature |= (I2CArrayBuffer[0]<<16) | (I2CArrayBuffer[1]<<8) | (I2CArrayBuffer[2]) ;
//    DigitalTemperature = 8077636; //for testing purposes
    
    //Calculate temperature
    //Difference between actual and reference temperature
    //int dT = D2 - TREF = D2 - C5 * 2^8
    MyPHT_VAR.dT = DigitalTemperature - ((int32_t)MyPHT_CONST.TREF <<8);	//(MyPHT_CONST.TREF * (1<<8));
    
    //Actual temperature (-40?85�C with 0.01�C resolution)
    //int TEMP = 20�C + dT *TEMPSENS = 2000 + dT * C6 / 2^23
    MyPHT_VAR.ActualTemperature = 2000 + ((int64_t)MyPHT_VAR.dT * (int64_t)MyPHT_CONST.TEMPSENS >>23);	//MyPHT_CONST.TEMPSENS / (1<<23);
//    MyPHT_VAR.TEMPfirstorder = (float)MyPHT_VAR.ActualTemperature/100.0;
	MyPHT_VAR.TEMPfirstorder = (int64_t)(MyPHT_VAR.ActualTemperature/100);

    /*
    *********(SECOND ORDER COMPENSATION)*********
    */
//    if (MyPHT_VAR.TEMPfirstorder < 20.0) //Low temperature
    if(MyPHT_VAR.ActualTemperature < 2000)
    {
        //T2 = 3 * dT^2 / 2^33
        //T2 = (float) 3 * pow(MyPHT_VAR.dT, 2) / ((long long)1<<33);
		T2 = ( 3 * ( (int64_t)MyPHT_VAR.dT  * (int64_t)MyPHT_VAR.dT  ) ) >> 33;
        Nop();
	    if(MyPHT_VAR.ActualTemperature < -1500)
   		{
			
		}
    }
    else //High temperature
    {
        //T2 = 5 * dT^2 / 2^38
        //T2 = (float) 5 * pow(MyPHT_VAR.dT, 2) / ((long long)1<<38);
		T2 = ( 5 * ( (int64_t)MyPHT_VAR.dT  * (int64_t)MyPHT_VAR.dT  ) ) >> 38;
        Nop();
    }
    MyPHT_VAR.TEMPsecondorder = (int64_t) MyPHT_VAR.ActualTemperature - T2;    //(float) MyPHT_VAR.ActualTemperature - T2;
//	MyPHT_VAR.TEMPsecondorder = (float) (()MyPHT_VAR.ActualTemperature - T2;
	MyPHT_VAR.TEMPsecondorder = (float) MyPHT_VAR.TEMPsecondorder;
    MyPHT_VAR.TEMPsecondorder /= 100.0;
	if(MyPHT_VAR.TEMPsecondorder > 85.00)
    {
		MyPHT_VAR.TEMPsecondorder = 85.00;
	}
	else if (MyPHT_VAR.TEMPsecondorder < -40.00)
    {
		MyPHT_VAR.TEMPsecondorder = -40.00;
	}
    Nop();
    return SUCCESSFUL_OPERATION;
}

test_result_t ProcessTemperatureTestFeedback ( message_t * pMessage, operation_status_t resultOfOperation )
{                    
    test_result_t resultOfTest ;
    
    if ( SUCCESSFUL_OPERATION == resultOfOperation )
    {
        if( HIGH_TEMPERATURE_WARNING_LIMIT >= MyPHT_VAR.TEMPsecondorder )
        {
            resultOfTest = PASS;
        }
        else
        {
            resultOfTest = FAIL;            
        } 
        pMessage->data[pMessage->body.dataLength++] = TEMPERATURE;
        pMessage->data[pMessage->body.dataLength++] = resultOfTest;
    }
    else
    {
        resultOfTest = FAIL;
    }
    
    return resultOfTest;
}

void ProcessTemperatureDataFeedback ( message_t * pMessage, operation_status_t resultOfOperation )
{                    
    float_t temperature ;
    
    if ( SUCCESSFUL_OPERATION == resultOfOperation )
    {
        temperature.DataFloat = MyPHT_VAR.TEMPsecondorder;
        memcpy(pMessage->data + pMessage->body.dataLength, temperature.Data, sizeof(float));
        pMessage->body.dataLength += sizeof(float);
    }
    else
    {
    }
}

operation_status_t AppendTemperatureMessage ( BaseType_t resultOfOperation )
{    
    return IsNotifyValid ( resultOfOperation, &phtMsgNotifyTail );    
}

operation_status_t ConsumeTemperatureMessage ( BaseType_t resultOfOperation )
{
    return IsNotifyValid ( resultOfOperation, &phtMsgNotifyHead );    
}

operation_status_t AppendTemperatureInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &phtInterruptNotifyTail );
}

operation_status_t ConsumeTemperatureInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &phtInterruptNotifyHead );    
}

BaseType_t GetTemperatureMessageTail ( ) 
{
    return phtMsgNotifyTail ;
}

operation_status_t GetPressure (message_t * pMessage)
{
    GetTemperatureReading (I2C_x_CHNL, PT_ADDRESS );
    return GetPressureReading (I2C_x_CHNL, PT_ADDRESS );
}

/*********************************************************************
 * Function:       	int GetPressureReading ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte )
 *
 * PreCondition:    Declaring SendCommandPHT and ReceiveDataPHT functions
 *                  using GetTemperatureReading function
 *                  Declaring MyPHT_VAR struct
 *
 * Input:           I2C_MODULE PHT_I2C_BUS:
 *                  unsigned char PTAddressByte: PHT sensor PT Address
 *
 * Output:          SUCCESSFUL_OPERATION if success
 *                  TIMEOUT_ERROR if failed
 *
 * Side Effects:    None
 *
 * Overview:        This function update pressure values in MyPHT_VAR struct
 *
 * Note:            None
 ********************************************************************/

operation_status_t GetPressureReading ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte )
{
    unsigned char                success = 1;
    signed int                   PRSSURE = 0;
    unsigned int                 DigitalPressure = 0; //Digital pressure value | (unsigned int 32 D1)
//    signed long long             OFF = 0, SENS = 0, OFF2 = 0, SENS2 = 0; 
    int64_t             OFF = 0, SENS = 0, OFF2 = 0, SENS2 = 0; 
    
    //Offset at actual temperature | (signed int 64)
    //OFF = C2 * 2^17 + (C4 * dT ) / 2^6
//    OFF = (long long) MyPHT_CONST.OFF_T1*(1<<17)+(MyPHT_CONST.TCO*MyPHT_VAR.dT)/(1<<6);
    OFF = ((int64_t) MyPHT_CONST.OFF_T1) *(1<<17) + ((int64_t)MyPHT_CONST.TCO*MyPHT_VAR.dT)/(1<<6);
    //Sensitivity at actual temperature | (signed int 64)
    //SENS = C1 * 2^16 + (C3 * dT ) / 2^7
//    SENS = (long long) MyPHT_CONST.SENS_T1*(1<<16)+(MyPHT_CONST.TCS*MyPHT_VAR.dT)/(1<<7);
    SENS = ((int64_t) MyPHT_CONST.SENS_T1)*(1<<16) + ((int64_t)MyPHT_CONST.TCS*MyPHT_VAR.dT)/(1<<7);
    Nop();
    
    //Read digital pressure data
    //Digital pressure value (unsigned int D1)
    //conversion command sequence 0b01001010 OSR (8192)
    /*
     *********(I2C command to initiate a pressure conversion (OSR=8192, typ=D1))*********
     */
    success = SendCommandPHT ( PHT_I2C_BUS, PTAddressByte, 0x4A ); //conversion command sequence 0b01011010 OSR (8192)
    vTaskDelay ( pdMS_TO_TICKS (20 ) ); //18
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    
    /*
     *********(I2C ADC read sequence)*********
     */
    success = SendCommandPHT ( PHT_I2C_BUS, PTAddressByte, 0x00);     //ADC read command sequence
    //delayMS(20);
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    
    /*
    *********(I2C answer from the ASIC)*********
    */
    success = ReceiveDataPHT ( PHT_I2C_BUS, PTAddressByte, 3 );     //I2C answer from the ASIC 3 Bytes
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    
    //Combine the data into one variable  
    DigitalPressure |= (I2CArrayBuffer[0]<<16) | (I2CArrayBuffer[1]<<8) | (I2CArrayBuffer[2]) ;
//    DigitalPressure = 6465444; //for testing purposes
    Nop();
    
    //Temperature compensated pressure
    //(10?1200 mbar with 0.01mbar resolution)
    //P = (D1 * SENS / 2^21 - OFF) / 2^15
    PRSSURE = (DigitalPressure * SENS / (1<<21) - OFF) / (1<<15);
    MyPHT_VAR.PressureFirstOrder = PRSSURE/100.0;
    
    /*
    *********(SECOND ORDER COMPENSATION)*********
    */
//    MyPHT_VARS.ActualTemperature = -1600; //for testing purposes
//    MyPHT_VARS.TEMPfirstorder = MyPHT_VARS.ActualTemperature/100; //for testing purposes
//    if (MyPHT_VAR.TEMPfirstorder < 20.0) //Low temperature
//    {
//        //OFF2 = 61 *(TEMP ? 2000)^2 / 2^4
//        OFF2 = (long long) 61 * pow((MyPHT_VAR.ActualTemperature - 2000), 2)/ (1<<4);
//        //SENS2 = 29 *(TEMP ? 2000)^2/ 2^4
//        SENS2 = (long long) 29 * pow((MyPHT_VAR.ActualTemperature - 2000), 2)/ (1<<4);
//        Nop();
//        if (MyPHT_VAR.TEMPfirstorder < -15.0) //Very low temperature
//        {
//            //OFF2 = OFF2 + 17 * (TEMP + 1500)^2
////            OFF2 = (long long) OFF2 + 17 * pow((MyPHT_VAR.ActualTemperature + 1500), 2);
//            OFF2 = (long long) OFF2 + 17 * pow((MyPHT_VAR.ActualTemperature + 1500), 2);
//
//            //SENS2 = SENS2 + 9 * (TEMP + 1500)^2
//            SENS2 = (long long) SENS2 + 9 * pow((MyPHT_VAR.ActualTemperature + 1500), 2);
//            Nop();
//        }
//    }
//    else //High temperature
//    {
//        OFF2 = 0;
//        SENS2 = 0;
//    }
    
	if( MyPHT_VAR.ActualTemperature < 2000 )
	{
		OFF2 = 61 * ((int64_t)MyPHT_VAR.ActualTemperature - 2000) * ((int64_t)MyPHT_VAR.ActualTemperature - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)MyPHT_VAR.ActualTemperature - 2000) * ((int64_t)MyPHT_VAR.ActualTemperature - 2000) / 16 ;
		
		if( MyPHT_VAR.ActualTemperature < -1500 )
		{
			OFF2 += 17 * ((int64_t)MyPHT_VAR.ActualTemperature + 1500) * ((int64_t)MyPHT_VAR.ActualTemperature + 1500) ;
			SENS2 += 9 * ((int64_t)MyPHT_VAR.ActualTemperature + 1500) * ((int64_t)MyPHT_VAR.ActualTemperature + 1500) ;
		}
	}
	else
	{
		OFF2 = 0 ;
		SENS2 = 0 ;
	}

    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    //P = (D1 * SENS / 2^21 - OFF) / 2^15
    PRSSURE = (DigitalPressure * SENS / (1<<21) - OFF) / (1<<15);
    MyPHT_VAR.PressureSecondOrder = PRSSURE/100.0;

	if(MyPHT_VAR.PressureSecondOrder > 2000.00)
    {
		MyPHT_VAR.PressureSecondOrder = 2000.00;
	}
	else if(MyPHT_VAR.PressureSecondOrder < 10.00)
    {
		MyPHT_VAR.PressureSecondOrder = 10.00;
	}
    Nop();
    return SUCCESSFUL_OPERATION;
}

test_result_t ProcessPressureTestFeedback ( message_t * pMessage, operation_status_t resultOfOperation )
{                 
    test_result_t resultOfTest; 

    resultOfTest = FAIL;
    
    if ( SUCCESSFUL_OPERATION == resultOfOperation )
    {
        if( HIGH_PRESSURE_WARNING_LIMIT >= MyPHT_VAR.PressureSecondOrder )
        {
            resultOfTest = PASS;
        }
        else
        {
            resultOfTest = FAIL;            
        } 
        pMessage->data[pMessage->body.dataLength++] = PRESSURE;
        pMessage->data[pMessage->body.dataLength++] = resultOfTest;
    }
    else
    {
        resultOfTest = FAIL;
    }
    
    return resultOfTest ;
}

void ProcessPressureDataFeedback ( message_t * pMessage, operation_status_t resultOfOperation )
{                 
    float_t pressure;
    
    if ( SUCCESSFUL_OPERATION == resultOfOperation )
    {
        pressure.DataFloat = MyPHT_VAR.PressureSecondOrder;
        memcpy ( pMessage->data + pMessage->body.dataLength, pressure.Data, sizeof(float) );
        pMessage->body.dataLength += sizeof(float);
    }
    else
    {
    }
}

operation_status_t AppendPressureMessage ( BaseType_t resultOfOperation )
{    
    return IsNotifyValid ( resultOfOperation, &phtMsgNotifyTail );    
}

operation_status_t ConsumePressureMessage ( BaseType_t resultOfOperation )
{
    return IsNotifyValid ( resultOfOperation, &phtMsgNotifyHead );    
}

operation_status_t AppendPressureInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &phtInterruptNotifyTail );
}

operation_status_t ConsumePressureInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &phtInterruptNotifyHead );    
}

BaseType_t GetPressureMessageTail ( ) 
{
    return phtMsgNotifyTail ;
}

operation_status_t GetHumidity (message_t* pMessage)
{
    GetTemperatureReading (I2C_x_CHNL, PT_ADDRESS);
    GetPressureReading (I2C_x_CHNL, PT_ADDRESS);
    return GetHumidityReading (I2C_x_CHNL, RH_ADDRESS);
}

/*********************************************************************
 * Function:       	int GetHumidity ( I2C_MODULE PHT_I2C_BUS, unsigned char RHAddressByte )
 *
 * PreCondition:    Declaring SendCommandPHT and ReceiveDataPHT functions
 *                  using GetTemperatureReading function
 *                  Declaring MyPHT_VAR struct
 *
 * Input:           I2C_MODULE PHT_I2C_BUS:
 *                  unsigned char RHAddressByte: PHT sensor RH Address
 *
 * Output:          0 if success
 *                  -1 if failed
 *
 * Side Effects:    None
 *
 * Overview:        This function update humidity value 
 *
 * Note:            None
 ********************************************************************/
operation_status_t GetHumidityReading ( I2C_MODULE PHT_I2C_BUS, unsigned char RHAddressByte ) 
{
    unsigned char       success = 1;
    unsigned short int  DigitalRH = 0;
    float               Tcoeff = -0.18;
    //RHMIN = -6 %RH RHMAX= 118 %RH
//    int RHMIN = -6;
//    int RHMAX= 118;
    //Reset Command 0b11111110
//    success = SendCommandPHT( RH_ADDRESS, 0xFE );    
//    if (!success)
//    {
//        return TIMEOUT_ERROR; //Error
//    }
    
    //Read digital relative humidity data 0b11100101 (Hold master)
    success = SendCommandPHT( PHT_I2C_BUS, RHAddressByte, 0xE5 ); 
    //delayMS(20);
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    
    success = ReceiveDataPHT( PHT_I2C_BUS, RHAddressByte, 3 );
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    Nop();
    
    //Digital relative humidity value (unsigned int D3)
    DigitalRH |= (I2CArrayBuffer[0]<<8) | (I2CArrayBuffer[1]) ;
//    DigitalRH = 31872; //for testing purposes
    
    //Calculate relative humidity
    //Actual relative humidity (-6 %RH?118%RH with 0.01 %RH resolution)
    //int RH = - 600 + 12500 * D3 / 2^16
    MyPHT_VAR.ActualRH = -600 + 12500 * DigitalRH / (1<<16);
    
    //The final compensated relative humidity value
    //int RHcompensated = RH + (20 - TEMP) * Tcoeff
    MyPHT_VAR.RHcompensated = (float)MyPHT_VAR.ActualRH/100 + ((float)20 - MyPHT_VAR.TEMPfirstorder) * Tcoeff;
	if(MyPHT_VAR.RHcompensated < 0.00)
    {
		MyPHT_VAR.RHcompensated = 0.00;
	}
	else if(MyPHT_VAR.RHcompensated > 100.00)
    {
		MyPHT_VAR.RHcompensated = 100.00;
	}
    Nop();
    return SUCCESSFUL_OPERATION;
}

test_result_t ProcessHumidityTestFeedback ( message_t * pMessage, operation_status_t resultOfOperation )
{                    
    test_result_t resultOfTest; 

    resultOfTest = FAIL;
    
    if ( SUCCESSFUL_OPERATION == resultOfOperation )
    {
        if( HIGH_HUMIDITY_WARNING_LIMIT >= MyPHT_VAR.RHcompensated )
        {
            resultOfTest = PASS;
        }
        else
        {
            resultOfTest = FAIL;            
        } 
        pMessage->data[pMessage->body.dataLength++] = HUMIDITY;
        pMessage->data[pMessage->body.dataLength++] = resultOfTest;
    }
    else
    {
        resultOfTest = FAIL;
    }
    
    return resultOfTest ;
}

void ProcessHumidityDataFeedback ( message_t * pMessage, operation_status_t resultOfOperation )
{                    
    float_t humidity;
    
    if ( SUCCESSFUL_OPERATION == resultOfOperation )
    {
        humidity.DataFloat = MyPHT_VAR.RHcompensated;
        memcpy ( pMessage->data + pMessage->body.dataLength, humidity.Data, sizeof(float) );
        pMessage->body.dataLength += sizeof(float);    
    }
    else
    {
    }
}


operation_status_t AppendHumidityMessage ( BaseType_t resultOfOperation )
{    
    return IsNotifyValid ( resultOfOperation, &phtMsgNotifyTail );    
}

operation_status_t ConsumeHumidityMessage ( BaseType_t resultOfOperation )
{
    return IsNotifyValid ( resultOfOperation, &phtMsgNotifyHead );    
}

operation_status_t AppendHumidityInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &phtInterruptNotifyTail );
}

operation_status_t ConsumeHumidityInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &phtInterruptNotifyHead );    
}

BaseType_t GetHumidityMessageTail ( void ) 
{
    return phtMsgNotifyTail ;
}


/*********************************************************************
 * Function:       	void InitiateTimer2 (void)
 *
 * PreCondition:    Include Microchip PIC32MX Peripheral Library
 * 
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function initialize timer2 to tick each 0.1 ms 
 *
 * Note:            None
 ********************************************************************/
/*void InitiateTimer2 (void)
{
    CloseTimer2();
    OpenTimer2(T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 4000);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_6 | T2_INT_SUB_PRIOR_3 );
    INTEnableSystemMultiVectoredInt();
}*/

/*********************************************************************
 * Function:       	int UpdatePHTReadings (I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte, unsigned char RHAddressByte)
 *
 * PreCondition:    Declaring GetTemperature, GetPressure and GetHumidity functions
 *
 * Input:           I2C_MODULE PHT_I2C_BUS: I2C bus number
 *                  unsigned char PTAddressByte: PHT sensor PT Address
 *                  unsigned char RHAddressByte: PHT sensor RH Address
 *
 * Output:          0 if all finished
 *                  1 if one operation is left
 *                  2 if two operations are left
 *                  -1 if failed (there is an error)
 *
 * Side Effects:    None
 *
 * Overview:        This function updates PHT sensor values one value at a time
 *
 * Note:            None
 ********************************************************************/
int UpdatePHTReadings (I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte, unsigned char RHAddressByte)
{
    static enum 
    {
        GetTemperatureState,
        GetPressureState,
        GetHumidityState,  
    } MyPHT_STATES = GetTemperatureState;
    
    static int success = 1; 
    switch (MyPHT_STATES)
        {
            case GetTemperatureState:
                success = 1;
                success = GetTemperatureReading(PHT_I2C_BUS, PTAddressByte);
                if (success==0)
                {
                    MyPHT_STATES = GetPressureState;
                    return 2;
                }
                else 
                {
                    return -1;
                }
                break;
            case GetPressureState:
                success = 1;
                success = GetPressureReading(PHT_I2C_BUS, PTAddressByte);
                if (success==0)
                {
                    MyPHT_STATES = GetHumidityState;
                    return 1;
                }
                else 
                {
                    return -1;
                }
                break;
            case GetHumidityState:
                success = 1;
                success = GetHumidityReading(PHT_I2C_BUS, RHAddressByte);
                if (success==0)
                {
                    MyPHT_STATES = GetTemperatureState;
                    return 0;
                }
                else 
                {
                    return -1;
                }
                break;
            default:
                MyPHT_STATES = GetTemperatureState;
                return -1;
                break;
        }
}


/*********************************************************************
 * Function:       	void I2CInitiate ( void )
 *
 * PreCondition:    Include Microchip PIC32MX Peripheral Library
 *                  and define SYS_CLOCK, PERIPH_CLOCK() and I2C_CLOCK_FREQ
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function initialize i2c module 
 *
 * Note:            None
 ********************************************************************/
void I2CInitiate ( I2C_MODULE PHT_I2C_BUS )
{
    unsigned int              actualClock;
    // Configure Various I2C Options
    // Setting this bit stops the I2C module when the processor enters Idle mode.
    // Setting this bit switches the I2C module to high-speed I2C signaling.
    I2CConfigure(PHT_I2C_BUS, I2C_ENABLE_HIGH_SPEED);

    // Set Desired Operation Frequency
    actualClock = I2CSetFrequency(PHT_I2C_BUS, configPERIPHERAL_CLOCK_HZ, I2C_CLOCK_FREQ);
    
    
    //Hang here if the transmission is not successful
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
		Nop();
        //while (1); //Error
    }

    // Enable the I2C bus
    I2CEnable(PHT_I2C_BUS, 1); 
}
/*********************************************************************
 * Function:       	unsigned char SendCommandPHT ( I2C_MODULE PHT_I2C_BUS, unsigned char AddressByte, unsigned char CMDByte )
 *
 * PreCondition:    Include Microchip PIC32MX Peripheral Library and Initialize i2c module using I2CInitiate()
 *
 * Input:           I2C_MODULE PHT_I2C_BUS: I2C Bus number
 *                  unsigned char AddressByte: Address of the slave device in byte form
 *                  unsigned char CMDByte: Command to be send
 *
 * Output:          0 if failed
 *                  1 if success
 *
 * Side Effects:    None
 *
 * Overview:        This function send a command to a PHT sensor
 *
 * Note:            None
 ********************************************************************/
unsigned char SendCommandPHT ( I2C_MODULE PHT_I2C_BUS, unsigned char AddressByte, unsigned char CMDByte )
{
    I2C_RESULT          result;
	BaseType_t	SendCommandPHTStartTime=0;
	SendCommandPHTStartTime= xTaskGetTickCount ();
    //Stay here until ready
    //while (!I2CBusIsIdle(PHT_I2C_BUS));
	IdleI2C2();
    // Start the transfer
    result = I2CStart(PHT_I2C_BUS);

    //Hang here if the transmission is not successful
    if (result != I2C_SUCCESS)
    {
        return 0;   //while(1); //Error
    }
    IdleI2C2();
    //DelayMs(100);
    //Stay here until ready
    //while (!I2CTransmitterIsReady(PHT_I2C_BUS));
	while (!I2CTransmitterIsReady(PHT_I2C_BUS))
    {
		if( (LONG) (xTaskGetTickCount() - SendCommandPHTStartTime) > pdMS_TO_TICKS( 200 ))
        {
// FIXME:			SystemFlags.PHTFlag = CheckResultFail;
			SendCommandPHTStartTime=0;
			SendCommandPHTStartTime=xTaskGetTickCount();
			return 0;
		}
	}
    //Send byte
    result = I2CSendByte(PHT_I2C_BUS, (AddressByte << 1)|I2C_WRITE);
    
    //Hang here if the transmission is not successful
    if (result != I2C_SUCCESS)
    {
        return 0;   //while(1); //Error
    }
    //Stay here until transmission completed 
    //while (!I2CTransmissionHasCompleted(PHT_I2C_BUS));
    while (!I2CTransmissionHasCompleted(PHT_I2C_BUS))
    {
		if( (LONG) (xTaskGetTickCount() - SendCommandPHTStartTime) > pdMS_TO_TICKS( 200 ))
        {
// FIXME:			SystemFlags.PHTFlag = CheckResultFail;
			SendCommandPHTStartTime=0;
			SendCommandPHTStartTime=xTaskGetTickCount();
			return 0;
		}
	}
    //Stay here until last byte Acknowledged 
    //while (!I2CByteWasAcknowledged(PHT_I2C_BUS));
	while (!I2CByteWasAcknowledged(PHT_I2C_BUS))
    {
		if( (LONG) (xTaskGetTickCount() - SendCommandPHTStartTime) > pdMS_TO_TICKS( 200 ))
        {
// FIXME:			SystemFlags.PHTFlag = CheckResultFail;
			SendCommandPHTStartTime=0;
			SendCommandPHTStartTime=xTaskGetTickCount();
			return 0;
		}
	}
    // transmission successful
    IdleI2C2();    
    Nop();
    //Stay here until ready
    while (!I2CTransmitterIsReady(PHT_I2C_BUS));
    //Send byte
    result = I2CSendByte(PHT_I2C_BUS, CMDByte); 

    //Hang here if the transmission is not successful
    if (result != I2C_SUCCESS)
    {
        return 0;   //while(1); //Error
    }
    //Stay here until transmission completed 
    while (!I2CTransmissionHasCompleted(PHT_I2C_BUS));
    //Stay here until last byte Acknowledged 
    while (!I2CByteWasAcknowledged(PHT_I2C_BUS));
    // transmission successful
    IdleI2C2();    
    // End the transfer
    I2CStop(PHT_I2C_BUS);
//    delayMS(20);
    Nop();
    return 1; 
}
/*********************************************************************
 * Function:       	unsigned char ReceiveDataPHT ( I2C_MODULE PHT_I2C_BUS, unsigned char AddressByte, unsigned char NumberofBytes )
 *
 * PreCondition:    Include Microchip PIC32MX Peripheral Library and Initialize i2c module using I2CInitiate() and define I2CArrayBuffer char array
 *
 * Input:           I2C_MODULE PHT_I2C_BUS: I2C Bus number
 *                  unsigned char AddressByte: Address of the slave device in byte form
 *                  unsigned char NumberofBytes: Number of bytes want to receive
 *
 * Output:          0 if failed
 *                  1 if success
 *
 * Side Effects:    None
 *
 * Overview:        This function receive bytes from PHT sensor 
 *
 * Note:            None
 ********************************************************************/
unsigned char ReceiveDataPHT ( I2C_MODULE PHT_I2C_BUS, unsigned char AddressByte, unsigned char NumberofBytes )
{
    I2C_RESULT          result;
    int                 index = 0;
	
    //Stay here until ready
    //while (!I2CBusIsIdle(PHT_I2C_BUS));
    IdleI2C2();
    // Start the transfer
    result = I2CStart(PHT_I2C_BUS);

    //Hang here if the transmission is not successful
    if (result != I2C_SUCCESS)
    {
        return 0;   //while(1); //Error
    }
    IdleI2C2();
	//DelayMs(100);
    //Stay here until ready
    while (!I2CTransmitterIsReady(PHT_I2C_BUS));
    //Send byte
    result = I2CSendByte(PHT_I2C_BUS, (AddressByte << 1)|I2C_READ);
    
    //Hang here if the transmission is not successful
    if (result != I2C_SUCCESS)
    {
        return 0;   //while(1); //Error
    }
    //Stay here until transmission completed 
    while (!I2CTransmissionHasCompleted(PHT_I2C_BUS));
    //Stay here until last byte Acknowledged 
    while (!I2CByteWasAcknowledged(PHT_I2C_BUS));
    IdleI2C2();
    Nop();
    // initialize buffer with zeros
    memset(I2CArrayBuffer, 0, sizeof(I2CArrayBuffer));
    while ( index < NumberofBytes)
    {
    	IdleI2C2();
        //enables the module to receive data from the I2C bus
        I2CReceiverEnable(PHT_I2C_BUS, 1);
        //Stay here until ready
        while (!I2CReceivedDataIsAvailable(PHT_I2C_BUS));
        //Receive byte
        I2CArrayBuffer[index] = I2CGetByte(PHT_I2C_BUS);
    	IdleI2C2();
        if (index == (NumberofBytes-1))
        {
            I2CAcknowledgeByte(PHT_I2C_BUS, 0);//NACK
            while(!I2CAcknowledgeHasCompleted(PHT_I2C_BUS));
            Nop();
            break;
        }
        else
        {
            I2CAcknowledgeByte(PHT_I2C_BUS, 1);//ACK
            while(!I2CAcknowledgeHasCompleted(PHT_I2C_BUS));
			Nop();
        }
        index++;
    }
    IdleI2C2();
    // End the transfer
    I2CStop(PHT_I2C_BUS);
//    delayMS(20);
    Nop();
    return 1;
}
/*********************************************************************
 * Function:       	unsigned char crc4_PT( unsigned short int n_prom[] )
 *
 * PreCondition:    None
 *
 * Input:           unsigned short int n_prom[]: Array of size 8 for PROM values
 *
 * Output:          CRC : The calculated 4 bit CRC
 *
 * Side Effects:    None
 *
 * Overview:        This function calculates CRC for the PROM values 
 *
 * Note:            None
 ********************************************************************/
unsigned char crc4_PT( unsigned short int n_prom[] ) // n_prom defined as 8x unsigned short int (n_prom[8])
{
    int                     cnt; // simple counter
    unsigned int            n_rem=0; // crc remainder
    unsigned char           n_bit;
    
    n_prom[0]=((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
    n_prom[7]=0; // Subsidiary value, set to 0
    
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt%2==1) 
        {
            n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        }
        else 
        {
            n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        }
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000)) 
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else 
            {
                n_rem = (n_rem << 1);
            }
        }
    }
    
    n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    
    return (n_rem ^ 0x00);
}
/*
unsigned char crc4_RH(unsigned short int n_prom[]) // n_prom defined as 8x unsigned short int (n_prom[8])
{
    int cnt; // simple counter
    unsigned int n_rem=0; // crc remainder
    unsigned char n_bit;
    
    n_prom[6]=((n_prom[6]) & 0xFFF0); // CRC byte is replaced by 0
    n_prom[7]=0; // Subsidiary value, set to 0
    
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt%2==1) 
        {
            n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        }
        else 
        {
            n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        }
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000)) 
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else 
            {
                n_rem = (n_rem << 1);
            }
        }
    }
    
    n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    
    return (n_rem ^ 0x00);
}
 */
/*********************************************************************
 * Function:       	int InitializePHT ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte, unsigned char RHAddressByte  )
 *
 * PreCondition:    Declaring SendCommandPHT and ReceiveDataPHT functions
 *
 * Input:           I2C_MODULE PHT_I2C_BUS: I2C bus number
 *                  unsigned char PTAddressByte: PHT sensor PT Address
 *                  unsigned char RHAddressByte: PHT sensor RH Address
 *
 * Output:          0 if success
 *                  -1 if failed
 *
 * Side Effects:    None
 *
 * Overview:        This function reset the sensor and update PROM calibration data 
 *
 * Note:            None
 ********************************************************************/
int InitializePHT ( I2C_MODULE PHT_I2C_BUS, unsigned char PTAddressByte, unsigned char RHAddressByte )
{
    unsigned char                success = 1;
    int                 i = 0, PTCRC = 0, PTcomputedCRC = 0;
    unsigned char               PTPROMAddresses[7] = {0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC};
    
    //P&T address 0x76
    //RESET P&T SEQUENCE 0b00011110
    success = SendCommandPHT( PHT_I2C_BUS, PTAddressByte, 0x1E ); //I2C Reset Command
    //delayMS(20);
    if (!success)
    {
        return TIMEOUT_ERROR;
    }
    Nop();
    
    //PROM READ P&T SEQUENCE 
    /*  
        0xA0 CRC | Factory Defined
        0xA2 C1: Pressure sensitivity | SENS T1
        0xA4 C2: Pressure offset | OFF T1
        0xA6 C3: Temperature coefficient of pressure sensitivity | TCS
        0xA8 C4: Temperature coefficient of pressure offset | TCO
        0xAA C5: Reference temperature | T REF
        0xAC C6: Temperature coefficient of the temperature | TEMPSENS
     */
    // initialize PROM array with zeros
    memset(PTPROM, 0, sizeof(PTPROM));
    for (i=0; i<7; i++)
    {
        success = SendCommandPHT( PHT_I2C_BUS, PTAddressByte, PTPROMAddresses[i] ); //I2C Command to read P&T memory PROM address
        //delayMS(20);
        if (!success)
        {
            return TIMEOUT_ERROR;
        }
        
        success = ReceiveDataPHT( PHT_I2C_BUS, PTAddressByte, 2 ); //I2C answer from ASIC (Pressure and temperature)
        //delayMS(20);
        if (!success)
        {
            return TIMEOUT_ERROR;
        }
        //copy from buffer to update PTROM array
        PTPROM[i] = I2CArrayBuffer[0]<<8 | I2CArrayBuffer[1];
        Nop();
    } 
    Nop();
    PTCRC = (PTPROM[0]>>12)&0x000F;
    PTcomputedCRC = crc4_PT( PTPROM );
    if (PTCRC != PTcomputedCRC)
        {
            return TIMEOUT_ERROR;
        }
    //Update MyPHT_CONST struct
    memcpy(&MyPHT_CONST, &PTPROM[1], 12);
//    MyPHT_CONST.SENS_T1 = 46372; //for testing purposes
//    MyPHT_CONST.OFF_T1 = 43981; //for testing purposes
//    MyPHT_CONST.TCS = 29059; //for testing purposes
//    MyPHT_CONST.TCO = 27842; //for testing purposes
//    MyPHT_CONST.TREF = 31553; //for testing purposes
//    MyPHT_CONST.TEMPSENS = 28165; //for testing purposes
    Nop();
    //HumidityReset Command 0b11111110
    success = SendCommandPHT( PHT_I2C_BUS, RHAddressByte, 0xFE );    
    if (!success)
    {
        return TIMEOUT_ERROR; //Error
    }
    return 0;
}


static operation_status_t TestSensor (message_t* pMessage)
{    
    GetTemperatureReading (I2C_x_CHNL, PT_ADDRESS);
    GetPressureReading (I2C_x_CHNL, PT_ADDRESS);
    return GetHumidityReading (I2C_x_CHNL, RH_ADDRESS);
}

#endif // USE_PHT_MS8607

/*** end of file ***/