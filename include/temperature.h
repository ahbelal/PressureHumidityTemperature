/**
 * @file temperature.h
 * @author Ahmad Belal
 * @date 03 Nov 2020
 * @brief temperature header file
 * @details temperature header file.
 */

#ifndef TEMPERATURE_H
#define	TEMPERATURE_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    /* 
     * Libraries 
     */
//#include "global.h"
#include "task_info.h"
//#include "HardwareProfile.h"
    
    /*
     *  Definitions 
     */
#define SIZE_OF_TEMPERATURE_DATA        4
#define SIZE_OF_TEMPERATURE_TEST        2
#define HIGH_TEMPERATURE_WARNING_LIMIT  60.0f
    
    /*
     *  Enum
     */

    /*
     *  Union / Struct
     */
    
    /* 
     *  Functions Prototypes
     */
    /**
     * 
     * @return 
     */
    operation_status_t GetTemperature(message_t * pMessage);
    
    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t AppendTemperatureMessage ( BaseType_t resultOfOperation );
    
    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t ConsumeTemperatureMessage ( BaseType_t resultOfOperation );

    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t AppendTemperatureInterrupt ( BaseType_t resultOfOperation );

    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t ConsumeTemperatureInterrupt ( BaseType_t resultOfOperation );

    /**
     * 
     * @return 
     */
    BaseType_t GetTemperatureMessageTail ( void ); 
    
    /*
     *  Varaibles
     */

    /* 
     *  Tasks 
     */
    // Prototypes
    portTASK_FUNCTION(TemperatureTask, pParams);
    // Handles
    TaskHandle_t ghTemperatureTask;         /**< Temperature task handle */


#ifdef	__cplusplus
}
#endif

#endif	/* TEMPERATURE_H */
