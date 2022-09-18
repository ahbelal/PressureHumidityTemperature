/**
 * @file humidity.h
 * @author Ahmad Belal
 * @date 03 Nov 2020
 * @brief humidity header file
 * @details humidity header file.
 */

#ifndef HUMIDITY_H
#define	HUMIDITY_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    /* 
     * Libraries 
     */
#include "global.h"
#include "task_info.h"
#include "HardwareProfile.h"
    
    /*
     *  Definitions 
     */
#define SIZE_OF_HUMIDITY_DATA           4
#define SIZE_OF_HUMIDITY_TEST           2
#define HIGH_HUMIDITY_WARNING_LIMIT     5.0f

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
    operation_status_t GetHumidity(message_t * pMessage);

        
    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t AppendHumidityMessage ( BaseType_t resultOfOperation );
    
    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t ConsumeHumidityMessage ( BaseType_t resultOfOperation );

    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t AppendHumidityInterrupt ( BaseType_t resultOfOperation );

    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t ConsumeHumidityInterrupt ( BaseType_t resultOfOperation );

    /**
     * 
     * @return 
     */
    BaseType_t GetHumidityMessageTail ( void ); 

    /*
     *  Varaibles
     */

    /* 
     *  Tasks 
     */
    // Prototypes
    portTASK_FUNCTION(HumidityTask, pParams);
    // Handles
    TaskHandle_t ghHumidityTask;         /**< Humidity task handle */


#ifdef	__cplusplus
}
#endif

#endif	/* HUMIDITY_H */
