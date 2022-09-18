/**
 * @file pressure.h
 * @author Ahmad Belal
 * @date 03 Nov 2020
 * @brief pressure header file
 * @details pressure header file.
 */

#ifndef PRESSURE_H
#define	PRESSURE_H

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
#define SIZE_OF_PRESSURE_DATA           4
#define SIZE_OF_PRESSURE_TEST           2
#define HIGH_PRESSURE_WARNING_LIMIT     1100.0f

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
    operation_status_t GetPressure( message_t * pMessage);

        /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t AppendPressureMessage ( BaseType_t resultOfOperation );
    
    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t ConsumePressureMessage ( BaseType_t resultOfOperation );

    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t AppendPressureInterrupt ( BaseType_t resultOfOperation );

    /**
     * 
     * @param resultOfOperation
     * @return 
     */
    operation_status_t ConsumePressureInterrupt ( BaseType_t resultOfOperation );

    /**
     * 
     * @return 
     */
    BaseType_t GetPressureMessageTail ( void ); 


    /*
     *  Varaibles
     */

    /* 
     *  Tasks 
     */
    // Prototypes
    portTASK_FUNCTION(PressureTask, pParams);
    // Handles
    TaskHandle_t ghPressureTask;            /**< Pressure task handle */


#ifdef	__cplusplus
}
#endif

#endif	/* PRESSURE_H */
