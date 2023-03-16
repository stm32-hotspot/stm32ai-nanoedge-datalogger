/**                                                                                                                                                                                                         
  ******************************************************************************                                                                                                                            
  * @file    NanoEdgeAI.h                                                                                                                                                                                   
  *                                                                                                                                                                                                         
  * @brief   Header file of NanoEdgeAI library                                                                                                                                                              
  ******************************************************************************                                                                                                                            
  * @attention                                                                                                                                                                                              
  *                                                                                                                                                                                                         
  * Copyright (c) 2022 STMicroelectronics.                                                                                                                                                                  
  * All rights reserved.                                                                                                                                                                                    
  *                                                                                                                                                                                                         
  * This software is licensed under terms that can be found in the LICENSE file                                                                                                                             
  * in the root directory of this software component.                                                                                                                                                       
  * If no LICENSE file comes with this software, it is provided AS-IS.                                                                                                                                      
  *                                                                                                                                                                                                         
  ******************************************************************************                                                                                                                            
*/

#ifndef __NANOEDGEAI_H__
#define __NANOEDGEAI_H__

/* Includes */
#include <stdint.h>

/* Define */
#define NEAI_ID "62e2814aec0a8efca99d1bcf"
#define AXIS_NUMBER 3
#define DATA_INPUT_USER 128
#define MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING 10

#ifndef __NEAI_STATE__
#define __NEAI_STATE__
enum neai_state { 
    NEAI_OK = 0,
    NEAI_INIT_FCT_NOT_CALLED = 123,
    NEAI_BOARD_ERROR,
    NEAI_KNOWLEDGE_BUFFER_ERROR,
    NEAI_NOT_ENOUGH_CALL_TO_LEARNING, //This is a fail-safe to prevent users from learning one or even no signals.
    NEAI_MINIMAL_RECOMMENDED_LEARNING_DONE,
    NEAI_UNKNOWN_ERROR};
#endif

/* Function prototypes */
#ifdef __cplusplus
extern "C" {
#endif
	enum neai_state neai_anomalydetection_init(void);
	enum neai_state neai_anomalydetection_learn(float data_input[]);
	enum neai_state neai_anomalydetection_detect(float data_input[], uint8_t *similarity);
	enum neai_state neai_anomalydetection_set_sensitivity(float sensitivity);
	float neai_anomalydetection_get_sensitivity(void);
#ifdef __cplusplus
}
#endif

#endif

/* =============
Here some sample declaration added in your main program for the use of the NanoEdge AI library.
You can directly copy these declarations or modify the names.
* WARNING: respect the size of the buffer.

uint8_t similarity = 0; // Point to similarity (see argument of neai_anomalydetection_detect fct)
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values
*/

