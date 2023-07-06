/* =============
Copyright (c) 2023, STMicroelectronics

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
  products derived from this software without specific prior written permission.

*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER / OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*
*/

#ifndef __NANOEDGEAI_H__
#define __NANOEDGEAI_H__

/* Includes */
#include <stdint.h>

/* Define */
#define AXIS_NUMBER 3
#define DATA_INPUT_USER 256

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
	/**
	 * @brief  Initialization can be run at the beginning to initialize the model.
	 *         Later to initialize a new model and reset all knowledge.
	 * @retval NEAI_OK in case of success.
	 */
	enum neai_state neai_anomalydetection_init(void);

	/**
	 * @brief  This function is used to learn patterns in your data.
	 *         It can be used at any time, in the beginning to build the original knowledge base of the AI model.
	 *         But also later, as an additional learning phase to complement the existing knowledge.
	 * @param  data_input[] [IN]: Signal to be learn of size AXIS_NUMBER * DATA_INPUT_USER
	 * @retval NEAI_NOT_ENOUGH_CALL_TO_LEARNING or NEAI_MINIMAL_RECOMMENDED_LEARNING_DONE.
	 */
	enum neai_state neai_anomalydetection_learn(float data_input[]);

	/**
	 * @brief  This function returns a similarity percentage.
	 *         The mathematical distance between the incoming signal and the existing knowledge.
	 * @param  data_input     [IN]: size of each buffer
	 * @param  *similarity     [OUT]: The percentage of similarity [0-100] ("100" means completely similar, and "0" completely dissimilar).
	 * @retval NEAI_OK in case of success
	 */
	enum neai_state neai_anomalydetection_detect(float data_input[], uint8_t *similarity);

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

