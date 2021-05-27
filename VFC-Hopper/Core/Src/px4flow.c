/*
 * px4flow.c
 *
 *  Created on: May 26, 2021
 *      Author: Victo
 */
#include "px4flow.h"
#define MAV_HEADER_SIZE 5;
PxParseState _currentState = PX_STATE_WAIT;
MavLinkV1Fame _currentFrame = {0};
OpticalFlowMessage _currentFlow = {0};
uint8_t _inProgress = 0;
uint8_t _currentBuffer[300];
uint8_t _currentIdx = 0;

extern osSemaphoreId_t semUART2DataHandle;

void pxReset(){
	_inProgress = 0;
	_currentIdx = 0;
	memset(_currentBuffer, 0, 300);

	PXNewData();
}
OpticalFlowMessage* pxGetLatest(){
	return &_currentFlow;
}
void pxUpdate(){
	if(!_inProgress){
		return;
	}

	if(_currentIdx == 6){
		_currentFrame.magic 	= _currentBuffer[0];
		_currentFrame.len 		= _currentBuffer[1];
		_currentFrame.seq 		= _currentBuffer[2];
		_currentFrame.sysid 	= _currentBuffer[3];
		_currentFrame.compid 	= _currentBuffer[4];
		_currentFrame.msgid 	= _currentBuffer[5];
	}

	if(_currentIdx >= (6 + (_currentFrame.len))){
		size_t offset = 6;
		float testFloat;
		size_t SIZE_UINT_64 = sizeof(uint64_t );
		size_t SIZE_FLOAT = sizeof(testFloat ) 	;
		size_t SIZE_INT_16 = sizeof(int16_t ) ;

		if(_currentFrame.msgid == 100){

			memcpy(&_currentFlow.time_usec, &_currentBuffer[offset],SIZE_UINT_64);
			offset += SIZE_UINT_64;

			memcpy(&_currentFlow.flow_comp_m_x, &_currentBuffer[offset], SIZE_FLOAT);
			offset += SIZE_FLOAT;

			memcpy(&_currentFlow.flow_comp_m_y, &_currentBuffer[offset], SIZE_FLOAT);
			offset += SIZE_FLOAT;

			memcpy(&_currentFlow.ground_distance, &_currentBuffer[offset], SIZE_FLOAT);
			offset += SIZE_FLOAT;

			memcpy(&_currentFlow.flow_x, &_currentBuffer[offset], SIZE_INT_16);
			offset += SIZE_INT_16;

			memcpy(&_currentFlow.flow_y, &_currentBuffer[offset], SIZE_INT_16);
			offset += SIZE_INT_16;

			memcpy(&_currentFlow.sensor_id, &_currentBuffer[offset], 1);
			offset += 1;


			memcpy(&_currentFlow.quality, &_currentBuffer[offset], 1);
			offset += 1;



			memcpy(&_currentFlow.flow_rate_x , &_currentBuffer[offset], SIZE_FLOAT);
			offset += SIZE_FLOAT;

			memcpy(&_currentFlow.flow_rate_y , &_currentBuffer[offset], SIZE_FLOAT);




		}

		pxReset();

	}

}

int pxParse(uint8_t* dataBuffer, size_t size){


	for(int i=0;i<size; i++){
		if(!_inProgress){
			_inProgress = dataBuffer[i] == 0xFE;
		}

		if(_inProgress){
			_currentBuffer[_currentIdx++] = dataBuffer[i];

			if(_currentIdx >= 290){
				pxReset();
			}

			pxUpdate();
		}


	}

	return 0;
}

