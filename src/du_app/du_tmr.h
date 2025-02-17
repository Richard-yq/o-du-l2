/*******************************************************************************
################################################################################
#   Copyright (c) [2017-2019] [Radisys]                                        #
#                                                                              #
#   Licensed under the Apache License, Version 2.0 (the "License");            #
#   you may not use this file except in compliance with the License.           #
#   You may obtain a copy of the License at                                    #
#                                                                              #
#       http://www.apache.org/licenses/LICENSE-2.0                             #
#                                                                              #
#   Unless required by applicable law or agreed to in writing, software        #
#   distributed under the License is distributed on an "AS IS" BASIS,          #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
#   See the License for the specific language governing permissions and        #
#   limitations under the License.                                             #
################################################################################
*******************************************************************************/

#define DU_TIMER_RESOLUTION 1
#define DU_TQ_SIZE 2

short int duActvTmr(Ent ent,Inst inst); 
bool duChkTmr(PTR cb, int16_t tmrEvnt);
void duStartTmr(PTR cb, int16_t tmrEvnt, uint32_t timerValue);
void duStopTmr(PTR cb, uint8_t tmrType);
/**********************************************************************
  End of file
 **********************************************************************/
