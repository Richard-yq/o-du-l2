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


/* header include files -- defines (.h) */
#include "common_def.h"
#include "lrg.h"
#include "lrg.x"
#include "du_app_mac_inf.h"
#include "mac_sch_interface.h"
#include "lwr_mac_upr_inf.h"
#include "mac.h"
#include "lwr_mac.h"
#ifdef INTEL_FAPI
#include "nr5g_fapi_internal.h"
#include "fapi_vendor_extension.h"
#endif
#ifdef INTEL_WLS_MEM
#include "wls_lib.h"
#endif
#include "lwr_mac_fsm.h"
#include "lwr_mac_phy.h"
#include "lwr_mac_utils.h"
#include "mac_utils.h"

/* ======== small cell integration ======== */
#ifdef NFAPI
#include "vnf.h"
#include "nfapi_vnf_interface.h"
#include "vnf_p7.h"
#include "vnf_small_cell_intgr.h"
#include <pthread.h>
#endif
/* ======================================== */

#define MIB_SFN_BITMASK 0xFC
#define PDCCH_PDU_TYPE 0
#define PDSCH_PDU_TYPE 1
#define SSB_PDU_TYPE 3
#define PRACH_PDU_TYPE 0
#define PUSCH_PDU_TYPE 1
#define PUCCH_PDU_TYPE 2
#define PDU_PRESENT 1
#define SET_MSG_LEN(x, size) x += size

/* 
* ========================================== *
* Global variables 
* ========================================== *
*/
LwrMacCb lwrMacCb;
#ifdef NFAPI
vnf_info *glb_vnf;
nfapi_vnf_config_t *glb_config;
ORAN_OAI_fapi_config_req_t *intgr_fapi_config;
PNF_Lock_t *pnf_state_lock;
#endif // NFAPI
/* 
* ========================================== *
* Global variables end
* ========================================== *
*/

uint8_t UnrestrictedSetNcsTable[MAX_ZERO_CORR_CFG_IDX];
void fapiMacConfigRsp(uint16_t cellId);
uint16_t sendTxDataReq(SlotTimingInfo currTimingInfo, MacDlSlot *dlSlot, p_fapi_api_queue_elem_t prevElem, fapi_vendor_tx_data_req_t *vendorTxDataReq);
uint16_t fillUlTtiReq(SlotTimingInfo currTimingInfo, p_fapi_api_queue_elem_t prevElem, fapi_vendor_ul_tti_req_t* vendorUlTti);
uint16_t fillUlDciReq(SlotTimingInfo currTimingInfo, p_fapi_api_queue_elem_t prevElem, fapi_vendor_ul_dci_req_t *vendorUlDciReq);
uint8_t lwr_mac_procStopReqEvt(SlotTimingInfo slotInfo, p_fapi_api_queue_elem_t  prevElem, fapi_stop_req_vendor_msg_t *vendorMsg);

/* ======== small cell integration ======== */
#ifdef NFAPI
uint16_t OAI_OSC_sendTxDataReq(SlotTimingInfo currTimingInfo, MacDlSlot *dlSlot);
uint16_t OAI_OSC_fillUlTtiReq(SlotTimingInfo currTimingInfo);
uint16_t OAI_OSC_fillUlDciReq(SlotTimingInfo currTimingInfo);
#endif
/* ======================================== */


void lwrMacLayerInit(Region region, Pool pool)
{
#ifdef INTEL_WLS_MEM
   uint8_t idx;
#endif

   memset(&lwrMacCb, 0, sizeof(LwrMacCb));
   lwrMacCb.region = region;
   lwrMacCb.pool = pool;
   lwrMacCb.clCfgDone = TRUE;
   lwrMacCb.numCell = 0;
   lwrMacCb.phyState = PHY_STATE_IDLE;

#ifdef NFAPI
   lwrMacCb.pnfState = PNF_STATE_IDLE;

   pnf_state_lock = (PNF_Lock_t *)malloc(sizeof(PNF_Lock_t));
   pnf_state_lock->flag = 0;
   pthread_mutex_init(&(pnf_state_lock->mutex), NULL);
   pthread_cond_init(&(pnf_state_lock->cond), NULL);

   glb_config = NULLP;
   glb_vnf = NULLP;
#endif

#ifdef INTEL_WLS_MEM
   /* Initializing WLS free mem list */
   lwrMacCb.phySlotIndCntr = 1;
   for(idx = 0; idx < WLS_MEM_FREE_PRD; idx++)
   {
      cmLListInit(&wlsBlockToFreeList[idx]);
   }
#endif
}

/*******************************************************************
 *
 * @brief Handles Invalid Request Event
 *
 * @details
 *
 *    Function : lwr_mac_procInvalidEvt
 *
 *    Functionality:
 *         - Displays the PHY state when the invalid event occurs
 *
 * @params[in]
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t lwr_mac_procInvalidEvt(void *msg)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : INVALID_EVENT\n");
#endif
   DU_LOG("\nERROR  -->  LWR_MAC: Error Indication Event[%d] received in state [%d]", lwrMacCb.event, lwrMacCb.phyState);
   return ROK;
}

#ifdef INTEL_FAPI
/*******************************************************************
 *
 * @brief Fills FAPI message header
 *
 * @details
 *
 *    Function : fillMsgHeader
 *
 *    Functionality:
 *         -Fills FAPI message header
 *
 * @params[in] Pointer to header
 *             Number of messages
 *             Messae Type
 *             Length of message
 * @return void
 *
 * ****************************************************************/
void fillMsgHeader(fapi_msg_t *hdr, uint16_t msgType, uint16_t msgLen)
{
   memset(hdr, 0, sizeof(fapi_msg_t));
   hdr->msg_id = msgType;
   hdr->length = msgLen;
}

/*******************************************************************
 *
 * @brief Fills FAPI Config Request message header
 *
 * @details
 *
 *    Function : fillTlvs
 *
 *    Functionality:
 *         -Fills FAPI Config Request message header
 *
 * @params[in] Pointer to TLV
 *             Tag
 *             Length
 *             Value
 *             MsgLen
 * @return void
 *
 * ****************************************************************/
void fillTlvs(fapi_uint32_tlv_t *tlv, uint16_t tag, uint16_t length,
      uint32_t value, uint32_t *msgLen)
{
   tlv->tl.tag    = tag;
   tlv->tl.length = length;
   tlv->value     = value;
   *msgLen        = *msgLen + sizeof(tag) + sizeof(length) + length;
}
/*******************************************************************
 *
 * @brief fills the cyclic prefix by comparing the bitmask
 *
 * @details
 *
 *    Function : fillCyclicPrefix
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's cyclic prefix.
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ********************************************************************/
void fillCyclicPrefix(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_NORMAL_CYCLIC_PREFIX_MASK) == FAPI_NORMAL_CYCLIC_PREFIX_MASK)
   {
      (*cellPtr)->cyclicPrefix   = NORMAL_CYCLIC_PREFIX_MASK;
   }
   else if((value & FAPI_EXTENDED_CYCLIC_PREFIX_MASK) == FAPI_EXTENDED_CYCLIC_PREFIX_MASK)
   {
      (*cellPtr)->cyclicPrefix   = EXTENDED_CYCLIC_PREFIX_MASK;
   }
   else
   {
      (*cellPtr)->cyclicPrefix = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the subcarrier spacing of Downlink by comparing the bitmask
 *
 * @details
 *
 *    Function : fillSubcarrierSpaceDl
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's subcarrier spacing in DL
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 * ****************************************************************/

void fillSubcarrierSpaceDl(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_15KHZ_MASK) == FAPI_15KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingDl = SPACING_15_KHZ;
   }
   else if((value & FAPI_30KHZ_MASK) == FAPI_30KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingDl = SPACING_30_KHZ;
   }
   else if((value & FAPI_60KHZ_MASK) == FAPI_60KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingDl = SPACING_60_KHZ;
   }
   else if((value & FAPI_120KHZ_MASK) == FAPI_120KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingDl = SPACING_120_KHZ;
   }
   else
   {
      (*cellPtr)->supportedSubcarrierSpacingDl = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the downlink bandwidth by comparing the bitmask
 *
 * @details
 *
 *    Function : fillBandwidthDl
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *         -fills the cellPtr's DL Bandwidth
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 * ****************************************************************/

void fillBandwidthDl(uint16_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_5MHZ_BW_MASK) == FAPI_5MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_5MHZ;
   }
   else if((value & FAPI_10MHZ_BW_MASK) == FAPI_10MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_10MHZ;
   }
   else if((value & FAPI_15MHZ_BW_MASK) == FAPI_15MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_15MHZ;
   }
   else if((value & FAPI_20MHZ_BW_MASK) == FAPI_20MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_20MHZ;
   }
   else if((value & FAPI_40MHZ_BW_MASK) == FAPI_40MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_40MHZ;
   }
   else if((value & FAPI_50MHZ_BW_MASK) == FAPI_50MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_50MHZ;
   }
   else if((value & FAPI_60MHZ_BW_MASK) == FAPI_60MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_60MHZ;
   }
   else if((value & FAPI_70MHZ_BW_MASK) == FAPI_70MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_70MHZ;
   }
   else if((value & FAPI_80MHZ_BW_MASK) == FAPI_80MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_80MHZ;
   }
   else if((value & FAPI_90MHZ_BW_MASK) == FAPI_90MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_90MHZ;
   }
   else if((value & FAPI_100MHZ_BW_MASK) == FAPI_100MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_100MHZ;
   }
   else if((value & FAPI_200MHZ_BW_MASK) == FAPI_200MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_200MHZ;
   }
   else if((value & FAPI_400MHZ_BW_MASK) == FAPI_400MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthDl = BW_400MHZ;
   }
   else
   {
      (*cellPtr)->supportedBandwidthDl = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the subcarrier spacing of Uplink by comparing the bitmask
 *
 * @details
 *
 *    Function : fillSubcarrierSpaceUl
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *         -fills cellPtr's subcarrier spacing in UL
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 * ****************************************************************/

void fillSubcarrierSpaceUl(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_15KHZ_MASK) == FAPI_15KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingsUl = SPACING_15_KHZ;
   }
   else if((value & FAPI_30KHZ_MASK) == FAPI_30KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingsUl = SPACING_30_KHZ;
   }
   else if((value & FAPI_60KHZ_MASK) == FAPI_60KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingsUl = SPACING_60_KHZ;
   }
   else if((value & FAPI_120KHZ_MASK) == FAPI_120KHZ_MASK)
   {
      (*cellPtr)->supportedSubcarrierSpacingsUl = SPACING_120_KHZ;
   }
   else
   {
      (*cellPtr)->supportedSubcarrierSpacingsUl = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the uplink bandwidth by comparing the bitmask
 *
 * @details
 *
 *    Function : fillBandwidthUl
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's UL Bandwidth
 *
 *
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 *
 * ****************************************************************/

void fillBandwidthUl(uint16_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_5MHZ_BW_MASK) == FAPI_5MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_5MHZ;
   }
   else if((value & FAPI_10MHZ_BW_MASK) == FAPI_10MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_10MHZ;
   }
   else if((value & FAPI_15MHZ_BW_MASK) == FAPI_15MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_15MHZ;
   }
   else if((value & FAPI_20MHZ_BW_MASK) == FAPI_20MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_20MHZ;
   }
   else if((value & FAPI_40MHZ_BW_MASK) == FAPI_40MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_40MHZ;
   }
   else if((value & FAPI_50MHZ_BW_MASK) == FAPI_50MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_50MHZ;
   }
   else if((value & FAPI_60MHZ_BW_MASK) == FAPI_60MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_60MHZ;
   }
   else if((value & FAPI_70MHZ_BW_MASK) == FAPI_70MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_70MHZ;
   }
   else if((value & FAPI_80MHZ_BW_MASK) == FAPI_80MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_80MHZ;
   }
   else if((value & FAPI_90MHZ_BW_MASK) == FAPI_90MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_90MHZ;
   }
   else if((value & FAPI_100MHZ_BW_MASK) == FAPI_100MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_100MHZ;
   }
   else if((value & FAPI_200MHZ_BW_MASK) == FAPI_200MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_200MHZ;
   }
   else if((value & FAPI_400MHZ_BW_MASK) == FAPI_400MHZ_BW_MASK)
   {
      (*cellPtr)->supportedBandwidthUl = BW_400MHZ;
   }
   else
   {
      (*cellPtr)->supportedBandwidthUl = INVALID_VALUE;
   }
}
/*******************************************************************
 *
 * @brief fills the CCE maping by comparing the bitmask
 *
 * @details
 *
 *    Function : fillCCEmaping
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's CCE Mapping Type
 *
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 * ****************************************************************/

void fillCCEmaping(uint8_t value,  ClCellParam **cellPtr)
{
   if ((value & FAPI_CCE_MAPPING_INTERLEAVED_MASK) == FAPI_CCE_MAPPING_INTERLEAVED_MASK)
   {
      (*cellPtr)->cceMappingType = CCE_MAPPING_INTERLEAVED_MASK;
   }
   else if((value & FAPI_CCE_MAPPING_INTERLEAVED_MASK) == FAPI_CCE_MAPPING_NONINTERLVD_MASK)
   {
      (*cellPtr)->cceMappingType = CCE_MAPPING_NONINTERLVD_MASK;
   }
   else
   {
      (*cellPtr)->cceMappingType = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUCCH format by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPucchFormat
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's pucch format
 *
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 * ****************************************************************/

void fillPucchFormat(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_FORMAT_0_MASK) == FAPI_FORMAT_0_MASK)
   {
      (*cellPtr)->pucchFormats    = FORMAT_0;
   }
   else if((value & FAPI_FORMAT_1_MASK) == FAPI_FORMAT_1_MASK)
   {
      (*cellPtr)->pucchFormats    = FORMAT_1;
   }
   else if((value & FAPI_FORMAT_2_MASK) == FAPI_FORMAT_2_MASK)
   {
      (*cellPtr)->pucchFormats    = FORMAT_2;
   }
   else if((value & FAPI_FORMAT_3_MASK) == FAPI_FORMAT_3_MASK)
   {
      (*cellPtr)->pucchFormats    = FORMAT_3;
   }
   else if((value & FAPI_FORMAT_4_MASK) == FAPI_FORMAT_4_MASK)
   {
      (*cellPtr)->pucchFormats    = FORMAT_4;
   }
   else
   {
      (*cellPtr)->pucchFormats    = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PDSCH Mapping Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPdschMappingType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PDSCH MappingType
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 * ****************************************************************/

void fillPdschMappingType(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PDSCH_MAPPING_TYPE_A_MASK) == FAPI_PDSCH_MAPPING_TYPE_A_MASK)
   {
      (*cellPtr)->pdschMappingType = MAPPING_TYPE_A;
   }
   else if((value & FAPI_PDSCH_MAPPING_TYPE_B_MASK) == FAPI_PDSCH_MAPPING_TYPE_B_MASK)
   {
      (*cellPtr)->pdschMappingType = MAPPING_TYPE_B;
   }
   else
   {
      (*cellPtr)->pdschMappingType = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PDSCH Allocation Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPdschAllocationType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PDSCH AllocationType
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 * ****************************************************************/

void fillPdschAllocationType(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PDSCH_ALLOC_TYPE_0_MASK) == FAPI_PDSCH_ALLOC_TYPE_0_MASK)
   {
      (*cellPtr)->pdschAllocationTypes = ALLOCATION_TYPE_0;
   }
   else if((value & FAPI_PDSCH_ALLOC_TYPE_1_MASK) == FAPI_PDSCH_ALLOC_TYPE_1_MASK)
   {
      (*cellPtr)->pdschAllocationTypes = ALLOCATION_TYPE_1;
   }
   else
   {
      (*cellPtr)->pdschAllocationTypes = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PDSCH PRB Mapping Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPrbMappingType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PRB Mapping Type
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/
void fillPrbMappingType(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PDSCH_VRB_TO_PRB_MAP_NON_INTLV_MASK) == FAPI_PDSCH_VRB_TO_PRB_MAP_NON_INTLV_MASK)
   {
      (*cellPtr)->pdschVrbToPrbMapping = VRB_TO_PRB_MAP_NON_INTLV;
   }
   else if((value & FAPI_PDSCH_VRB_TO_PRB_MAP_INTLVD_MASK) == FAPI_PDSCH_VRB_TO_PRB_MAP_INTLVD_MASK)
   {
      (*cellPtr)->pdschVrbToPrbMapping = VRB_TO_PRB_MAP_INTLVD;
   }
   else
   {
      (*cellPtr)->pdschVrbToPrbMapping = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PDSCH DmrsConfig Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPdschDmrsConfigType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's DmrsConfig Type
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPdschDmrsConfigType(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PDSCH_DMRS_CONFIG_TYPE_1_MASK) == FAPI_PDSCH_DMRS_CONFIG_TYPE_1_MASK)
   {
      (*cellPtr)->pdschDmrsConfigTypes = DMRS_CONFIG_TYPE_1;
   }
   else if((value & FAPI_PDSCH_DMRS_CONFIG_TYPE_2_MASK) == FAPI_PDSCH_DMRS_CONFIG_TYPE_2_MASK)
   {
      (*cellPtr)->pdschDmrsConfigTypes = DMRS_CONFIG_TYPE_2;
   }
   else
   {
      (*cellPtr)->pdschDmrsConfigTypes = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PDSCH DmrsLength by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPdschDmrsLength
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PdschDmrsLength
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/
void fillPdschDmrsLength(uint8_t value, ClCellParam **cellPtr)
{
   if(value == FAPI_PDSCH_DMRS_MAX_LENGTH_1)
   {
      (*cellPtr)->pdschDmrsMaxLength = DMRS_MAX_LENGTH_1;
   }
   else if(value == FAPI_PDSCH_DMRS_MAX_LENGTH_2)
   {
      (*cellPtr)->pdschDmrsMaxLength = DMRS_MAX_LENGTH_2;
   }
   else
   {
      (*cellPtr)->pdschDmrsMaxLength = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PDSCH Dmrs Additional Pos by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPdschDmrsAddPos
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's Pdsch DmrsAddPos
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPdschDmrsAddPos(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_DMRS_ADDITIONAL_POS_0_MASK) == FAPI_DMRS_ADDITIONAL_POS_0_MASK)
   {
      (*cellPtr)->pdschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_0;
   }
   else if((value & FAPI_DMRS_ADDITIONAL_POS_1_MASK) == FAPI_DMRS_ADDITIONAL_POS_1_MASK)
   {
      (*cellPtr)->pdschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_1;
   }
   else if((value & FAPI_DMRS_ADDITIONAL_POS_2_MASK) == FAPI_DMRS_ADDITIONAL_POS_2_MASK)
   {
      (*cellPtr)->pdschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_2;
   }
   else if((value & FAPI_DMRS_ADDITIONAL_POS_3_MASK) == FAPI_DMRS_ADDITIONAL_POS_3_MASK)
   {
      (*cellPtr)->pdschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_3;
   }
   else
   {
      (*cellPtr)->pdschDmrsAdditionalPos = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the Modulation Order in DL by comparing the bitmask
 *
 * @details
 *
 *    Function : fillModulationOrderDl
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's ModulationOrder in DL.
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/
void fillModulationOrderDl(uint8_t value, ClCellParam **cellPtr)
{
   if(value == 0 )
   {
      (*cellPtr)->supportedMaxModulationOrderDl = MOD_QPSK;
   }
   else if(value == 1)
   {
      (*cellPtr)->supportedMaxModulationOrderDl = MOD_16QAM;
   }
   else if(value == 2)
   {
      (*cellPtr)->supportedMaxModulationOrderDl = MOD_64QAM;
   }
   else if(value == 3)
   {
      (*cellPtr)->supportedMaxModulationOrderDl = MOD_256QAM;
   }
   else
   {
      (*cellPtr)->supportedMaxModulationOrderDl = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUSCH DmrsConfig Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPuschDmrsConfigType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PUSCH DmrsConfigType
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPuschDmrsConfig(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PUSCH_DMRS_CONFIG_TYPE_1_MASK) == FAPI_PUSCH_DMRS_CONFIG_TYPE_1_MASK)
   {
      (*cellPtr)->puschDmrsConfigTypes = DMRS_CONFIG_TYPE_1;
   }
   else if((value & FAPI_PUSCH_DMRS_CONFIG_TYPE_2_MASK) == FAPI_PUSCH_DMRS_CONFIG_TYPE_2_MASK)
   {
      (*cellPtr)->puschDmrsConfigTypes = DMRS_CONFIG_TYPE_2;
   }
   else
   {
      (*cellPtr)->puschDmrsConfigTypes = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUSCH DmrsLength by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPuschDmrsLength
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PUSCH DmrsLength
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPuschDmrsLength(uint8_t value, ClCellParam **cellPtr)
{
   if(value  == FAPI_PUSCH_DMRS_MAX_LENGTH_1)
   {
      (*cellPtr)->puschDmrsMaxLength = DMRS_MAX_LENGTH_1;
   }
   else if(value  == FAPI_PUSCH_DMRS_MAX_LENGTH_2)
   {
      (*cellPtr)->puschDmrsMaxLength = DMRS_MAX_LENGTH_2;
   }
   else
   {
      (*cellPtr)->puschDmrsMaxLength = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUSCH Dmrs Additional position by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPuschDmrsAddPos
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PUSCH DmrsAddPos
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPuschDmrsAddPos(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_DMRS_ADDITIONAL_POS_0_MASK) == FAPI_DMRS_ADDITIONAL_POS_0_MASK)
   {
      (*cellPtr)->puschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_0;
   }
   else if((value & FAPI_DMRS_ADDITIONAL_POS_1_MASK) == FAPI_DMRS_ADDITIONAL_POS_1_MASK)
   {
      (*cellPtr)->puschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_1;
   }
   else if((value & FAPI_DMRS_ADDITIONAL_POS_2_MASK) == FAPI_DMRS_ADDITIONAL_POS_2_MASK)
   {
      (*cellPtr)->puschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_2;
   }
   else if((value & FAPI_DMRS_ADDITIONAL_POS_3_MASK) == FAPI_DMRS_ADDITIONAL_POS_3_MASK)
   {
      (*cellPtr)->puschDmrsAdditionalPos = DMRS_ADDITIONAL_POS_3;
   }
   else
   {
      (*cellPtr)->puschDmrsAdditionalPos = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUSCH Mapping Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPuschMappingType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PUSCH MappingType
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPuschMappingType(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PUSCH_MAPPING_TYPE_A_MASK) == FAPI_PUSCH_MAPPING_TYPE_A_MASK)
   {
      (*cellPtr)->puschMappingType = MAPPING_TYPE_A;
   }
   else if((value & FAPI_PUSCH_MAPPING_TYPE_B_MASK) == FAPI_PUSCH_MAPPING_TYPE_B_MASK)
   {
      (*cellPtr)->puschMappingType = MAPPING_TYPE_B;
   }
   else
   {
      (*cellPtr)->puschMappingType = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUSCH Allocation Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPuschAllocationType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PUSCH AllocationType
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPuschAllocationType(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PUSCH_ALLOC_TYPE_0_MASK) == FAPI_PUSCH_ALLOC_TYPE_0_MASK)
   {
      (*cellPtr)->puschAllocationTypes = ALLOCATION_TYPE_0;
   }
   else if((value & FAPI_PUSCH_ALLOC_TYPE_0_MASK) == FAPI_PUSCH_ALLOC_TYPE_0_MASK)
   {
      (*cellPtr)->puschAllocationTypes = ALLOCATION_TYPE_1;
   }
   else
   {
      (*cellPtr)->puschAllocationTypes = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUSCH PRB Mapping Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPuschPrbMappingType
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PUSCH PRB MApping Type
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPuschPrbMappingType(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PUSCH_VRB_TO_PRB_MAP_NON_INTLV_MASK) == FAPI_PUSCH_VRB_TO_PRB_MAP_NON_INTLV_MASK)
   {
      (*cellPtr)->puschVrbToPrbMapping = VRB_TO_PRB_MAP_NON_INTLV;
   }
   else if((value & FAPI_PUSCH_VRB_TO_PRB_MAP_INTLVD_MASK) == FAPI_PUSCH_VRB_TO_PRB_MAP_INTLVD_MASK)
   {
      (*cellPtr)->puschVrbToPrbMapping = VRB_TO_PRB_MAP_INTLVD;
   }
   else
   {
      (*cellPtr)->puschVrbToPrbMapping = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the Modulation Order in Ul by comparing the bitmask
 *
 * @details
 *
 *    Function : fillModulationOrderUl
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's Modualtsion Order in UL.
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillModulationOrderUl(uint8_t value, ClCellParam **cellPtr)
{
   if(value == 0)
   {
      (*cellPtr)->supportedModulationOrderUl = MOD_QPSK;
   }
   else if(value == 1)
   {
      (*cellPtr)->supportedModulationOrderUl = MOD_16QAM;
   }
   else if(value == 2)
   {
      (*cellPtr)->supportedModulationOrderUl = MOD_64QAM;
   }
   else if(value == 3)
   {
      (*cellPtr)->supportedModulationOrderUl = MOD_256QAM;
   }
   else
   {
      (*cellPtr)->supportedModulationOrderUl = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PUSCH Aggregation Factor by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPuschAggregationFactor
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PUSCH Aggregation Factor
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPuschAggregationFactor(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_FORMAT_0_MASK) == FAPI_FORMAT_0_MASK)
   {
      (*cellPtr)->puschAggregationFactor    = AGG_FACTOR_1;
   }
   else if((value & FAPI_FORMAT_1_MASK) == FAPI_FORMAT_1_MASK)
   {
      (*cellPtr)->puschAggregationFactor    = AGG_FACTOR_2;
   }
   else if((value & FAPI_FORMAT_2_MASK) == FAPI_FORMAT_2_MASK)
   {
      (*cellPtr)->puschAggregationFactor    = AGG_FACTOR_4;
   }
   else if((value & FAPI_FORMAT_3_MASK) == FAPI_FORMAT_3_MASK)
   {
      (*cellPtr)->puschAggregationFactor    = AGG_FACTOR_8;
   }
   else
   {
      (*cellPtr)->puschAggregationFactor    = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PRACH Long Format by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPrachLongFormat
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PRACH Long Format
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPrachLongFormat(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PRACH_LF_FORMAT_0_MASK) == FAPI_PRACH_LF_FORMAT_0_MASK)
   {
      (*cellPtr)->prachLongFormats    = FORMAT_0;
   }
   else if((value & FAPI_PRACH_LF_FORMAT_1_MASK) == FAPI_PRACH_LF_FORMAT_1_MASK)
   {
      (*cellPtr)->prachLongFormats    = FORMAT_1;
   }
   else if((value & FAPI_PRACH_LF_FORMAT_2_MASK) == FAPI_PRACH_LF_FORMAT_2_MASK)
   {
      (*cellPtr)->prachLongFormats    = FORMAT_2;
   }
   else if((value & FAPI_PRACH_LF_FORMAT_3_MASK) == FAPI_PRACH_LF_FORMAT_3_MASK)
   {
      (*cellPtr)->prachLongFormats    = FORMAT_3;
   }
   else
   {
      (*cellPtr)->prachLongFormats    = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the PRACH Short Format by comparing the bitmask
 *
 * @details
 *
 *    Function : fillPrachShortFormat
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's PRACH ShortFormat
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillPrachShortFormat(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_PRACH_SF_FORMAT_A1_MASK) == FAPI_PRACH_SF_FORMAT_A1_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_A1;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_A2_MASK) == FAPI_PRACH_SF_FORMAT_A2_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_A2;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_A3_MASK) == FAPI_PRACH_SF_FORMAT_A3_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_A3;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_B1_MASK) == FAPI_PRACH_SF_FORMAT_B1_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_B1;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_B2_MASK) == FAPI_PRACH_SF_FORMAT_B2_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_B2;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_B3_MASK) == FAPI_PRACH_SF_FORMAT_B3_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_B3;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_B4_MASK) == FAPI_PRACH_SF_FORMAT_B4_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_B4;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_C0_MASK) == FAPI_PRACH_SF_FORMAT_C0_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_C0;
   }
   else if((value & FAPI_PRACH_SF_FORMAT_C2_MASK) == FAPI_PRACH_SF_FORMAT_C2_MASK)
   {
      (*cellPtr)->prachShortFormats    = SF_FORMAT_C2;
   }
   else
   {
      (*cellPtr)->prachShortFormats    = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the Fd Occasions Type by comparing the bitmask
 *
 * @details
 *
 *    Function : fillFdOccasions
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's Fd Occasions
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillFdOccasions(uint8_t value, ClCellParam **cellPtr)
{
   if(value == 0)
   {
      (*cellPtr)->maxPrachFdOccasionsInASlot = PRACH_FD_OCC_IN_A_SLOT_1;
   }
   else if(value == 1)
   {
      (*cellPtr)->maxPrachFdOccasionsInASlot = PRACH_FD_OCC_IN_A_SLOT_2;
   }
   else if(value == 3)
   {
      (*cellPtr)->maxPrachFdOccasionsInASlot = PRACH_FD_OCC_IN_A_SLOT_4;
   }
   else if(value == 4)
   {
      (*cellPtr)->maxPrachFdOccasionsInASlot = PRACH_FD_OCC_IN_A_SLOT_8;
   }
   else
   {
      (*cellPtr)->maxPrachFdOccasionsInASlot = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief fills the RSSI Measurement by comparing the bitmask
 *
 * @details
 *
 *    Function : fillRssiMeas
 *
 *    Functionality:
 *         -checks the value with the bitmask and
 *          fills the cellPtr's RSSI Measurement report
 *
 * @params[in] Pointer to ClCellParam
 *             Value to be compared
 * @return void
 *
 ******************************************************************/

void fillRssiMeas(uint8_t value, ClCellParam **cellPtr)
{
   if((value & FAPI_RSSI_REPORT_IN_DBM_MASK) == FAPI_RSSI_REPORT_IN_DBM_MASK)
   {
      (*cellPtr)->rssiMeasurementSupport    = RSSI_REPORT_DBM;
   }
   else if((value & FAPI_RSSI_REPORT_IN_DBFS_MASK) == FAPI_RSSI_REPORT_IN_DBFS_MASK)
   {
      (*cellPtr)->rssiMeasurementSupport    = RSSI_REPORT_DBFS;
   }
   else
   {
      (*cellPtr)->rssiMeasurementSupport    = INVALID_VALUE;
   }
}

/*******************************************************************
 *
 * @brief Returns the TLVs value
 *
 * @details
 *
 *    Function : getParamValue
 *
 *    Functionality:
 *         -return TLVs value
 *
 * @params[in]
 * @return ROK     - temp
 *         RFAILED - failure
 *
 * ****************************************************************/

uint32_t getParamValue(fapi_uint16_tlv_t *tlv, uint16_t type)
{
   void *posPtr;
   posPtr   = &tlv->tl.tag;
   posPtr   += sizeof(tlv->tl.tag);
   posPtr   += sizeof(tlv->tl.length);
   /*TO DO: malloc to SSI memory */
   if(type == FAPI_UINT_8)
   {
      return(*(uint8_t *)posPtr);
   }
   else if(type == FAPI_UINT_16)
   {
      return(*(uint16_t *)posPtr);
   }
   else if(type == FAPI_UINT_32)
   {
      return(*(uint32_t *)posPtr);
   }
   else
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Value Extraction failed" );
      return RFAILED;
   }
}
#endif /* FAPI */

/*******************************************************************
 *
 * @brief Modifes the received mibPdu to uint32 bit
 *        and stores it in MacCellCfg
 *
 * @details
 *
 *    Function : setMibPdu
 *
 *    Functionality:
 *         -Sets the MibPdu
 *
 * @params[in] Pointer to mibPdu
 *             pointer to modified value
 ******************************************************************/
void setMibPdu(uint8_t *mibPdu, uint32_t *val, uint16_t sfn)
{
   *mibPdu |= (((uint8_t)(sfn << 2)) & MIB_SFN_BITMASK);
   *val = (mibPdu[0] << 24 | mibPdu[1] << 16 | mibPdu[2] << 8);
   DU_LOG("\nDEBUG  -->  LWR_MAC: MIB PDU %x", *val);
}

/*******************************************************************
 *
 * @brief Sends FAPI Param req to PHY
 *
 * @details
 *
 *    Function : lwr_mac_procParamReqEvt
 *
 *    Functionality:
 *         -Sends FAPI Param req to PHY
 *
 * @params[in]
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/

uint8_t lwr_mac_procParamReqEvt(void *msg)
{
#ifdef INTEL_FAPI
#ifdef CALL_FLOW_DEBUG_LOG 
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : PARAM_REQ\n");
#endif

   /* startGuardTimer(); */
   fapi_param_req_t         *paramReq = NULL;
   fapi_msg_header_t        *msgHeader;
   p_fapi_api_queue_elem_t  paramReqElem;
   p_fapi_api_queue_elem_t  headerElem;

   LWR_MAC_ALLOC(paramReqElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_param_req_t)));
   if(paramReq != NULL)
   {
      FILL_FAPI_LIST_ELEM(paramReqElem, NULLP, FAPI_PARAM_REQUEST, 1, \
         sizeof(fapi_tx_data_req_t));
      paramReq = (fapi_param_req_t *)(paramReqElem +1);
      memset(paramReq, 0, sizeof(fapi_param_req_t));
      fillMsgHeader(&paramReq->header, FAPI_PARAM_REQUEST, sizeof(fapi_param_req_t));

      /* Fill message header */
      LWR_MAC_ALLOC(headerElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_msg_header_t)));
      if(!headerElem)
      {
         DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for param req header");
         LWR_MAC_FREE(paramReqElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_param_req_t)));
         return RFAILED;
      }
      FILL_FAPI_LIST_ELEM(headerElem, paramReqElem, FAPI_VENDOR_MSG_HEADER_IND, 1, \
         sizeof(fapi_msg_header_t));
      msgHeader = (fapi_msg_header_t *)(headerElem + 1);
      msgHeader->num_msg = 1;
      msgHeader->handle = 0;

      DU_LOG("\nDEBUG  -->  LWR_MAC: Sending Param Request to Phy");
      LwrMacSendToL1(headerElem);
   }
   else
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Failed to allocate memory for Param Request");
      return RFAILED;
   }
#endif
   return ROK;
}

/*******************************************************************
 *
 * @brief Sends FAPI Param Response to MAC via PHY
 *
 * @details
 *
 *    Function : lwr_mac_procParamRspEvt
 *
 *    Functionality:
 *         -Sends FAPI Param rsp to MAC via PHY
 *
 * @params[in]
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/

uint8_t lwr_mac_procParamRspEvt(void *msg)
{
#ifdef INTEL_FAPI
   /* stopGuardTimer(); */
   uint8_t index;
   uint32_t encodedVal;
   fapi_param_resp_t *paramRsp;
   ClCellParam *cellParam = NULLP;

   paramRsp = (fapi_param_resp_t *)msg;
   DU_LOG("\nINFO  -->  LWR_MAC: Received EVENT[%d] at STATE[%d]", lwrMacCb.event, lwrMacCb.phyState);

   if(paramRsp != NULLP)
   {
      MAC_ALLOC(cellParam, sizeof(ClCellParam));
      if(cellParam != NULLP)
      {
	 DU_LOG("\nDEBUG  -->  LWR_MAC: Filling TLVS into MAC API");
	 if(paramRsp->error_code == MSG_OK)
	 {
	    for(index = 0; index < paramRsp->number_of_tlvs; index++)
	    {
	       switch(paramRsp->tlvs[index].tl.tag)
	       {
		  case FAPI_RELEASE_CAPABILITY_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_16);
		     if(encodedVal != RFAILED && (encodedVal & RELEASE_15) == RELEASE_15)
		     {
			cellParam->releaseCapability = RELEASE_15;
		     }
		     break;

		  case FAPI_PHY_STATE_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != lwrMacCb.phyState)
		     {
			DU_LOG("\nERROR  -->  PhyState mismatch [%d][%d]", lwrMacCb.phyState, lwrMacCb.event);
			return RFAILED;
		     }
		     break;

		  case FAPI_SKIP_BLANK_DL_CONFIG_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->skipBlankDlConfig = SUPPORTED;
		     }
		     else
		     {
			cellParam->skipBlankDlConfig = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_SKIP_BLANK_UL_CONFIG_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->skipBlankUlConfig = SUPPORTED;
		     }
		     else
		     {
			cellParam->skipBlankUlConfig = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_NUM_CONFIG_TLVS_TO_REPORT_TYPE_TAG:
		     cellParam->numTlvsToReport = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_16);
		     break;

		  case FAPI_CYCLIC_PREFIX_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillCyclicPrefix(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_SUPPORTED_SUBCARRIER_SPACING_DL_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillSubcarrierSpaceDl(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_SUPPORTED_BANDWIDTH_DL_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_16);
		     if(encodedVal != RFAILED)
		     {
			fillBandwidthDl(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_SUPPORTED_SUBCARRIER_SPACING_UL_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillSubcarrierSpaceUl(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_SUPPORTED_BANDWIDTH_UL_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_16);
		     if(encodedVal != RFAILED)
		     {
			fillBandwidthUl(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_CCE_MAPPING_TYPE_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillCCEmaping(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_CORESET_OUTSIDE_FIRST_3_OFDM_SYMS_OF_SLOT_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->coresetOutsideFirst3OfdmSymsOfSlot = SUPPORTED;
		     }
		     else
		     {
			cellParam->coresetOutsideFirst3OfdmSymsOfSlot = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PRECODER_GRANULARITY_CORESET_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->precoderGranularityCoreset = SUPPORTED;
		     }
		     else
		     {
			cellParam->precoderGranularityCoreset = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PDCCH_MU_MIMO_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->pdcchMuMimo = SUPPORTED;
		     }
		     else
		     {
			cellParam->pdcchMuMimo = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PDCCH_PRECODER_CYCLING_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->pdcchPrecoderCycling = SUPPORTED;
		     }
		     else
		     {
			cellParam->pdcchPrecoderCycling = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_MAX_PDCCHS_PER_SLOT_TAG:
		     cellParam->maxPdcchsPerSlot = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     break;

		  case FAPI_PUCCH_FORMATS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPucchFormat(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_MAX_PUCCHS_PER_SLOT_TAG:
		     cellParam->maxPucchsPerSlot   = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     break;

		  case FAPI_PDSCH_MAPPING_TYPE_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPdschMappingType(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PDSCH_ALLOCATION_TYPES_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPdschAllocationType(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PDSCH_VRB_TO_PRB_MAPPING_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPrbMappingType(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PDSCH_CBG_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->pdschCbg = SUPPORTED;
		     }
		     else
		     {
			cellParam->pdschCbg = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PDSCH_DMRS_CONFIG_TYPES_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPdschDmrsConfigType(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PDSCH_DMRS_MAX_LENGTH_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPdschDmrsLength(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PDSCH_DMRS_ADDITIONAL_POS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPdschDmrsAddPos(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_MAX_PDSCHS_TBS_PER_SLOT_TAG:
		     cellParam->maxPdschsTBsPerSlot = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     break;

		  case FAPI_MAX_NUMBER_MIMO_LAYERS_PDSCH_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal < FAPI_MAX_NUMBERMIMO_LAYERS_PDSCH)
		     {
			cellParam->maxNumberMimoLayersPdsch   = encodedVal;
		     }
		     break;

		  case FAPI_SUPPORTED_MAX_MODULATION_ORDER_DL_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillModulationOrderDl(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_MAX_MU_MIMO_USERS_DL_TAG:
		     cellParam->maxMuMimoUsersDl         = \
		        getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     break;

		  case FAPI_PDSCH_DATA_IN_DMRS_SYMBOLS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->pdschDataInDmrsSymbols = SUPPORTED;
		     }
		     else
		     {
			cellParam->pdschDataInDmrsSymbols = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PREMPTIONSUPPORT_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->premptionSupport = SUPPORTED;
		     }
		     else
		     {
			cellParam->premptionSupport = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PDSCH_NON_SLOT_SUPPORT_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->pdschNonSlotSupport = SUPPORTED;
		     }
		     else
		     {
			cellParam->pdschNonSlotSupport = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_UCI_MUX_ULSCH_IN_PUSCH_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->uciMuxUlschInPusch = SUPPORTED;
		     }
		     else
		     {
			cellParam->uciMuxUlschInPusch = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_UCI_ONLY_PUSCH_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->uciOnlyPusch = SUPPORTED;
		     }
		     else
		     {
			cellParam->uciOnlyPusch = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PUSCH_FREQUENCY_HOPPING_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->puschFrequencyHopping = SUPPORTED;
		     }
		     else
		     {
			cellParam->puschFrequencyHopping = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PUSCH_DMRS_CONFIG_TYPES_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPuschDmrsConfig(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PUSCH_DMRS_MAX_LEN_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPuschDmrsLength(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PUSCH_DMRS_ADDITIONAL_POS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPuschDmrsAddPos(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PUSCH_CBG_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->puschCbg = SUPPORTED;
		     }
		     else
		     {
			cellParam->puschCbg = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PUSCH_MAPPING_TYPE_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPuschMappingType(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PUSCH_ALLOCATION_TYPES_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPuschAllocationType(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PUSCH_VRB_TO_PRB_MAPPING_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPuschPrbMappingType(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PUSCH_MAX_PTRS_PORTS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal < FAPI_PUSCH_MAX_PTRS_PORTS_UB)
		     {
			cellParam->puschMaxPtrsPorts = encodedVal;
		     }
		     break;

		  case FAPI_MAX_PDUSCHS_TBS_PER_SLOT_TAG:
		     cellParam->maxPduschsTBsPerSlot = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     break;

		  case FAPI_MAX_NUMBER_MIMO_LAYERS_NON_CB_PUSCH_TAG:
		     cellParam->maxNumberMimoLayersNonCbPusch = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     break;

		  case FAPI_SUPPORTED_MODULATION_ORDER_UL_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillModulationOrderUl(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_MAX_MU_MIMO_USERS_UL_TAG:
		     cellParam->maxMuMimoUsersUl = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     break;

		  case FAPI_DFTS_OFDM_SUPPORT_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->dftsOfdmSupport = SUPPORTED;
		     }
		     else
		     {
			cellParam->dftsOfdmSupport = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_PUSCH_AGGREGATION_FACTOR_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPuschAggregationFactor(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PRACH_LONG_FORMATS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPrachLongFormat(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PRACH_SHORT_FORMATS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillPrachShortFormat(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_PRACH_RESTRICTED_SETS_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED && encodedVal != 0)
		     {
			cellParam->prachRestrictedSets = SUPPORTED;
		     }
		     else
		     {
			cellParam->prachRestrictedSets = NOT_SUPPORTED;
		     }
		     break;

		  case FAPI_MAX_PRACH_FD_OCCASIONS_IN_A_SLOT_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillFdOccasions(encodedVal, &cellParam);
		     }
		     break;

		  case FAPI_RSSI_MEASUREMENT_SUPPORT_TAG:
		     encodedVal = getParamValue(&paramRsp->tlvs[index], FAPI_UINT_8);
		     if(encodedVal != RFAILED)
		     {
			fillRssiMeas(encodedVal, &cellParam);
		     }
		     break;
		  default:
		     //DU_LOG("\nERROR  -->   Invalid value for TLV[%x] at index[%d]", paramRsp->tlvs[index].tl.tag, index);
		     break;
	       }
	    }
	    MAC_FREE(cellParam, sizeof(ClCellParam));
	    sendToLowerMac(FAPI_CONFIG_REQUEST, 0, (void *)NULL);
	    return ROK;
	 }
	 else
	 {
	    DU_LOG("\nERROR  -->   LWR_MAC: Invalid error code %d", paramRsp->error_code);
	    return RFAILED;
	 }
      }
      else
      {
	 DU_LOG("\nERROR  -->  LWR_MAC: Failed to allocate memory for cell param");
	 return RFAILED;
      }
   }
   else
   {
      DU_LOG("\nERROR  -->  LWR_MAC:  Param Response received from PHY is NULL");
      return RFAILED;
   }
#else
   return ROK;
#endif
}

#ifdef INTEL_TIMER_MODE
uint8_t lwr_mac_procIqSamplesReqEvt(void *msg)
{
   void * wlsHdlr = NULLP;
   fapi_msg_header_t *msgHeader;
   fapi_vendor_ext_iq_samples_req_t *iqSampleReq;
   p_fapi_api_queue_elem_t  headerElem;
   p_fapi_api_queue_elem_t  iqSampleElem;
   char filename[100] = "/root/intel/FlexRAN/testcase/ul/mu0_20mhz/2/uliq00_prach_tst2.bin"; 

   uint8_t buffer[] ={0,0,0,0,0,2,11,0,212,93,40,0,20,137,38,0,20,0,20,0,0,8,0,8,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,1,0,0,0,0,0,0,1,0,2,0,0,0,0,0,0,0,1,0};

   size_t bufferSize = sizeof(buffer) / sizeof(buffer[0]);

   /* Fill IQ sample req */
   mtGetWlsHdl(&wlsHdlr);
   //iqSampleElem = (p_fapi_api_queue_elem_t)WLS_Alloc(wlsHdlr, \
      (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_ext_iq_samples_req_t))); 
   LWR_MAC_ALLOC(iqSampleElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_ext_iq_samples_req_t)));
   if(!iqSampleElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for IQ sample req");
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(iqSampleElem, NULLP, FAPI_VENDOR_EXT_UL_IQ_SAMPLES, 1, \
      sizeof(fapi_vendor_ext_iq_samples_req_t));

   iqSampleReq = (fapi_vendor_ext_iq_samples_req_t *)(iqSampleElem + 1);
   memset(iqSampleReq, 0, sizeof(fapi_vendor_ext_iq_samples_req_t));
   fillMsgHeader(&iqSampleReq->header, FAPI_VENDOR_EXT_UL_IQ_SAMPLES, \
      sizeof(fapi_vendor_ext_iq_samples_req_t));

   iqSampleReq->iq_samples_info.carrNum = 0;
   iqSampleReq->iq_samples_info.numSubframes = 40;
   iqSampleReq->iq_samples_info.nIsRadioMode = 0;
   iqSampleReq->iq_samples_info.timerModeFreqDomain = 0;
   iqSampleReq->iq_samples_info.phaseCompensationEnable = 0;
   iqSampleReq->iq_samples_info.startFrameNum = 0;
   iqSampleReq->iq_samples_info.startSlotNum = 0;
   iqSampleReq->iq_samples_info.startSymNum = 0;
   strncpy(iqSampleReq->iq_samples_info.filename_in_ul_iq[0], filename, 100);
   memcpy(iqSampleReq->iq_samples_info.buffer, buffer, bufferSize);

   /* TODO : Fill remaining parameters */

   /* Fill message header */
   LWR_MAC_ALLOC(headerElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_msg_header_t)));
   if(!headerElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for FAPI header in lwr_mac_procIqSamplesReqEvt");
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(headerElem, iqSampleElem, FAPI_VENDOR_MSG_HEADER_IND, 1, \
     sizeof(fapi_msg_header_t));
   msgHeader = (fapi_msg_header_t *)(headerElem + 1);
   msgHeader->num_msg = 1; 
   msgHeader->handle = 0;

   DU_LOG("\nINFO   -->  LWR_MAC: Sending IQ Sample request to Phy");
   LwrMacSendToL1(headerElem);
   return ROK;
}
#endif

/*******************************************************************
 *
 * @brief Sends FAPI Config req to PHY
 *
 * @details
 *
 *    Function : lwr_mac_procConfigReqEvt
 *
 *    Functionality:
 *         -Sends FAPI Config Req to PHY
 *
 * @params[in]
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/

uint8_t lwr_mac_procConfigReqEvt(void *msg)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : CONFIG_REQ\n");
#endif
#ifdef NR_TDD
   uint8_t slotIdx = 0; 
   uint8_t symbolIdx =0;
#endif   
   uint16_t index = 0;
   uint16_t *cellId =NULLP;
   uint16_t cellIdx =0;
   uint32_t msgLen = 0;
   uint32_t mib = 0;
   uint32_t dlFreq = 0, ulFreq = 0;
   MacCellCfg macCfgParams;
   fapi_vendor_msg_t *vendorMsg;
   fapi_config_req_t *configReq;
   fapi_msg_header_t *msgHeader;
   p_fapi_api_queue_elem_t  headerElem;
   p_fapi_api_queue_elem_t  vendorMsgQElem;
   p_fapi_api_queue_elem_t  cfgReqQElem;

   DU_LOG("\nINFO  -->  LWR_MAC: Received EVENT[%d] at STATE[%d]", lwrMacCb.event, \
         lwrMacCb.phyState);

   cellId = (uint16_t *)msg;
   GET_CELL_IDX(*cellId, cellIdx);
   macCfgParams = macCb.macCell[cellIdx]->macCellCfg;

   /* Fill Cell Configuration in lwrMacCb */
   memset(&lwrMacCb.cellCb[lwrMacCb.numCell], 0, sizeof(LwrMacCellCb));
   lwrMacCb.cellCb[lwrMacCb.numCell].cellId = macCfgParams.cellId;
   lwrMacCb.cellCb[lwrMacCb.numCell].phyCellId = macCfgParams.cellCfg.phyCellId; 
   lwrMacCb.numCell++;

   /* Allocte And fill Vendor msg */
   LWR_MAC_ALLOC(vendorMsgQElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_msg_t)));  
   if(!vendorMsgQElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for vendor msg in config req");
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(vendorMsgQElem, NULLP, FAPI_VENDOR_MESSAGE, 1, sizeof(fapi_vendor_msg_t)); 
   vendorMsg = (fapi_vendor_msg_t *)(vendorMsgQElem + 1);
   fillMsgHeader(&vendorMsg->header, FAPI_VENDOR_MESSAGE, sizeof(fapi_vendor_msg_t));
   vendorMsg->config_req_vendor.hopping_id = 0;
   vendorMsg->config_req_vendor.carrier_aggregation_level = 0;
   vendorMsg->config_req_vendor.group_hop_flag = 0;
   vendorMsg->config_req_vendor.sequence_hop_flag = 0;
   vendorMsg->config_req_vendor.urllc_capable = 0;
   vendorMsg->config_req_vendor.urllc_mini_slot_mask =0;
   vendorMsg->config_req_vendor.nr_of_dl_ports =1;
   vendorMsg->config_req_vendor.nr_of_ul_ports =1;
   vendorMsg->config_req_vendor.prach_nr_of_rx_ru =1;
   vendorMsg->config_req_vendor.ssb_subc_spacing =1;
   vendorMsg->config_req_vendor.use_vendor_EpreXSSB = USE_VENDOR_EPREXSSB;
   vendorMsg->start_req_vendor.sfn = 0;
   vendorMsg->start_req_vendor.slot = 0;
   vendorMsg->start_req_vendor.mode = 4;
#ifdef DEBUG_MODE
   vendorMsg->start_req_vendor.count = 0;
   vendorMsg->start_req_vendor.period = 1;
#endif
   /* Fill FAPI config req */
   LWR_MAC_ALLOC(cfgReqQElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_config_req_t)));
   if(!cfgReqQElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for config req");
      LWR_MAC_FREE(vendorMsgQElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_msg_t)));
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(cfgReqQElem, vendorMsgQElem, FAPI_CONFIG_REQUEST, 1, \
         sizeof(fapi_config_req_t));

   configReq = (fapi_config_req_t *)(cfgReqQElem + 1);
   memset(configReq, 0, sizeof(fapi_config_req_t));
   fillMsgHeader(&configReq->header, FAPI_CONFIG_REQUEST, sizeof(fapi_config_req_t));
/* ======== small cell integration ======== */
#ifdef NFAPI
   #ifdef NR_TDD
      configReq->number_of_tlvs = 35 + 140;
   #else
      configReq->number_of_tlvs = 26;
   #endif
#else
   configReq->number_of_tlvs = 25 + 1 + MAX_TDD_PERIODICITY_SLOTS * MAX_SYMB_PER_SLOT;
#endif 
/* ======================================== */

   msgLen = sizeof(configReq->number_of_tlvs);

   fillTlvs(&configReq->tlvs[index++], FAPI_DL_BANDWIDTH_TAG,           \
         sizeof(uint16_t), macCfgParams.carrCfg.dlBw, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_DL_FREQUENCY_TAG,           \
         sizeof(uint32_t), macCfgParams.carrCfg.dlFreq, &msgLen);
   /* Due to bug in Intel FT code, commenting TLVs that are are not 
    * needed to avoid error. Must be uncommented when FT bug is fixed */
   fillTlvs(&configReq->tlvs[index++], FAPI_DL_K0_TAG,                  \
   sizeof(uint16_t), macCfgParams.carrCfg.dl_k0[0], &msgLen);
   
   // N_DL_RB
   fillTlvs(&configReq->tlvs[index++], FAPI_DL_GRIDSIZE_TAG,            \
   sizeof(uint16_t), macCfgParams.carrCfg.dlgridSize[0], &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_NUM_TX_ANT_TAG,             \
         sizeof(uint16_t), macCfgParams.carrCfg.numTxAnt, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_UPLINK_BANDWIDTH_TAG,       \
         sizeof(uint16_t), macCfgParams.carrCfg.ulBw, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_UPLINK_FREQUENCY_TAG,       \
         sizeof(uint32_t), macCfgParams.carrCfg.ulFreq, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_UL_K0_TAG,                  \
   sizeof(uint16_t), macCfgParams.carrCfg.ul_k0[0], &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_UL_GRID_SIZE_TAG,           \
   sizeof(uint16_t), macCfgParams.carrCfg.ulgridSize[0], &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_NUM_RX_ANT_TAG,             \
         sizeof(uint16_t), macCfgParams.carrCfg.numRxAnt, &msgLen);
   //fillTlvs(&configReq->tlvs[index++], FAPI_FREQUENCY_SHIFT_7P5_KHZ_TAG,   \
   sizeof(uint8_t), macCfgParams.freqShft, &msgLen);

   /* fill cell config */
   fillTlvs(&configReq->tlvs[index++], FAPI_PHY_CELL_ID_TAG,               \
         sizeof(uint16_t), macCfgParams.cellCfg.phyCellId, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_FRAME_DUPLEX_TYPE_TAG,         \
         sizeof(uint8_t), macCfgParams.cellCfg.dupType, &msgLen);

   /* fill SSB configuration */
   fillTlvs(&configReq->tlvs[index++], FAPI_SS_PBCH_POWER_TAG,             \
         sizeof(uint32_t), macCfgParams.ssbCfg.ssbPbchPwr, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_BCH_PAYLOAD_TAG,               \
   sizeof(uint8_t), macCfgParams.ssbCfg.bchPayloadFlag, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_SCS_COMMON_TAG,                \
         sizeof(uint8_t), macCfgParams.ssbCfg.scsCmn, &msgLen);

   /* fill PRACH configuration */
   fillTlvs(&configReq->tlvs[index++], FAPI_PRACH_SEQUENCE_LENGTH_TAG,     \
   sizeof(uint8_t), macCfgParams.prachCfg.prachSeqLen, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_PRACH_SUBC_SPACING_TAG,        \
         sizeof(uint8_t), convertScsValToScsEnum(macCfgParams.prachCfg.prachSubcSpacing), &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_RESTRICTED_SET_CONFIG_TAG,     \
         sizeof(uint8_t), macCfgParams.prachCfg.prachRstSetCfg, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_NUM_PRACH_FD_OCCASIONS_TAG,
         sizeof(uint8_t), macCfgParams.prachCfg.msg1Fdm, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_PRACH_CONFIG_INDEX_TAG,
         sizeof(uint8_t), macCfgParams.prachCfg.prachCfgIdx, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_PRACH_ROOT_SEQUENCE_INDEX_TAG, \
         sizeof(uint16_t), macCfgParams.prachCfg.fdm[0].rootSeqIdx, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_NUM_ROOT_SEQUENCES_TAG,        \
   sizeof(uint8_t), macCfgParams.prachCfg.fdm[0].numRootSeq, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_K1_TAG,                        \
         sizeof(uint16_t), macCfgParams.prachCfg.fdm[0].k1, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_PRACH_ZERO_CORR_CONF_TAG ,     \
         sizeof(uint8_t), macCfgParams.prachCfg.fdm[0].zeroCorrZoneCfg, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_NUM_UNUSED_ROOT_SEQUENCES_TAG, \
   sizeof(uint16_t), 0, &msgLen);
   /* if(macCfgParams.prachCfg.fdm[0].numUnusedRootSeq)
      {
      for(idx = 0; idx < macCfgParams.prachCfg.fdm[0].numUnusedRootSeq; idx++)
      fillTlvs(&configReq->tlvs[index++], FAPI_UNUSED_ROOT_SEQUENCES_TAG,   \
      sizeof(uint8_t), macCfgParams.prachCfg.fdm[0].unsuedRootSeq[idx], \
      &msgLen);
      }
      else
      {
      macCfgParams.prachCfg.fdm[0].unsuedRootSeq = NULL;
      }*/

   fillTlvs(&configReq->tlvs[index++], FAPI_SSB_PER_RACH_TAG,
            sizeof(uint8_t), macCfgParams.prachCfg.ssbPerRach, &msgLen);

   fillTlvs(&configReq->tlvs[index++], FAPI_PRACH_MULTIPLE_CARRIERS_IN_A_BAND_TAG,  \
   sizeof(uint8_t), 0, &msgLen);

   /* fill SSB table */
   fillTlvs(&configReq->tlvs[index++], FAPI_SSB_OFFSET_POINT_A_TAG,
            sizeof(uint16_t), macCfgParams.ssbCfg.ssbOffsetPointA, &msgLen);

   //fillTlvs(&configReq->tlvs[index++], FAPI_BETA_PSS_TAG,                \
   sizeof(uint8_t),  macCfgParams.ssbCfg.betaPss, &msgLen);
   fillTlvs(&configReq->tlvs[index++], FAPI_SSB_PERIOD_TAG,
            sizeof(uint8_t), macCfgParams.ssbCfg.ssbPeriod, &msgLen);

   fillTlvs(&configReq->tlvs[index++], FAPI_SSB_SUBCARRIER_OFFSET_TAG,
            sizeof(uint8_t), macCfgParams.ssbCfg.ssbScOffset, &msgLen);

   setMibPdu(macCfgParams.ssbCfg.mibPdu, &mib, 0);
   fillTlvs(&configReq->tlvs[index++], FAPI_MIB_TAG,
            sizeof(uint32_t), 0, &msgLen);

   fillTlvs(&configReq->tlvs[index++], FAPI_SSB_MASK_TAG,
            sizeof(uint32_t), macCfgParams.ssbCfg.ssbMask[0], &msgLen);

   fillTlvs(&configReq->tlvs[index++], FAPI_BEAM_ID_TAG,
            sizeof(uint8_t), macCfgParams.ssbCfg.beamId[0], &msgLen);

   //fillTlvs(&configReq->tlvs[index++], FAPI_SS_PBCH_MULTIPLE_CARRIERS_IN_A_BAND_TAG, \
   sizeof(uint8_t), macCfgParams.ssbCfg.multCarrBand, &msgLen);
   //fillTlvs(&configReq->tlvs[index++], FAPI_MULTIPLE_CELLS_SS_PBCH_IN_A_CARRIER_TAG, \
   sizeof(uint8_t), macCfgParams.ssbCfg.multCellCarr, &msgLen);

#ifdef NR_TDD
   /* fill TDD table */
   fillTlvs(&configReq->tlvs[index++], FAPI_TDD_PERIOD_TAG,                \
         sizeof(uint8_t), macCfgParams.tddCfg.tddPeriod, &msgLen);

   fillTlvs(&configReq->tlvs[index++], FAPI_RSSI_MEASUREMENT_TAG,                \
         sizeof(uint8_t), 1, &msgLen);

   for(slotIdx =0 ;slotIdx < MAX_TDD_PERIODICITY_SLOTS; slotIdx++) 
   {
      for(symbolIdx = 0; symbolIdx < MAX_SYMB_PER_SLOT; symbolIdx++)
      {
         /*Fill Full-DL Slots as well as DL symbols ini 1st Flexi Slo*/
         if(slotIdx < macCfgParams.tddCfg.nrOfDlSlots || \
               (slotIdx == macCfgParams.tddCfg.nrOfDlSlots && symbolIdx < macCfgParams.tddCfg.nrOfDlSymbols)) 
         {
            fillTlvs(&configReq->tlvs[index++], FAPI_SLOT_CONFIG_TAG,               \
                  sizeof(uint8_t), DL_SYMBOL, &msgLen);
         }

         /*Fill Full-FLEXI SLOT and as well as Flexi Symbols in 1 slot preceding FULL-UL slot*/ 
         else if(slotIdx < (MAX_TDD_PERIODICITY_SLOTS - macCfgParams.tddCfg.nrOfUlSlots -1) ||  \
               (slotIdx == (MAX_TDD_PERIODICITY_SLOTS - macCfgParams.tddCfg.nrOfUlSlots -1) && \
                symbolIdx < (MAX_SYMB_PER_SLOT - macCfgParams.tddCfg.nrOfUlSymbols)))
         {
            fillTlvs(&configReq->tlvs[index++], FAPI_SLOT_CONFIG_TAG,               \
                  sizeof(uint8_t), FLEXI_SYMBOL, &msgLen);
         }
         /*Fill Partial UL symbols and Full-UL slot*/
         else
         {
            fillTlvs(&configReq->tlvs[index++], FAPI_SLOT_CONFIG_TAG,               \
                  sizeof(uint8_t), UL_SYMBOL, &msgLen);
         }
      }
   }
#endif   

#ifndef NFAPI
   /* Fill message header */
   LWR_MAC_ALLOC(headerElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_msg_header_t)));
   if(!headerElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for vendor msg in config req");
      LWR_MAC_FREE(cfgReqQElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_config_req_t)));
      LWR_MAC_FREE(vendorMsgQElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_msg_t)));
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(headerElem, cfgReqQElem, FAPI_VENDOR_MSG_HEADER_IND, 1, \
         sizeof(fapi_msg_header_t));
   msgHeader = (fapi_msg_header_t *)(headerElem + 1);
   msgHeader->num_msg = 2; /* Config req msg and vendor specific msg */
   msgHeader->handle = 0;

   DU_LOG("\nDEBUG  -->  LWR_MAC: Sending Config Request to Phy");
   LwrMacSendToL1(headerElem);
#else
/* ======== small cell integration ======== */
   intgr_fapi_config = configReq;
   (glb_config->nr_param_resp)(glb_config, intgr_p5_idx, intgr_resp);
/* ======================================== */
#endif // NFAPI
   return ROK;
} /* lwr_mac_handleConfigReqEvt */

/*******************************************************************
 *
 * @brief Processes config response from phy
 *
 * @details
 *
 *    Function : lwr_mac_procConfigRspEvt
 *
 *    Functionality:
 *          Processes config response from phy
 *
 * @params[in] FAPI message pointer 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/

uint8_t lwr_mac_procConfigRspEvt(void *msg)
{
#ifdef INTEL_FAPI
#ifndef NFAPI
   fapi_config_resp_t *configRsp;
   configRsp = (fapi_config_resp_t *)msg;

   DU_LOG("\nINFO  -->  LWR_MAC: Received EVENT[%d] at STATE[%d]", lwrMacCb.event, \
	 lwrMacCb.phyState);

   if(configRsp != NULL)
   {
      if(configRsp->error_code == MSG_OK)
      {
	 DU_LOG("\nDEBUG  -->  LWR_MAC: PHY has moved to Configured state \n");
	 lwrMacCb.phyState = PHY_STATE_CONFIGURED;
	 lwrMacCb.cellCb[0].state = PHY_STATE_CONFIGURED;
	 /* TODO : 
	  * Store config response into an intermediate struture and send to MAC
	  * Support LC and LWLC for sending config rsp to MAC 
	  */
	 fapiMacConfigRsp(lwrMacCb.cellCb[0].cellId);
      }
      else
      {
	 DU_LOG("\nERROR  -->  LWR_MAC: Invalid error code %d", configRsp->error_code);
	 return RFAILED;
      }
   }
   else
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Config Response received from PHY is NULL");
      return RFAILED;
   }
#endif // not define NFAPI
   nfapi_nr_config_response_scf_t *nr_cfg_resp;
   nr_cfg_resp = (nfapi_nr_config_response_scf_t *)msg;
   DU_LOG("\nINFO  -->  LWR_MAC: Received EVENT[%d] at STATE[%d]", lwrMacCb.event, lwrMacCb.phyState);

   if (nr_cfg_resp != NULL)
   {
      if (nr_cfg_resp->error_code == MSG_OK)
      {
         DU_LOG("\nDEBUG  -->  LWR_MAC: PHY has moved to Configured state \n");
         lwrMacCb.phyState = PHY_STATE_CONFIGURED;
         lwrMacCb.cellCb[0].state = PHY_STATE_CONFIGURED;
         /* TODO : When connected to OAI CU, you can uncomment this part.
          * Store config response into an intermediate struture and send to MAC
          * Support LC and LWLC for sending config rsp to MAC
          */
          fapiMacConfigRsp(lwrMacCb.cellCb[0].cellId);
      }
      else
      {
         DU_LOG("\nERROR  -->  LWR_MAC: Invalid error code %d", nr_cfg_resp->error_code);
         return RFAILED;
      }
   }
   else
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Config Response received from PHY is NULL");
      return RFAILED;
   }
#endif //INTEL_FAPI

   return ROK;
} /* lwr_mac_procConfigRspEvt */

/*******************************************************************
 *
 * @brief Build and send start request to phy
 *
 * @details
 *
 *    Function : lwr_mac_procStartReqEvt
 *
 *    Functionality:
 *       Build and send start request to phy
 *
 * @params[in] FAPI message pointer
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t lwr_mac_procStartReqEvt(void *msg)
{
#ifndef NFAPI
#ifdef INTEL_FAPI
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : START_REQ\n");
#endif
   fapi_msg_header_t *msgHeader;
   fapi_start_req_t *startReq;
   fapi_vendor_msg_t *vendorMsg;
   p_fapi_api_queue_elem_t  headerElem;
   p_fapi_api_queue_elem_t  startReqElem;
   p_fapi_api_queue_elem_t  vendorMsgElem;

   /* Allocte And fill Vendor msg */
   LWR_MAC_ALLOC(vendorMsgElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_msg_t)));
   if(!vendorMsgElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for vendor msg in start req");
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(vendorMsgElem, NULLP, FAPI_VENDOR_MESSAGE, 1, sizeof(fapi_vendor_msg_t));
   vendorMsg = (fapi_vendor_msg_t *)(vendorMsgElem + 1);
   fillMsgHeader(&vendorMsg->header, FAPI_VENDOR_MESSAGE, sizeof(fapi_vendor_msg_t));
   vendorMsg->start_req_vendor.sfn = 0;
   vendorMsg->start_req_vendor.slot = 0;
   vendorMsg->start_req_vendor.mode = 4; /* for Radio mode */
#ifdef DEBUG_MODE
   vendorMsg->start_req_vendor.count = 0;
   vendorMsg->start_req_vendor.period = 1;
#endif

   /* Fill FAPI config req */
   LWR_MAC_ALLOC(startReqElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_start_req_t)));
   if(!startReqElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for start req");
      LWR_MAC_FREE(vendorMsgElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_msg_t)));
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(startReqElem, vendorMsgElem, FAPI_START_REQUEST, 1, \
      sizeof(fapi_start_req_t));

   startReq = (fapi_start_req_t *)(startReqElem + 1);
   memset(startReq, 0, sizeof(fapi_start_req_t));
   fillMsgHeader(&startReq->header, FAPI_START_REQUEST, sizeof(fapi_start_req_t));

   /* Fill message header */
   LWR_MAC_ALLOC(headerElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_msg_header_t)));
   if(!headerElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for vendor msg in config req");
      LWR_MAC_FREE(startReqElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_start_req_t)));
      LWR_MAC_FREE(vendorMsgElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_msg_t)));
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(headerElem, startReqElem, FAPI_VENDOR_MSG_HEADER_IND, 1, \
      sizeof(fapi_msg_header_t));
   msgHeader = (fapi_msg_header_t *)(headerElem + 1);
   msgHeader->num_msg = 2; /* Start req msg and vendor specific msg */
   msgHeader->handle = 0;

   /* Send to PHY */
   DU_LOG("\nDEBUG  -->  LWR_MAC: Sending Start Request to Phy");
   LwrMacSendToL1(headerElem);
#endif // INTEL_FAPI
#endif // Not define NFAPI
   return ROK;
} /* lwr_mac_procStartReqEvt */

/*******************************************************************
 *
 * @brief Sends FAPI Stop Req to PHY
 *
 * @details
 *
 *    Function : lwr_mac_procStopReqEvt
 *
 *    Functionality:
 *         -Sends FAPI Stop Req to PHY
 *
 * @params[in]
 * @return ROK     - success
 *         RFAILED - failure
 *
 ********************************************************************/

uint8_t lwr_mac_procStopReqEvt(SlotTimingInfo slotInfo, p_fapi_api_queue_elem_t  prevElem, fapi_stop_req_vendor_msg_t *vendorMsg)
{
#ifdef INTEL_FAPI
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : STOP_REQ\n");
#endif

   fapi_stop_req_t   *stopReq;
   p_fapi_api_queue_elem_t  stopReqElem;

   vendorMsg->sfn = slotInfo.sfn;
   vendorMsg->slot = slotInfo.slot;

   /* Fill FAPI stop req */
   LWR_MAC_ALLOC(stopReqElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_stop_req_t)));
   if(!stopReqElem)
   {
      DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for stop req");
      return RFAILED;
   }
   FILL_FAPI_LIST_ELEM(stopReqElem, NULLP, FAPI_STOP_REQUEST, 1, sizeof(fapi_stop_req_t));
   stopReq = (fapi_stop_req_t *)(stopReqElem + 1);
   memset(stopReq, 0, sizeof(fapi_stop_req_t));
   fillMsgHeader(&stopReq->header, FAPI_STOP_REQUEST, sizeof(fapi_stop_req_t));

   /* Send to PHY */
   DU_LOG("\nINFO  -->  LWR_MAC: Sending Stop Request to Phy");
   prevElem->p_next = stopReqElem;

#endif
   return ROK;
}

#ifdef INTEL_FAPI
/*******************************************************************
 *
 * @brief fills SSB PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillSsbPdu
 *
 *    Functionality:
 *         -Fills the SSB PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to FAPI DL TTI Req
 *             Pointer to RgCellCb
 *             Pointer to msgLen of DL TTI Info
 * @return ROK
 *
 ******************************************************************/

uint8_t fillSsbPdu(fapi_dl_tti_req_pdu_t *dlTtiReqPdu, MacCellCfg *macCellCfg,
      MacDlSlot *currDlSlot, uint8_t ssbIdxCount, uint16_t sfn)
{
   uint32_t mibPayload = 0;
   if(dlTtiReqPdu != NULL)
   {
      dlTtiReqPdu->pduType = SSB_PDU_TYPE;     /* SSB PDU */
      dlTtiReqPdu->pdu.ssb_pdu.physCellId = macCellCfg->cellCfg.phyCellId;
      dlTtiReqPdu->pdu.ssb_pdu.betaPss = macCellCfg->ssbCfg.betaPss;
      dlTtiReqPdu->pdu.ssb_pdu.ssbBlockIndex = currDlSlot->dlInfo.brdcstAlloc.ssbInfo[ssbIdxCount].ssbIdx;
      dlTtiReqPdu->pdu.ssb_pdu.ssbSubCarrierOffset = macCellCfg->ssbCfg.ssbScOffset;;
      /* ssbOfPdufstA to be filled in ssbCfg */
      dlTtiReqPdu->pdu.ssb_pdu.ssbOffsetPointA = macCellCfg->ssbCfg.ssbOffsetPointA;;
      dlTtiReqPdu->pdu.ssb_pdu.bchPayloadFlag = macCellCfg->ssbCfg.bchPayloadFlag;
      /* Bit manipulation for SFN */
      setMibPdu(macCellCfg->ssbCfg.mibPdu, &mibPayload, sfn);
      dlTtiReqPdu->pdu.ssb_pdu.bchPayload.bchPayload = mibPayload;
      dlTtiReqPdu->pdu.ssb_pdu.preCodingAndBeamforming.numPrgs = 0;
      dlTtiReqPdu->pdu.ssb_pdu.preCodingAndBeamforming.prgSize = 0;
      dlTtiReqPdu->pdu.ssb_pdu.preCodingAndBeamforming.digBfInterfaces = 0;
      dlTtiReqPdu->pdu.ssb_pdu.preCodingAndBeamforming.pmi_bfi[0].pmIdx = 0;
      dlTtiReqPdu->pdu.ssb_pdu.preCodingAndBeamforming. \
	 pmi_bfi[0].beamIdx[0].beamidx = macCellCfg->ssbCfg.beamId[0];
      dlTtiReqPdu->pduSize = sizeof(fapi_dl_ssb_pdu_t);  /* Size of SSB PDU */
      return ROK;
   }
   return RFAILED;
}

/*******************************************************************
 *
 * @brief fills Dl DCI PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillSib1DlDciPdu
 *
 *    Functionality:
 *         -Fills the Dl DCI PDU
 *
 * @params[in] Pointer to fapi_dl_dci_t
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/

void fillSib1DlDciPdu(fapi_dl_dci_t *dlDciPtr, PdcchCfg *sib1PdcchInfo)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes=0;
      uint8_t bytePos=0;
      uint8_t bitPos=0;

      uint16_t coreset0Size=0;
      uint16_t rbStart=0;
      uint16_t rbLen=0;
      uint32_t freqDomResAssign=0;
      uint32_t timeDomResAssign=0;
      uint8_t  VRB2PRBMap=0;
      uint32_t modNCodScheme=0;
      uint8_t  redundancyVer=0;
      uint32_t sysInfoInd=0;
      uint32_t reserved=0;

      /* Size(in bits) of each field in DCI format 0_1 
       * as mentioned in spec 38.214 */
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t redundancyVerSize    = 2;
      uint8_t sysInfoIndSize       = 1;
      uint8_t reservedSize         = 15;
   
      dlDciPtr->rnti = sib1PdcchInfo->dci[0].rnti;
      dlDciPtr->scramblingId = sib1PdcchInfo->dci[0].scramblingId;    
      dlDciPtr->scramblingRnti = sib1PdcchInfo->dci[0].scramblingRnti;
      dlDciPtr->cceIndex = sib1PdcchInfo->dci[0].cceIndex;
      dlDciPtr->aggregationLevel = sib1PdcchInfo->dci[0].aggregLevel;
      dlDciPtr->pc_and_bform.numPrgs = sib1PdcchInfo->dci[0].beamPdcchInfo.numPrgs;
      dlDciPtr->pc_and_bform.prgSize = sib1PdcchInfo->dci[0].beamPdcchInfo.prgSize;
      dlDciPtr->pc_and_bform.digBfInterfaces = sib1PdcchInfo->dci[0].beamPdcchInfo.digBfInterfaces;
      dlDciPtr->pc_and_bform.pmi_bfi[0].pmIdx = sib1PdcchInfo->dci[0].beamPdcchInfo.prg[0].pmIdx;
      dlDciPtr->pc_and_bform.pmi_bfi[0].beamIdx[0].beamidx = sib1PdcchInfo->dci[0].beamPdcchInfo.prg[0].beamIdx[0];
      dlDciPtr->beta_pdcch_1_0 = sib1PdcchInfo->dci[0].txPdcchPower.beta_pdcch_1_0;           
      dlDciPtr->powerControlOffsetSS = sib1PdcchInfo->dci[0].txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      coreset0Size= sib1PdcchInfo->coresetCfg.coreSetSize;
      rbStart = sib1PdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.startPrb;
      rbLen = sib1PdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coreset0Size - rbStart))
      {
	 if((rbLen - 1) <= floor(coreset0Size / 2))
	    freqDomResAssign = (coreset0Size * (rbLen-1)) + rbStart;
	 else
	    freqDomResAssign = (coreset0Size * (coreset0Size - rbLen + 1)) \
			       + (coreset0Size - 1 - rbStart);

	 freqDomResAssignSize = ceil(log2(coreset0Size * (coreset0Size + 1) / 2));
      }

      /* Fetching DCI field values */
      timeDomResAssign = sib1PdcchInfo->dci[0].pdschCfg.pdschTimeAlloc.rowIndex -1;
      VRB2PRBMap       = sib1PdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.vrbPrbMapping;
      modNCodScheme    = sib1PdcchInfo->dci[0].pdschCfg.codeword[0].mcsIndex;
      redundancyVer    = sib1PdcchInfo->dci[0].pdschCfg.codeword[0].rvIndex;
      sysInfoInd       = 0;           /* 0 for SIB1; 1 for SI messages */
      reserved         = 0;

      /* Reversing bits in each DCI field */
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      redundancyVer    = reverseBits(redundancyVer, redundancyVerSize);
      sysInfoInd       = reverseBits(sysInfoInd, sysInfoIndSize);

      /* Calulating total number of bytes in buffer */
      dlDciPtr->payloadSizeBits = freqDomResAssignSize + timeDomResAssignSize\
				  + VRB2PRBMapSize + modNCodSchemeSize + redundancyVerSize\
				  + sysInfoIndSize + reservedSize;

      numBytes = dlDciPtr->payloadSizeBits / 8;
      if(dlDciPtr->payloadSizeBits % 8)
	 numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
	 DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
	 return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
	 dlDciPtr->payload[bytePos] = 0;

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    redundancyVer, redundancyVerSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    sysInfoInd, sysInfoIndSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    reserved, reservedSize);

   }
} /* fillSib1DlDciPdu */


/*******************************************************************
 *
 * @brief fills Dl DCI PDU for Paging required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillPageDlDciPdu
 *
 *    Functionality:
 *         -Fills the Dl DCI PDU for Paging
 *
 * @params[in] Pointer to fapi_dl_dci_t
 *             Pointer to dlPageAlloc
 * @return ROK
 *
 ******************************************************************/

void fillPageDlDciPdu(fapi_dl_dci_t *dlDciPtr, DlPageAlloc *dlPageAlloc, MacCellCfg *macCellCfg)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes=0;
      uint8_t bytePos=0;
      uint8_t bitPos=0;

      uint16_t coreset0Size     = 0;
      uint16_t rbStart          = 0;
      uint16_t rbLen            = 0;
      uint8_t  shortMsgInd      = 0;
      uint8_t  shortMsg         = 0;
      uint32_t freqDomResAssign = 0;
      uint32_t timeDomResAssign = 0;
      uint8_t  VRB2PRBMap       = 0;
      uint32_t modNCodScheme    = 0;
      uint8_t  tbScaling        = 0;
      uint32_t reserved         = 0;

      /* Size(in bits) of each field in DCI format 1_0 
       * as mentioned in spec 38.214 */
      uint8_t shortMsgIndSize      = 2;
      uint8_t shortMsgSize         = 8;
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t tbScalingSize        = 2;
      uint8_t reservedSize         = 6;

      dlDciPtr->rnti = P_RNTI;
      dlDciPtr->scramblingId = macCellCfg->cellCfg.phyCellId;
      dlDciPtr->scramblingRnti = 0;
      dlDciPtr->cceIndex = dlPageAlloc->pageDlDci.cceIndex;
      dlDciPtr->aggregationLevel = dlPageAlloc->pageDlDci.aggregLevel;
      dlDciPtr->pc_and_bform.numPrgs = 1;
      dlDciPtr->pc_and_bform.prgSize = 1;
      dlDciPtr->pc_and_bform.digBfInterfaces = 0;
      dlDciPtr->pc_and_bform.pmi_bfi[0].pmIdx = 0;
      dlDciPtr->pc_and_bform.pmi_bfi[0].beamIdx[0].beamidx = 0;
      dlDciPtr->beta_pdcch_1_0 = 0;
      dlDciPtr->powerControlOffsetSS = 0;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      coreset0Size = dlPageAlloc->pageDlDci.coreSetSize;
      rbStart = dlPageAlloc->pageDlSch.freqAlloc.startPrb;
      rbLen = dlPageAlloc->pageDlSch.freqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coreset0Size - rbStart))
      {
         if((rbLen - 1) <= floor(coreset0Size / 2))
            freqDomResAssign = (coreset0Size * (rbLen-1)) + rbStart;
         else
            freqDomResAssign = (coreset0Size * (coreset0Size - rbLen + 1)) \
                               + (coreset0Size - 1 - rbStart);

         freqDomResAssignSize = ceil(log2(coreset0Size * (coreset0Size + 1) / 2));
      }

      /*Fetching DCI field values */

      /*Refer:38.212 - Table 7.3.1.2.1-1: Short Message indicator >*/
      if(dlPageAlloc->shortMsgInd != TRUE)
      {
         /*When Short Msg is absent*/
         shortMsgInd = 1;
         shortMsg    = 0;
      }
      else
      {
         /*Short Msg is Present*/
         if(dlPageAlloc->pageDlSch.dlPagePduLen == 0 || dlPageAlloc->pageDlSch.dlPagePdu == NULLP)
         {
            /*When Paging Msg is absent*/
            shortMsgInd = 2;
         }
         else
         {
            /*Both Short and Paging is present*/
            shortMsgInd = 3;
         }
         shortMsg = dlPageAlloc->shortMsg;
      }

      timeDomResAssign = 0;
      VRB2PRBMap       = dlPageAlloc->pageDlSch.vrbPrbMapping;
      modNCodScheme    = dlPageAlloc->pageDlSch.tbInfo.mcs;
      tbScaling        = 0;
      reserved         = 0;

      /* Reversing bits in each DCI field */
      shortMsgInd      = reverseBits(shortMsgInd, shortMsgIndSize);
      shortMsg         = reverseBits(shortMsg, shortMsgSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      tbScaling        = reverseBits(tbScaling, tbScalingSize); 

      /* Calulating total number of bytes in buffer */
      dlDciPtr->payloadSizeBits = shortMsgIndSize + shortMsgSize + freqDomResAssignSize\
                                  + timeDomResAssignSize + VRB2PRBMapSize + modNCodSchemeSize\
                                  + tbScaling + reservedSize;

      numBytes = dlDciPtr->payloadSizeBits / 8;
      if(dlDciPtr->payloadSizeBits % 8)
      {
         numBytes += 1;
      }

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
         DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
         return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
      {
         dlDciPtr->payload[bytePos] = 0;
      }

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            shortMsgInd, shortMsgIndSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            shortMsg, shortMsgSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            tbScaling, tbScalingSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            reserved, reservedSize);
   }
} /* fillPageDlDciPdu */

/*******************************************************************
 *
 * @brief fills Dl DCI PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillRarDlDciPdu
 *
 *    Functionality:
 *         -Fills the Dl DCI PDU
 *
 * @params[in] Pointer to fapi_dl_dci_t
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/

void fillRarDlDciPdu(fapi_dl_dci_t *dlDciPtr, PdcchCfg *rarPdcchInfo)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes =0;
      uint8_t bytePos =0;
      uint8_t bitPos =0;

      uint16_t coreset0Size =0;
      uint16_t rbStart =0;
      uint16_t rbLen =0;
      uint32_t freqDomResAssign =0;
      uint8_t timeDomResAssign =0;
      uint8_t  VRB2PRBMap =0;
      uint8_t modNCodScheme =0;
      uint8_t tbScaling =0;
      uint32_t reserved =0;

      /* Size(in bits) of each field in DCI format 1_0 */
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t tbScalingSize        = 2;
      uint8_t reservedSize         = 16;
      
      dlDciPtr->rnti = rarPdcchInfo->dci[0].rnti;
      dlDciPtr->scramblingId = rarPdcchInfo->dci[0].scramblingId;    
      dlDciPtr->scramblingRnti = rarPdcchInfo->dci[0].scramblingRnti;
      dlDciPtr->cceIndex = rarPdcchInfo->dci[0].cceIndex;
      dlDciPtr->aggregationLevel = rarPdcchInfo->dci[0].aggregLevel;
      dlDciPtr->pc_and_bform.numPrgs = rarPdcchInfo->dci[0].beamPdcchInfo.numPrgs;
      dlDciPtr->pc_and_bform.prgSize = rarPdcchInfo->dci[0].beamPdcchInfo.prgSize;
      dlDciPtr->pc_and_bform.digBfInterfaces = rarPdcchInfo->dci[0].beamPdcchInfo.digBfInterfaces;
      dlDciPtr->pc_and_bform.pmi_bfi[0].pmIdx = rarPdcchInfo->dci[0].beamPdcchInfo.prg[0].pmIdx;
      dlDciPtr->pc_and_bform.pmi_bfi[0].beamIdx[0].beamidx = rarPdcchInfo->dci[0].beamPdcchInfo.prg[0].beamIdx[0];
      dlDciPtr->beta_pdcch_1_0 = rarPdcchInfo->dci[0].txPdcchPower.beta_pdcch_1_0;           
      dlDciPtr->powerControlOffsetSS = rarPdcchInfo->dci[0].txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */

      /* TODO: Fill values of coreset0Size, rbStart and rbLen */
      coreset0Size= rarPdcchInfo->coresetCfg.coreSetSize;
      rbStart = rarPdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.startPrb;
      rbLen = rarPdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coreset0Size - rbStart))
      {
	 if((rbLen - 1) <= floor(coreset0Size / 2))
	    freqDomResAssign = (coreset0Size * (rbLen-1)) + rbStart;
	 else
	    freqDomResAssign = (coreset0Size * (coreset0Size - rbLen + 1)) \
			       + (coreset0Size - 1 - rbStart);

	 freqDomResAssignSize = ceil(log2(coreset0Size * (coreset0Size + 1) / 2));
      }

      /* Fetching DCI field values */
      timeDomResAssign = rarPdcchInfo->dci[0].pdschCfg.pdschTimeAlloc.rowIndex;
      VRB2PRBMap       = rarPdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.vrbPrbMapping;
      modNCodScheme    = rarPdcchInfo->dci[0].pdschCfg.codeword[0].mcsIndex;
      tbScaling        = 0; /* configured to 0 scaling */
      reserved         = 0;

      /* Reversing bits in each DCI field */
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      tbScaling        = reverseBits(tbScaling, tbScalingSize); 

      /* Calulating total number of bytes in buffer */
      dlDciPtr->payloadSizeBits = freqDomResAssignSize + timeDomResAssignSize\
				  + VRB2PRBMapSize + modNCodSchemeSize + tbScalingSize + reservedSize;

      numBytes = dlDciPtr->payloadSizeBits / 8;
      if(dlDciPtr->payloadSizeBits % 8)
	 numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
	 DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
	 return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
	 dlDciPtr->payload[bytePos] = 0;

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    tbScaling, tbScalingSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
	    reserved, reservedSize);
   }
} /* fillRarDlDciPdu */

/*******************************************************************
 *
 * @brief fills DL DCI PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillDlMsgDlDciPdu
 *
 *    Functionality:
 *         -Fills the Dl DCI PDU  
 *
 * @params[in] Pointer to fapi_dl_dci_t
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
void fillDlMsgDlDciPdu(fapi_dl_dci_t *dlDciPtr, PdcchCfg *pdcchInfo,\
      DlMsgSchInfo *dlMsgSchInfo)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes;
      uint8_t bytePos;
      uint8_t bitPos;

      uint16_t coresetSize = 0;
      uint16_t rbStart = 0;
      uint16_t rbLen = 0;
      uint8_t  dciFormatId;
      uint32_t freqDomResAssign;
      uint8_t  timeDomResAssign;
      uint8_t  VRB2PRBMap;
      uint8_t  modNCodScheme;
      uint8_t  ndi = 0;
      uint8_t  redundancyVer = 0;
      uint8_t  harqProcessNum = 0;
      uint8_t  dlAssignmentIdx = 0;
      uint8_t  pucchTpc = 0;
      uint8_t  pucchResoInd = 0;
      uint8_t  harqFeedbackInd = 0;

      /* Size(in bits) of each field in DCI format 1_0 */
      uint8_t dciFormatIdSize    = 1;
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t ndiSize              = 1;
      uint8_t redundancyVerSize    = 2;
      uint8_t harqProcessNumSize   = 4;
      uint8_t dlAssignmentIdxSize  = 2;
      uint8_t pucchTpcSize         = 2;
      uint8_t pucchResoIndSize     = 3;
      uint8_t harqFeedbackIndSize  = 3;

      dlDciPtr->rnti = pdcchInfo->dci[0].rnti;
      dlDciPtr->scramblingId = pdcchInfo->dci[0].scramblingId;
      dlDciPtr->scramblingRnti = pdcchInfo->dci[0].scramblingRnti;
      dlDciPtr->cceIndex = pdcchInfo->dci[0].cceIndex;
      dlDciPtr->aggregationLevel = pdcchInfo->dci[0].aggregLevel;
      dlDciPtr->pc_and_bform.numPrgs = pdcchInfo->dci[0].beamPdcchInfo.numPrgs;
      dlDciPtr->pc_and_bform.prgSize = pdcchInfo->dci[0].beamPdcchInfo.prgSize;
      dlDciPtr->pc_and_bform.digBfInterfaces = pdcchInfo->dci[0].beamPdcchInfo.digBfInterfaces;
      dlDciPtr->pc_and_bform.pmi_bfi[0].pmIdx = pdcchInfo->dci[0].beamPdcchInfo.prg[0].pmIdx;
      dlDciPtr->pc_and_bform.pmi_bfi[0].beamIdx[0].beamidx = pdcchInfo->dci[0].beamPdcchInfo.prg[0].beamIdx[0];
      dlDciPtr->beta_pdcch_1_0 = pdcchInfo->dci[0].txPdcchPower.beta_pdcch_1_0;
      dlDciPtr->powerControlOffsetSS = pdcchInfo->dci[0].txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      coresetSize = pdcchInfo->coresetCfg.coreSetSize;
      rbStart = pdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.startPrb;
      rbLen = pdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coresetSize - rbStart))
      {
         if((rbLen - 1) <= floor(coresetSize / 2))
            freqDomResAssign = (coresetSize * (rbLen-1)) + rbStart;
         else
            freqDomResAssign = (coresetSize * (coresetSize - rbLen + 1)) \
                               + (coresetSize - 1 - rbStart);

         freqDomResAssignSize = ceil(log2(coresetSize * (coresetSize + 1) / 2));
      }

      /* Fetching DCI field values */
      dciFormatId      = dlMsgSchInfo->dciFormatId;     /* Always set to 1 for DL */
      timeDomResAssign = pdcchInfo->dci[0].pdschCfg.pdschTimeAlloc.rowIndex -1;
      VRB2PRBMap       = pdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.vrbPrbMapping;
      modNCodScheme    = pdcchInfo->dci[0].pdschCfg.codeword[0].mcsIndex;
      ndi              = dlMsgSchInfo->transportBlock[0].ndi;
      redundancyVer    = pdcchInfo->dci[0].pdschCfg.codeword[0].rvIndex;
      harqProcessNum   = dlMsgSchInfo->harqProcNum;
      dlAssignmentIdx  = dlMsgSchInfo->dlAssignIdx;
      pucchTpc         = dlMsgSchInfo->pucchTpc;
      pucchResoInd     = dlMsgSchInfo->pucchResInd;
      harqFeedbackInd  = dlMsgSchInfo->harqFeedbackInd;

      /* Reversing bits in each DCI field */
      dciFormatId      = reverseBits(dciFormatId, dciFormatIdSize);
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      ndi              = reverseBits(ndi, ndiSize);
      redundancyVer    = reverseBits(redundancyVer, redundancyVerSize);
      harqProcessNum   = reverseBits(harqProcessNum, harqProcessNumSize);
      dlAssignmentIdx  = reverseBits(dlAssignmentIdx , dlAssignmentIdxSize);
      pucchTpc         = reverseBits(pucchTpc, pucchTpcSize);
      pucchResoInd     = reverseBits(pucchResoInd, pucchResoIndSize);
      harqFeedbackInd  = reverseBits(harqFeedbackInd, harqFeedbackIndSize);


      /* Calulating total number of bytes in buffer */
      dlDciPtr->payloadSizeBits = (dciFormatIdSize + freqDomResAssignSize\
            + timeDomResAssignSize + VRB2PRBMapSize + modNCodSchemeSize\
            + ndiSize + redundancyVerSize + harqProcessNumSize + dlAssignmentIdxSize\
            + pucchTpcSize + pucchResoIndSize + harqFeedbackIndSize);

      numBytes = dlDciPtr->payloadSizeBits / 8;
      if(dlDciPtr->payloadSizeBits % 8)
         numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
         DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
         return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
         dlDciPtr->payload[bytePos] = 0;

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            dciFormatId, dciFormatIdSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            ndi, ndiSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            redundancyVer, redundancyVerSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            redundancyVer, redundancyVerSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            harqProcessNum, harqProcessNumSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            dlAssignmentIdx, dlAssignmentIdxSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            pucchTpc, pucchTpcSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            pucchResoInd, pucchResoIndSize);
      fillDlDciPayload(dlDciPtr->payload, &bytePos, &bitPos,\
            harqFeedbackInd, harqFeedbackIndSize);
   }
}

/*******************************************************************
 *
 * @brief fills Dl PDCCH Info from DL PageAlloc
 *
 * @details
 *
 *    Function : fillPdcchInfoFrmPageAlloc
 *
 *    Functionality:
 *         -Fills the PdcchInfo
 *
 * @params[in] Pointer to DlPageAlloc
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
void fillPagePdcchPdu(fapi_dl_tti_req_pdu_t *dlTtiReqPdu, fapi_vendor_dl_tti_req_pdu_t *dlTtiVendorPdu, DlPageAlloc *pageAlloc, MacCellCfg *macCellCfg)
{
   if(dlTtiReqPdu != NULLP)
   {
      BwpCfg *bwp = NULLP;

      memset(&dlTtiReqPdu->pdu.pdcch_pdu, 0, sizeof(fapi_dl_pdcch_pdu_t));
      bwp = &pageAlloc->bwp;
      fillPageDlDciPdu(dlTtiReqPdu->pdu.pdcch_pdu.dlDci, pageAlloc, macCellCfg);

      dlTtiReqPdu->pduType = PDCCH_PDU_TYPE;

      dlTtiReqPdu->pdu.pdcch_pdu.bwpSize           = bwp->freqAlloc.numPrb;
      dlTtiReqPdu->pdu.pdcch_pdu.bwpStart          = bwp->freqAlloc.startPrb;
      dlTtiReqPdu->pdu.pdcch_pdu.subCarrierSpacing = bwp->subcarrierSpacing;
      dlTtiReqPdu->pdu.pdcch_pdu.cyclicPrefix      = bwp->cyclicPrefix;

      dlTtiReqPdu->pdu.pdcch_pdu.startSymbolIndex    = pageAlloc->pageDlDci.ssStartSymbolIndex;
      dlTtiReqPdu->pdu.pdcch_pdu.durationSymbols     = pageAlloc->pageDlDci.durationSymbols;
      memcpy(dlTtiReqPdu->pdu.pdcch_pdu.freqDomainResource, pageAlloc->pageDlDci.freqDomainResource, 6*sizeof(uint8_t));
      dlTtiReqPdu->pdu.pdcch_pdu.cceRegMappingType   = pageAlloc->pageDlDci.cceRegMappingType;
      dlTtiReqPdu->pdu.pdcch_pdu.regBundleSize       = pageAlloc->pageDlDci.cceReg.interleaved.regBundleSize;
      dlTtiReqPdu->pdu.pdcch_pdu.interleaverSize     = pageAlloc->pageDlDci.cceReg.interleaved.interleaverSize;
      dlTtiReqPdu->pdu.pdcch_pdu.shiftIndex          = pageAlloc->pageDlDci.cceReg.interleaved.shiftIndex;
      dlTtiReqPdu->pdu.pdcch_pdu.precoderGranularity = pageAlloc->pageDlDci.precoderGranularity;
      dlTtiReqPdu->pdu.pdcch_pdu.numDlDci            = 1;
      dlTtiReqPdu->pdu.pdcch_pdu.coreSetType         = CORESET_TYPE0;

      /* Calculating PDU length. Considering only one dl dci pdu for now */
      dlTtiReqPdu->pduSize = sizeof(fapi_dl_pdcch_pdu_t);

      /* Filling Vendor message PDU */
      dlTtiVendorPdu->pdu_type = FAPI_PDCCH_PDU_TYPE;
      dlTtiVendorPdu->pdu_size = sizeof(fapi_vendor_dl_pdcch_pdu_t);
      dlTtiVendorPdu->pdu.pdcch_pdu.num_dl_dci = dlTtiReqPdu->pdu.pdcch_pdu.numDlDci;
      dlTtiVendorPdu->pdu.pdcch_pdu.dl_dci[0].epre_ratio_of_pdcch_to_ssb = 0;
      dlTtiVendorPdu->pdu.pdcch_pdu.dl_dci[0].epre_ratio_of_dmrs_to_ssb = 0;
   }
}

/*******************************************************************
 *
 * @brief fills PDCCH PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillPdcchPdu
 *
 *    Functionality:
 *         -Fills the Pdcch PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to FAPI DL TTI Req
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
uint8_t fillPdcchPdu(fapi_dl_tti_req_pdu_t *dlTtiReqPdu, fapi_vendor_dl_tti_req_pdu_t *dlTtiVendorPdu, MacDlSlot *dlSlot, int8_t dlMsgSchInfoIdx, \
      RntiType rntiType, uint8_t coreSetType, uint8_t ueIdx)
{
   uint8_t dciIndex = 0;

   if(dlTtiReqPdu != NULLP)
   {
      PdcchCfg *pdcchInfo = NULLP;
      BwpCfg *bwp = NULLP;

      memset(&dlTtiReqPdu->pdu.pdcch_pdu, 0, sizeof(fapi_dl_pdcch_pdu_t));
      if(rntiType == SI_RNTI_TYPE)
      {
         pdcchInfo = dlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg;
         bwp = &dlSlot->dlInfo.brdcstAlloc.sib1Alloc.bwp;
         fillSib1DlDciPdu(dlTtiReqPdu->pdu.pdcch_pdu.dlDci, pdcchInfo);
      }
      else if(rntiType == RA_RNTI_TYPE)
      {
         pdcchInfo = dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg;
         bwp = &dlSlot->dlInfo.rarAlloc[ueIdx]->bwp;
         fillRarDlDciPdu(dlTtiReqPdu->pdu.pdcch_pdu.dlDci, pdcchInfo);
      }
      else if(rntiType == TC_RNTI_TYPE || rntiType == C_RNTI_TYPE)
      {
         pdcchInfo = dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg;
         bwp = &dlSlot->dlInfo.dlMsgAlloc[ueIdx]->bwp;
         fillDlMsgDlDciPdu(dlTtiReqPdu->pdu.pdcch_pdu.dlDci, pdcchInfo,\
               dlSlot->dlInfo.dlMsgAlloc[ueIdx]);
      }
      else
      {
         DU_LOG("\nERROR  -->  LWR_MAC: Failed filling PDCCH Pdu");
         return RFAILED;
      }

      dlTtiReqPdu->pduType = PDCCH_PDU_TYPE;
      dlTtiReqPdu->pdu.pdcch_pdu.bwpSize = bwp->freqAlloc.numPrb;
      dlTtiReqPdu->pdu.pdcch_pdu.bwpStart = bwp->freqAlloc.startPrb;
      dlTtiReqPdu->pdu.pdcch_pdu.subCarrierSpacing = bwp->subcarrierSpacing; 
      dlTtiReqPdu->pdu.pdcch_pdu.cyclicPrefix = bwp->cyclicPrefix; 

      dlTtiReqPdu->pdu.pdcch_pdu.startSymbolIndex = pdcchInfo->coresetCfg.startSymbolIndex;
      dlTtiReqPdu->pdu.pdcch_pdu.durationSymbols = pdcchInfo->coresetCfg.durationSymbols;
      memcpy(dlTtiReqPdu->pdu.pdcch_pdu.freqDomainResource, pdcchInfo->coresetCfg.freqDomainResource, 6);
      dlTtiReqPdu->pdu.pdcch_pdu.cceRegMappingType = pdcchInfo->coresetCfg.cceRegMappingType;
      dlTtiReqPdu->pdu.pdcch_pdu.regBundleSize = pdcchInfo->coresetCfg.regBundleSize;
      dlTtiReqPdu->pdu.pdcch_pdu.interleaverSize = pdcchInfo->coresetCfg.interleaverSize;
      dlTtiReqPdu->pdu.pdcch_pdu.shiftIndex =  pdcchInfo->coresetCfg.shiftIndex;
      dlTtiReqPdu->pdu.pdcch_pdu.precoderGranularity = pdcchInfo->coresetCfg.precoderGranularity;
      dlTtiReqPdu->pdu.pdcch_pdu.numDlDci = pdcchInfo->numDlDci;
      dlTtiReqPdu->pdu.pdcch_pdu.coreSetType = coreSetType;

      /* Calculating PDU length. Considering only one dl dci pdu for now */
      dlTtiReqPdu->pduSize = sizeof(fapi_dl_pdcch_pdu_t);

      /* Filling Vendor message PDU */
      dlTtiVendorPdu->pdu_type = FAPI_PDCCH_PDU_TYPE;
      dlTtiVendorPdu->pdu_size = sizeof(fapi_vendor_dl_pdcch_pdu_t);
      dlTtiVendorPdu->pdu.pdcch_pdu.num_dl_dci = dlTtiReqPdu->pdu.pdcch_pdu.numDlDci;
      dlTtiVendorPdu->pdu.pdcch_pdu.dl_dci[0].epre_ratio_of_pdcch_to_ssb = 0;
      dlTtiVendorPdu->pdu.pdcch_pdu.dl_dci[0].epre_ratio_of_dmrs_to_ssb = 0;
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief fills PDSCH PDU from PageAlloc required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillPagePdschPdu
 *
 *    Functionality:
 *         -Fills the Pdsch PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to FAPI DL TTI Req
 *             Pointer to PdschCfg
 *             Pointer to msgLen of DL TTI Info
 * @return ROK
 *
 ******************************************************************/
void fillPagePdschPdu(fapi_dl_tti_req_pdu_t *dlTtiReqPdu, fapi_vendor_dl_tti_req_pdu_t *dlTtiVendorPdu, DlPageAlloc *pageAlloc,
                       uint16_t pduIndex, MacCellCfg *macCellCfg)
{
   uint8_t idx;

   if(dlTtiReqPdu != NULLP)
   {
      dlTtiReqPdu->pduType = PDSCH_PDU_TYPE;
      memset(&dlTtiReqPdu->pdu.pdsch_pdu, 0, sizeof(fapi_dl_pdsch_pdu_t));
      dlTtiReqPdu->pdu.pdsch_pdu.pduBitMap = 0; /* PTRS and CBG params are excluded */
      dlTtiReqPdu->pdu.pdsch_pdu.rnti = P_RNTI;
      dlTtiReqPdu->pdu.pdsch_pdu.pdu_index = pduIndex;
      dlTtiReqPdu->pdu.pdsch_pdu.bwpSize = pageAlloc->bwp.freqAlloc.numPrb;
      dlTtiReqPdu->pdu.pdsch_pdu.bwpStart = pageAlloc->bwp.freqAlloc.startPrb;
      dlTtiReqPdu->pdu.pdsch_pdu.subCarrierSpacing = pageAlloc->bwp.subcarrierSpacing;
      dlTtiReqPdu->pdu.pdsch_pdu.cyclicPrefix = pageAlloc->bwp.cyclicPrefix;
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfCodeWords = 1;
      for(idx = 0; idx < MAX_CODEWORDS ; idx++)
      { 
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].targetCodeRate = 308;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].qamModOrder = 2;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].mcsIndex = pageAlloc->pageDlSch.tbInfo.mcs;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].mcsTable = 0;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].rvIndex = 0;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].tbSize = pageAlloc->pageDlSch.tbInfo.tbSize;
      }
      dlTtiReqPdu->pdu.pdsch_pdu.dataScramblingId = macCellCfg->cellCfg.phyCellId;
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfLayers = 1;
      dlTtiReqPdu->pdu.pdsch_pdu.transmissionScheme = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.refPoint = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.dlDmrsSymbPos = DL_DMRS_SYMBOL_POS;
      dlTtiReqPdu->pdu.pdsch_pdu.dmrsConfigType = pageAlloc->pageDlSch.dmrs.dmrsType;
      dlTtiReqPdu->pdu.pdsch_pdu.dlDmrsScramblingId = macCellCfg->cellCfg.phyCellId;
      dlTtiReqPdu->pdu.pdsch_pdu.scid = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.numDmrsCdmGrpsNoData = 1;
      dlTtiReqPdu->pdu.pdsch_pdu.dmrsPorts = 0x0001;
      dlTtiReqPdu->pdu.pdsch_pdu.resourceAlloc = 1;
      /* since we are using type-1, hence rbBitmap excluded */
      dlTtiReqPdu->pdu.pdsch_pdu.rbStart = pageAlloc->pageDlSch.freqAlloc.startPrb;
      dlTtiReqPdu->pdu.pdsch_pdu.rbSize = pageAlloc->pageDlSch.freqAlloc.numPrb;
      dlTtiReqPdu->pdu.pdsch_pdu.vrbToPrbMapping = pageAlloc->pageDlSch.vrbPrbMapping;
      dlTtiReqPdu->pdu.pdsch_pdu.startSymbIndex = pageAlloc->pageDlSch.timeAlloc.startSymb;
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfSymbols = pageAlloc->pageDlSch.timeAlloc.numSymb;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.numPrgs = 1;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.prgSize = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.digBfInterfaces = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.pmi_bfi[0].pmIdx = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.pmi_bfi[0].beamIdx[0].beamidx = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.powerControlOffset = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.powerControlOffsetSS = 0;
      dlTtiReqPdu->pdu.pdsch_pdu.mappingType =   pageAlloc->pageDlSch.timeAlloc.mappingType;
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfDmrsSymbols = pageAlloc->pageDlSch.dmrs.nrOfDmrsSymbols;
      dlTtiReqPdu->pdu.pdsch_pdu.dmrsAddPos = pageAlloc->pageDlSch.dmrs.dmrsAddPos;

      dlTtiReqPdu->pduSize = sizeof(fapi_dl_pdsch_pdu_t);

      /* DL TTI Request vendor message */
      dlTtiVendorPdu->pdu_type = FAPI_PDSCH_PDU_TYPE;
      dlTtiVendorPdu->pdu_size = sizeof(fapi_vendor_dl_pdsch_pdu_t);
      dlTtiVendorPdu->pdu.pdsch_pdu.nr_of_antenna_ports = 1;
      for(int i =0; i< FAPI_VENDOR_MAX_TXRU_NUM; i++)
      {
	      dlTtiVendorPdu->pdu.pdsch_pdu.tx_ru_idx[i] =0;
      }
   }
}

/*******************************************************************
 *
 * @brief fills PDSCH PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : fillPdschPdu
 *
 *    Functionality:
 *         -Fills the Pdsch PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to FAPI DL TTI Req
 *             Pointer to PdschCfg
 *             Pointer to msgLen of DL TTI Info
 * @return ROK
 *
 ******************************************************************/

void fillPdschPdu(fapi_dl_tti_req_pdu_t *dlTtiReqPdu, fapi_vendor_dl_tti_req_pdu_t *dlTtiVendorPdu, PdschCfg *pdschInfo,
      BwpCfg bwp, uint16_t pduIndex)
{
   uint8_t idx;

   if(dlTtiReqPdu != NULLP)
   {
      dlTtiReqPdu->pduType = PDSCH_PDU_TYPE;
      memset(&dlTtiReqPdu->pdu.pdsch_pdu, 0, sizeof(fapi_dl_pdsch_pdu_t));
      dlTtiReqPdu->pdu.pdsch_pdu.pduBitMap = pdschInfo->pduBitmap;
      dlTtiReqPdu->pdu.pdsch_pdu.rnti = pdschInfo->rnti;         
      dlTtiReqPdu->pdu.pdsch_pdu.pdu_index = pduIndex;
      dlTtiReqPdu->pdu.pdsch_pdu.bwpSize = bwp.freqAlloc.numPrb;       
      dlTtiReqPdu->pdu.pdsch_pdu.bwpStart = bwp.freqAlloc.startPrb;
      dlTtiReqPdu->pdu.pdsch_pdu.subCarrierSpacing = bwp.subcarrierSpacing;
      dlTtiReqPdu->pdu.pdsch_pdu.cyclicPrefix = bwp.cyclicPrefix;
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfCodeWords = pdschInfo->numCodewords;
      for(idx = 0; idx < MAX_CODEWORDS ; idx++)
      { 
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].targetCodeRate = pdschInfo->codeword[idx].targetCodeRate;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].qamModOrder = pdschInfo->codeword[idx].qamModOrder;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].mcsIndex = pdschInfo->codeword[idx].mcsIndex;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].mcsTable = pdschInfo->codeword[idx].mcsTable;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].rvIndex = pdschInfo->codeword[idx].rvIndex;
         dlTtiReqPdu->pdu.pdsch_pdu.cwInfo[idx].tbSize = pdschInfo->codeword[idx].tbSize;
      }
      dlTtiReqPdu->pdu.pdsch_pdu.dataScramblingId = pdschInfo->dataScramblingId;       
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfLayers = pdschInfo->numLayers;
      dlTtiReqPdu->pdu.pdsch_pdu.transmissionScheme = pdschInfo->transmissionScheme;
      dlTtiReqPdu->pdu.pdsch_pdu.refPoint = pdschInfo->refPoint;
      dlTtiReqPdu->pdu.pdsch_pdu.dlDmrsSymbPos = pdschInfo->dmrs.dlDmrsSymbPos;
      dlTtiReqPdu->pdu.pdsch_pdu.dmrsConfigType = pdschInfo->dmrs.dmrsConfigType;
      dlTtiReqPdu->pdu.pdsch_pdu.dlDmrsScramblingId = pdschInfo->dmrs.dlDmrsScramblingId;
      dlTtiReqPdu->pdu.pdsch_pdu.scid = pdschInfo->dmrs.scid;
      dlTtiReqPdu->pdu.pdsch_pdu.numDmrsCdmGrpsNoData = pdschInfo->dmrs.numDmrsCdmGrpsNoData;
      dlTtiReqPdu->pdu.pdsch_pdu.dmrsPorts = pdschInfo->dmrs.dmrsPorts;
      dlTtiReqPdu->pdu.pdsch_pdu.resourceAlloc = pdschInfo->pdschFreqAlloc.resourceAllocType;
      /* since we are using type-1, hence rbBitmap excluded */
      dlTtiReqPdu->pdu.pdsch_pdu.rbStart = pdschInfo->pdschFreqAlloc.startPrb;
      dlTtiReqPdu->pdu.pdsch_pdu.rbSize = pdschInfo->pdschFreqAlloc.numPrb;
      dlTtiReqPdu->pdu.pdsch_pdu.vrbToPrbMapping = pdschInfo->pdschFreqAlloc.vrbPrbMapping;
      dlTtiReqPdu->pdu.pdsch_pdu.startSymbIndex = pdschInfo->pdschTimeAlloc.startSymb;
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfSymbols = pdschInfo->pdschTimeAlloc.numSymb;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.numPrgs = pdschInfo->beamPdschInfo.numPrgs;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.prgSize = pdschInfo->beamPdschInfo.prgSize;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.digBfInterfaces = pdschInfo->beamPdschInfo.digBfInterfaces;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.pmi_bfi[0]. \
         pmIdx = pdschInfo->beamPdschInfo.prg[0].pmIdx;
      dlTtiReqPdu->pdu.pdsch_pdu.preCodingAndBeamforming.pmi_bfi[0]. \
         beamIdx[0].beamidx = pdschInfo->beamPdschInfo.prg[0].beamIdx[0];
      dlTtiReqPdu->pdu.pdsch_pdu.powerControlOffset = pdschInfo->txPdschPower.powerControlOffset;  
      dlTtiReqPdu->pdu.pdsch_pdu.powerControlOffsetSS = pdschInfo->txPdschPower.powerControlOffsetSS;
      dlTtiReqPdu->pdu.pdsch_pdu.mappingType =   pdschInfo->dmrs.mappingType;
      dlTtiReqPdu->pdu.pdsch_pdu.nrOfDmrsSymbols = pdschInfo->dmrs.nrOfDmrsSymbols;
      dlTtiReqPdu->pdu.pdsch_pdu.dmrsAddPos = pdschInfo->dmrs.dmrsAddPos;

      dlTtiReqPdu->pduSize = sizeof(fapi_dl_pdsch_pdu_t);

      /* DL TTI Request vendor message */
      dlTtiVendorPdu->pdu_type = FAPI_PDSCH_PDU_TYPE;
      dlTtiVendorPdu->pdu_size = sizeof(fapi_vendor_dl_pdsch_pdu_t);
      dlTtiVendorPdu->pdu.pdsch_pdu.nr_of_antenna_ports = 1;
      for(int i =0; i< FAPI_VENDOR_MAX_TXRU_NUM; i++)
      {
	      dlTtiVendorPdu->pdu.pdsch_pdu.tx_ru_idx[i] =0;
      }
   }
}
#endif
/***********************************************************************
 *
 * @brief calculates the total size to be allocated for DL TTI Req
 *
 * @details
 *
 *    Function : calcDlTtiReqPduCount
 *
 *    Functionality:
 *         -calculates the total pdu count to be allocated for DL TTI Req
 *
 * @params[in]   MacDlSlot *dlSlot 
 * @return count
 *
 * ********************************************************************/
uint8_t calcDlTtiReqPduCount(MacDlSlot *dlSlot)
{
   uint8_t count = 0;
   uint8_t idx = 0, ueIdx=0;

   if(dlSlot->dlInfo.isBroadcastPres)
   {
      if(dlSlot->dlInfo.brdcstAlloc.ssbTransmissionMode)
      {
         for(idx = 0; idx < dlSlot->dlInfo.brdcstAlloc.ssbIdxSupported; idx++)
         {
            /* SSB PDU is filled */
            count++;
         }
      }
      if(dlSlot->dlInfo.brdcstAlloc.sib1TransmissionMode)
      {
         /* PDCCH and PDSCH PDU is filled */
         count += 2;
      }
   }

   if(dlSlot->pageAllocInfo)
   {
      /* PDCCH and PDSCH PDU is filled */
      count += 2;
   }

   for(ueIdx=0; ueIdx<MAX_NUM_UE; ueIdx++)
   {
      if(dlSlot->dlInfo.rarAlloc[ueIdx] != NULLP)
      {
         /* PDCCH and PDSCH PDU is filled */
         if(dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg && dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg)
            count += 2;
         else
            count += 1;
      }

      if(dlSlot->dlInfo.dlMsgAlloc[ueIdx] != NULLP)
      {
         /* PDCCH and PDSCH PDU is filled */
         if(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg)
            count += 1;
         if(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg)
            count += 1;
      }
   }
   return count;
}

/***********************************************************************
 *
 * @brief calculates the total size to be allocated for DL TTI Req
 *
 * @details
 *
 *    Function : calcTxDataReqPduCount
 *
 *    Functionality:
 *         -calculates the total pdu count to be allocated for DL TTI Req
 *
 * @params[in]    DlBrdcstAlloc *cellBroadcastInfo
 * @return count
 *
 * ********************************************************************/
uint8_t calcTxDataReqPduCount(MacDlSlot *dlSlot)
{
   uint8_t count = 0, ueIdx=0;

   if(dlSlot->dlInfo.isBroadcastPres && dlSlot->dlInfo.brdcstAlloc.sib1TransmissionMode)
   {
      count++;
   }
   if(dlSlot->pageAllocInfo)
   {
      count++;
   }

   for(ueIdx=0; ueIdx<MAX_NUM_UE; ueIdx++)
   {
      if((dlSlot->dlInfo.rarAlloc[ueIdx] != NULLP) &&  (dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg))
         count++;

      if(dlSlot->dlInfo.dlMsgAlloc[ueIdx] != NULLP)
      {
         if(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg)
            count++;
      }
   }
   return count;
}

#ifdef INTEL_FAPI
/***********************************************************************
 *
 * @brief fills the SIB1 TX-DATA request message
 *
 * @details
 *
 *    Function : fillSib1TxDataReq
 *
 *    Functionality:
 *         - fills the SIB1 TX-DATA request message
 *
 * @params[in]    fapi_tx_pdu_desc_t *pduDesc
 * @params[in]    macCellCfg consist of SIB1 pdu
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t fillSib1TxDataReq(fapi_tx_pdu_desc_t *pduDesc, uint16_t pduIndex, MacCellCfg *macCellCfg,
      PdschCfg *pdschCfg)
{
   uint32_t payloadSize = 0;
   uint8_t *sib1Payload = NULLP;
   fapi_api_queue_elem_t *payloadElem = NULLP;
#ifdef INTEL_WLS_MEM
   void * wlsHdlr = NULLP;
#endif

   pduDesc[pduIndex].pdu_index = pduIndex;
   pduDesc[pduIndex].num_tlvs = 1;

   /* fill the TLV */
   payloadSize = pdschCfg->codeword[0].tbSize;
   pduDesc[pduIndex].tlvs[0].tl.tag = ((payloadSize & 0xff0000) >> 8) | FAPI_TX_DATA_PTR_TO_PAYLOAD_64;
   pduDesc[pduIndex].tlvs[0].tl.length = (payloadSize & 0x0000ffff);
   LWR_MAC_ALLOC(sib1Payload, payloadSize);
   if(sib1Payload == NULLP)
   {
      return RFAILED;
   }
   payloadElem = (fapi_api_queue_elem_t *)sib1Payload;
   FILL_FAPI_LIST_ELEM(payloadElem, NULLP, FAPI_VENDOR_MSG_PHY_ZBC_BLOCK_REQ, 1, \
      macCellCfg->cellCfg.sib1Cfg.sib1PduLen);
   memcpy(sib1Payload + TX_PAYLOAD_HDR_LEN, macCellCfg->cellCfg.sib1Cfg.sib1Pdu, macCellCfg->cellCfg.sib1Cfg.sib1PduLen);

#ifdef INTEL_WLS_MEM
   mtGetWlsHdl(&wlsHdlr);
   pduDesc[pduIndex].tlvs[0].value = (uint8_t *)(WLS_VA2PA(wlsHdlr, sib1Payload));
#else
   pduDesc[pduIndex].tlvs[0].value = sib1Payload;
#endif
   pduDesc[pduIndex].pdu_length = payloadSize; 

#ifdef INTEL_WLS_MEM   
   addWlsBlockToFree(sib1Payload, payloadSize, (lwrMacCb.phySlotIndCntr-1));
#else
   LWR_MAC_FREE(sib1Payload, payloadSize);
#endif

   return ROK;
}

/***********************************************************************
 *
 * @brief fills the PAGE TX-DATA request message
 *
 * @details
 *
 *    Function : fillPageTxDataReq
 *
 *    Functionality:
 *         - fills the Page TX-DATA request message
 *
 * @params[in]    fapi_tx_pdu_desc_t *pduDesc
 * @params[in]    macCellCfg consist of SIB1 pdu
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t fillPageTxDataReq(fapi_tx_pdu_desc_t *pduDesc, uint16_t pduIndex, DlPageAlloc *pageAllocInfo)
{
   uint32_t payloadSize = 0;
   uint8_t *pagePayload = NULLP;
   fapi_api_queue_elem_t *payloadElem = NULLP;
#ifdef INTEL_WLS_MEM
   void * wlsHdlr = NULLP;
#endif

   pduDesc[pduIndex].pdu_index = pduIndex;
   pduDesc[pduIndex].num_tlvs = 1;

   /* fill the TLV */
   payloadSize = pageAllocInfo->pageDlSch.tbInfo.tbSize;
   pduDesc[pduIndex].tlvs[0].tl.tag = ((payloadSize & 0xff0000) >> 8) | FAPI_TX_DATA_PTR_TO_PAYLOAD_64;
   pduDesc[pduIndex].tlvs[0].tl.length = (payloadSize & 0x0000ffff);
   LWR_MAC_ALLOC(pagePayload, payloadSize);
   if(pagePayload == NULLP)
   {
      return RFAILED;
   }
   payloadElem = (fapi_api_queue_elem_t *)pagePayload;
   FILL_FAPI_LIST_ELEM(payloadElem, NULLP, FAPI_VENDOR_MSG_PHY_ZBC_BLOCK_REQ, 1, \
         pageAllocInfo->pageDlSch.dlPagePduLen);
   memcpy(pagePayload + TX_PAYLOAD_HDR_LEN, pageAllocInfo->pageDlSch.dlPagePdu, pageAllocInfo->pageDlSch.dlPagePduLen);

#ifdef INTEL_WLS_MEM
   mtGetWlsHdl(&wlsHdlr);
   pduDesc[pduIndex].tlvs[0].value = (uint8_t *)(WLS_VA2PA(wlsHdlr, pagePayload));
#else
   pduDesc[pduIndex].tlvs[0].value = pagePayload;
#endif
   pduDesc[pduIndex].pdu_length = payloadSize; 

#ifdef INTEL_WLS_MEM   
   addWlsBlockToFree(pagePayload, payloadSize, (lwrMacCb.phySlotIndCntr-1));
#else
   LWR_MAC_FREE(pagePayload, payloadSize);
#endif

   return ROK;
}

/***********************************************************************
 *
 * @brief fills the RAR TX-DATA request message
 *
 * @details
 *
 *    Function : fillRarTxDataReq
 *
 *    Functionality:
 *         - fills the RAR TX-DATA request message
 *
 * @params[in]    fapi_tx_pdu_desc_t *pduDesc
 * @params[in]    RarInfo *rarInfo
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t fillRarTxDataReq(fapi_tx_pdu_desc_t *pduDesc, uint16_t pduIndex, RarInfo *rarInfo, PdschCfg *pdschCfg)
{
   uint16_t payloadSize;
   uint8_t  *rarPayload = NULLP;
   fapi_api_queue_elem_t *payloadElem = NULLP;
#ifdef INTEL_WLS_MEM
   void * wlsHdlr = NULLP;
#endif

   pduDesc[pduIndex].pdu_index = pduIndex;
   pduDesc[pduIndex].num_tlvs = 1;

   /* fill the TLV */
   payloadSize = pdschCfg->codeword[0].tbSize;
   pduDesc[pduIndex].tlvs[0].tl.tag = FAPI_TX_DATA_PTR_TO_PAYLOAD_64;
   pduDesc[pduIndex].tlvs[0].tl.length = payloadSize;
   LWR_MAC_ALLOC(rarPayload, payloadSize);
   if(rarPayload == NULLP)
   {
      return RFAILED;
   }
   payloadElem = (fapi_api_queue_elem_t *)rarPayload;
   FILL_FAPI_LIST_ELEM(payloadElem, NULLP, FAPI_VENDOR_MSG_PHY_ZBC_BLOCK_REQ, 1, rarInfo->rarPduLen);
   memcpy(rarPayload + TX_PAYLOAD_HDR_LEN, rarInfo->rarPdu, rarInfo->rarPduLen);

#ifdef INTEL_WLS_MEM
   mtGetWlsHdl(&wlsHdlr);
   pduDesc[pduIndex].tlvs[0].value = (uint8_t *)(WLS_VA2PA(wlsHdlr, rarPayload));
#else
   pduDesc[pduIndex].tlvs[0].value = rarPayload;
#endif
   pduDesc[pduIndex].pdu_length = payloadSize;

#ifdef INTEL_WLS_MEM
   addWlsBlockToFree(rarPayload, payloadSize, (lwrMacCb.phySlotIndCntr-1));
#else
   LWR_MAC_FREE(rarPayload, payloadSize);
#endif
   return ROK;
}

/***********************************************************************
 *
 * @brief fills the DL dedicated Msg TX-DATA request message
 *
 * @details
 *
 *    Function : fillDlMsgTxDataReq
 *
 *    Functionality:
 *         - fills the Dl Dedicated Msg TX-DATA request message
 *
 * @params[in]    fapi_tx_pdu_desc_t *pduDesc
 * @params[in]    DlMsgInfo *dlMsgInfo
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t fillDlMsgTxDataReq(fapi_tx_pdu_desc_t *pduDesc, uint16_t pduIndex, DlMsgSchInfo *dlMsgSchInfo, PdschCfg *pdschCfg)
{
   uint16_t payloadSize;
   uint8_t  *dlMsgPayload = NULLP;
   fapi_api_queue_elem_t *payloadElem = NULLP;
#ifdef INTEL_WLS_MEM
   void * wlsHdlr = NULLP;
#endif

   pduDesc[pduIndex].pdu_index = pduIndex;
   pduDesc[pduIndex].num_tlvs = 1;

   /* fill the TLV */
   payloadSize = pdschCfg->codeword[0].tbSize;
   pduDesc[pduIndex].tlvs[0].tl.tag = FAPI_TX_DATA_PTR_TO_PAYLOAD_64;
   pduDesc[pduIndex].tlvs[0].tl.length = payloadSize;
   LWR_MAC_ALLOC(dlMsgPayload, payloadSize);
   if(dlMsgPayload == NULLP)
   {
      return RFAILED;
   }
   payloadElem = (fapi_api_queue_elem_t *)dlMsgPayload;
   FILL_FAPI_LIST_ELEM(payloadElem, NULLP, FAPI_VENDOR_MSG_PHY_ZBC_BLOCK_REQ, 1, dlMsgSchInfo->dlMsgPduLen);
   memcpy(dlMsgPayload + TX_PAYLOAD_HDR_LEN, dlMsgSchInfo->dlMsgPdu, dlMsgSchInfo->dlMsgPduLen);

#ifdef INTEL_WLS_MEM
   mtGetWlsHdl(&wlsHdlr);
   pduDesc[pduIndex].tlvs[0].value = (uint8_t *)(WLS_VA2PA(wlsHdlr, dlMsgPayload));
#else
   pduDesc[pduIndex].tlvs[0].value = dlMsgPayload;
#endif
   pduDesc[pduIndex].pdu_length = payloadSize;

#ifdef INTEL_WLS_MEM
   addWlsBlockToFree(dlMsgPayload, payloadSize, (lwrMacCb.phySlotIndCntr-1));
#else
   LWR_MAC_FREE(dlMsgPayload, payloadSize);
#endif
   return ROK;
}

#endif /* FAPI */

/*******************************************************************
 *
 * @brief fills PDCCH PDU required for nFAPI DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillPdcchPdu
 *
 *    Functionality:
 *         -Fills the Pdcch PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to nFAPI DL TTI Req
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
uint8_t OAI_OSC_fillPdcchPdu(nfapi_nr_dl_tti_request_pdu_t *dlTtiReqPdu, MacDlSlot *dlSlot, int8_t dlMsgSchInfoIdx, \
      RntiType rntiType, uint8_t coreSetType, uint8_t ueIdx)
{
   if(dlTtiReqPdu != NULLP)
   {
      PdcchCfg *pdcchInfo = NULLP;
      BwpCfg *bwp = NULLP;

      memset(&dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15, 0, sizeof(nfapi_nr_dl_tti_pdcch_pdu_rel15_t));
      if(rntiType == SI_RNTI_TYPE)
      {
         pdcchInfo = dlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg;
         bwp = &dlSlot->dlInfo.brdcstAlloc.sib1Alloc.bwp;
         OAI_OSC_fillSib1DlDciPdu(dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.dci_pdu, pdcchInfo);
      }
      else if(rntiType == RA_RNTI_TYPE)
      {
         pdcchInfo = dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg;
         bwp = &dlSlot->dlInfo.rarAlloc[ueIdx]->bwp;
         OAI_OSC_fillRarDlDciPdu(dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.dci_pdu, pdcchInfo);
      }
      else if(rntiType == TC_RNTI_TYPE || rntiType == C_RNTI_TYPE)
      {
         pdcchInfo = dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg;
         bwp = &dlSlot->dlInfo.dlMsgAlloc[ueIdx]->bwp;
         OAI_OSC_fillDlMsgDlDciPdu(dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.dci_pdu, pdcchInfo,\
               dlSlot->dlInfo.dlMsgAlloc[ueIdx]);
      }
      else
      {
         DU_LOG("\nERROR  -->  LWR_MAC: Failed filling PDCCH Pdu");
         return RFAILED;
      }
      
      dlTtiReqPdu->PDUType = PDCCH_PDU_TYPE;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.BWPSize = bwp->freqAlloc.numPrb;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.BWPStart = bwp->freqAlloc.startPrb;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.SubcarrierSpacing = bwp->subcarrierSpacing; 
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.CyclicPrefix = bwp->cyclicPrefix; 

      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.StartSymbolIndex = pdcchInfo->coresetCfg.startSymbolIndex;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.DurationSymbols = pdcchInfo->coresetCfg.durationSymbols;
      memcpy(dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.FreqDomainResource, pdcchInfo->coresetCfg.freqDomainResource, 6);
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.CceRegMappingType = pdcchInfo->coresetCfg.cceRegMappingType;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.RegBundleSize = pdcchInfo->coresetCfg.regBundleSize;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.InterleaverSize = pdcchInfo->coresetCfg.interleaverSize;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.ShiftIndex =  pdcchInfo->coresetCfg.shiftIndex;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.precoderGranularity = pdcchInfo->coresetCfg.precoderGranularity;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.numDlDci = pdcchInfo->numDlDci;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.CoreSetType = coreSetType;

      /* Calculating PDU length. Considering only one dl dci pdu for now */
      dlTtiReqPdu->PDUSize = sizeof(nfapi_nr_dl_tti_pdcch_pdu_rel15_t);
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief fills PDSCH PDU required for nFPAI DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillPdschPdu
 *
 *    Functionality:
 *         -Fills the Pdsch PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to nFAPI DL TTI Req
 *             Pointer to PdschCfg
 *             Pointer to msgLen of DL TTI Info
 * @return ROK
 *
 ******************************************************************/
void OAI_OSC_fillPdschPdu(nfapi_nr_dl_tti_request_pdu_t *dlTtiReqPdu, PdschCfg *pdschInfo,
BwpCfg bwp, uint16_t pduIndex)
{
   uint8_t idx;
   if(dlTtiReqPdu != NULLP)
   {
         dlTtiReqPdu->PDUType = PDSCH_PDU_TYPE;
   memset(&dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15, 0, sizeof(nfapi_nr_dl_tti_pdsch_pdu_rel15_t));
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.pduBitmap = pdschInfo->pduBitmap;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rnti = pdschInfo->rnti;         
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.pduIndex = pduIndex;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.BWPSize = bwp.freqAlloc.numPrb;       
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.BWPStart = bwp.freqAlloc.startPrb;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.SubcarrierSpacing = bwp.subcarrierSpacing;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.CyclicPrefix = bwp.cyclicPrefix;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.NrOfCodewords = pdschInfo->numCodewords;
   for(idx = 0; idx < MAX_CODEWORDS ; idx++)
   {
            dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.targetCodeRate[idx] = pdschInfo->codeword[idx].targetCodeRate;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.qamModOrder[idx] = pdschInfo->codeword[idx].qamModOrder;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.mcsIndex[idx] = pdschInfo->codeword[idx].mcsIndex;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.mcsTable[idx] = pdschInfo->codeword[idx].mcsTable;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rvIndex[idx] = pdschInfo->codeword[idx].rvIndex;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.TBSize[idx] = pdschInfo->codeword[idx].tbSize;
   }
         dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dataScramblingId = pdschInfo->dataScramblingId;       
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.nrOfLayers = pdschInfo->numLayers;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.transmissionScheme = pdschInfo->transmissionScheme;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.refPoint = pdschInfo->refPoint;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dlDmrsSymbPos = pdschInfo->dmrs.dlDmrsSymbPos;

/* ======== small cell integration ======== */
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.maintenance_parms_v3.ldpcBaseGraph = 2;
/* ======================================== */

   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dmrsConfigType = pdschInfo->dmrs.dmrsConfigType;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dlDmrsScramblingId = pdschInfo->dmrs.dlDmrsScramblingId;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.SCID = pdschInfo->dmrs.scid;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.numDmrsCdmGrpsNoData = pdschInfo->dmrs.numDmrsCdmGrpsNoData;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dmrsPorts = pdschInfo->dmrs.dmrsPorts;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.resourceAlloc = pdschInfo->pdschFreqAlloc.resourceAllocType;
   /* since we are using type-1, hence rbBitmap excluded */
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rbStart = pdschInfo->pdschFreqAlloc.startPrb;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rbSize = pdschInfo->pdschFreqAlloc.numPrb;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.VRBtoPRBMapping = pdschInfo->pdschFreqAlloc.vrbPrbMapping;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.StartSymbolIndex = pdschInfo->pdschTimeAlloc.startSymb;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.NrOfSymbols = pdschInfo->pdschTimeAlloc.numSymb;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.num_prgs = pdschInfo->beamPdschInfo.numPrgs;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.prg_size = pdschInfo->beamPdschInfo.prgSize;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.dig_bf_interfaces = pdschInfo->beamPdschInfo.digBfInterfaces;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.prgs_list[0]. \
   pm_idx = pdschInfo->beamPdschInfo.prg[0].pmIdx;
   dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.prgs_list[0]. \
   dig_bf_interface_list[0].beam_idx = pdschInfo->beamPdschInfo.prg[0].beamIdx[0];
   /*Not changed yet*/
   //dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.powerControlOffset = pdschInfo->txPdschPower.powerControlOffset;  
   //dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.powerControlOffsetSS = pdschInfo->txPdschPower.powerControlOffsetSS;
   //dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.mappingType =   pdschInfo->dmrs.mappingType;
   //dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.nrOfDmrsSymbols = pdschInfo->dmrs.nrOfDmrsSymbols;
   //dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dmrsAddPos = pdschInfo->dmrs.dmrsAddPos;

   dlTtiReqPdu->PDUSize = sizeof(nfapi_nr_dl_tti_pdsch_pdu_rel15_t);
   }
}

/***********************************************************************
 *
 * @brief Fills the PRACH PDU in nFAPI UL TTI Request
 *
 * @details
 *
 *    Function : OAI_OSC_fillPrachPdu
 *
 *    Functionality:
 *         -Fills the PRACH PDU in nFAPI UL TTI Request
 *
 * @params[in] Pointer to Prach Pdu
 *             Pointer to CurrUlSlot
 *             Pointer to macCellCfg
 *             Pointer to msgLen
 * ********************************************************************/
void OAI_OSC_fillPrachPdu(nfapi_nr_ul_tti_request_number_of_pdus_t *ulTtiReqPdu, MacCellCfg *macCellCfg, MacUlSlot *currUlSlot)
{
   DU_LOG("\nDEBUG  --> OAI_OSC_fillPrachPdu()\n");
   if(ulTtiReqPdu != NULLP)
   {
      ulTtiReqPdu->pdu_type = PRACH_PDU_TYPE; 
      ulTtiReqPdu->prach_pdu.phys_cell_id = macCellCfg->cellCfg.phyCellId;
      ulTtiReqPdu->prach_pdu.num_prach_ocas = \
         currUlSlot->ulInfo.prachSchInfo.numPrachOcas;
      ulTtiReqPdu->prach_pdu.prach_format = \
	 currUlSlot->ulInfo.prachSchInfo.prachFormat;
      ulTtiReqPdu->prach_pdu.num_ra = currUlSlot->ulInfo.prachSchInfo.numRa;
      ulTtiReqPdu->prach_pdu.prach_start_symbol = \
	 currUlSlot->ulInfo.prachSchInfo.prachStartSymb;
      setNumCs(&ulTtiReqPdu->prach_pdu.num_cs, macCellCfg);
      DU_LOG("\nDEBUG  -->  ulTtiReqPdu->prach_pdu.num_cs:%d\n", ulTtiReqPdu->prach_pdu.num_cs);
      ulTtiReqPdu->prach_pdu.beamforming.num_prgs = 0;
      ulTtiReqPdu->prach_pdu.beamforming.prg_size = 0;
      ulTtiReqPdu->prach_pdu.beamforming.dig_bf_interface = 0;
      ulTtiReqPdu->prach_pdu.beamforming.prgs_list->dig_bf_interface_list->beam_idx = 0;
      ulTtiReqPdu->pdu_size = sizeof(nfapi_nr_prach_pdu_t); 
   }
}

/*******************************************************************
 *
 * @brief Filling PUSCH PDU in nFAPI UL TTI Request
 *
 * @details
 *
 *    Function : OAI_OSC_fillPuschPdu
 *
 *    Functionality: Filling PUSCH PDU in nFAPI UL TTI Request
 *
 * @params[in] 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
void OAI_OSC_fillPuschPdu(nfapi_nr_ul_tti_request_number_of_pdus_t *ulTtiReqPdu, MacCellCfg *macCellCfg, MacUlSlot *currUlSlot)
{
   if(ulTtiReqPdu != NULLP)
   {
      ulTtiReqPdu->pdu_type = PUSCH_PDU_TYPE;
      memset(&ulTtiReqPdu->pusch_pdu, 0, sizeof(nfapi_nr_pusch_pdu_t));
      ulTtiReqPdu->pusch_pdu.pdu_bit_map = 1;
      ulTtiReqPdu->pusch_pdu.rnti = currUlSlot->ulInfo.crnti;
      /* TODO : Fill handle in raCb when scheduling pusch and access here */
      ulTtiReqPdu->pusch_pdu.handle = 100;
      ulTtiReqPdu->pusch_pdu.bwp_size = macCellCfg->cellCfg.initialUlBwp.bwp.numPrb;
      ulTtiReqPdu->pusch_pdu.bwp_start = macCellCfg->cellCfg.initialUlBwp.bwp.firstPrb;
      ulTtiReqPdu->pusch_pdu.subcarrier_spacing = \
         macCellCfg->cellCfg.initialUlBwp.bwp.scs;
      ulTtiReqPdu->pusch_pdu.cyclic_prefix = \
         macCellCfg->cellCfg.initialUlBwp.bwp.cyclicPrefix;
      ulTtiReqPdu->pusch_pdu.target_code_rate = 308;
      ulTtiReqPdu->pusch_pdu.qam_mod_order = currUlSlot->ulInfo.schPuschInfo.tbInfo.qamOrder;
      ulTtiReqPdu->pusch_pdu.mcs_index = currUlSlot->ulInfo.schPuschInfo.tbInfo.mcs;
      ulTtiReqPdu->pusch_pdu.mcs_table = currUlSlot->ulInfo.schPuschInfo.tbInfo.mcsTable;
      ulTtiReqPdu->pusch_pdu.transform_precoding = 1;
      ulTtiReqPdu->pusch_pdu.data_scrambling_id = currUlSlot->ulInfo.cellId;
      ulTtiReqPdu->pusch_pdu.nrOfLayers = 1;
      ulTtiReqPdu->pusch_pdu.ul_dmrs_symb_pos = 4;
      ulTtiReqPdu->pusch_pdu.dmrs_config_type = 0;
      ulTtiReqPdu->pusch_pdu.ul_dmrs_scrambling_id = currUlSlot->ulInfo.cellId;
      ulTtiReqPdu->pusch_pdu.scid = 0;
      ulTtiReqPdu->pusch_pdu.num_dmrs_cdm_grps_no_data = 1;
      ulTtiReqPdu->pusch_pdu.dmrs_ports = 0;
      ulTtiReqPdu->pusch_pdu.resource_alloc = \
	 currUlSlot->ulInfo.schPuschInfo.fdAlloc.resAllocType;
      ulTtiReqPdu->pusch_pdu.rb_start = \
         currUlSlot->ulInfo.schPuschInfo.fdAlloc.resAlloc.type1.startPrb;
      ulTtiReqPdu->pusch_pdu.rb_size = \
	 currUlSlot->ulInfo.schPuschInfo.fdAlloc.resAlloc.type1.numPrb;
      ulTtiReqPdu->pusch_pdu.vrb_to_prb_mapping = 0;
      ulTtiReqPdu->pusch_pdu.frequency_hopping = 0;
      ulTtiReqPdu->pusch_pdu.tx_direct_current_location = 0;
      ulTtiReqPdu->pusch_pdu.uplink_frequency_shift_7p5khz = 0;
      ulTtiReqPdu->pusch_pdu.start_symbol_index = \
         currUlSlot->ulInfo.schPuschInfo.tdAlloc.startSymb;
      ulTtiReqPdu->pusch_pdu.nr_of_symbols = \
         currUlSlot->ulInfo.schPuschInfo.tdAlloc.numSymb;

      ulTtiReqPdu->pusch_pdu.pusch_data.rv_index = \
         currUlSlot->ulInfo.schPuschInfo.tbInfo.rv;
      ulTtiReqPdu->pusch_pdu.pusch_data.harq_process_id = \
         currUlSlot->ulInfo.schPuschInfo.harqProcId;
      ulTtiReqPdu->pusch_pdu.pusch_data.new_data_indicator = \
         currUlSlot->ulInfo.schPuschInfo.tbInfo.ndi;
      ulTtiReqPdu->pusch_pdu.pusch_data.tb_size = \
         currUlSlot->ulInfo.schPuschInfo.tbInfo.tbSize;
      /* numCb is 0 for new transmission */
      ulTtiReqPdu->pusch_pdu.pusch_data.num_cb = 0;

      ulTtiReqPdu->pdu_size = sizeof(nfapi_nr_pusch_pdu_t);
   }
}

/*******************************************************************
 *
 * @brief Fill PUCCH PDU in nFAPI Ul TTI Request
 *
 * @details
 *
 *    Function : OAI_OSC_fillPucchPdu
 *
 *    Functionality: Fill PUCCH PDU in nFAPI Ul TTI Request
 *
 * @params[in]
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
void OAI_OSC_fillPucchPdu(nfapi_nr_ul_tti_request_number_of_pdus_t *ulTtiReqPdu, MacCellCfg *macCellCfg, MacUlSlot *currUlSlot){
  if (ulTtiReqPdu != NULLP) {
    ulTtiReqPdu->pdu_type = PUCCH_PDU_TYPE;
    memset(&ulTtiReqPdu->pucch_pdu, 0, sizeof(nfapi_nr_pucch_pdu_t));
    ulTtiReqPdu->pucch_pdu.rnti = currUlSlot->ulInfo.crnti;
    /* TODO : Fill handle in raCb when scheduling pucch and access here */
    ulTtiReqPdu->pucch_pdu.handle = 100;
    ulTtiReqPdu->pucch_pdu.bwp_size =
        macCellCfg->cellCfg.initialUlBwp.bwp.numPrb;
    ulTtiReqPdu->pucch_pdu.bwp_start =
        macCellCfg->cellCfg.initialUlBwp.bwp.firstPrb;
    ulTtiReqPdu->pucch_pdu.subcarrier_spacing =
        macCellCfg->cellCfg.initialUlBwp.bwp.scs;
    ulTtiReqPdu->pucch_pdu.cyclic_prefix =
        macCellCfg->cellCfg.initialUlBwp.bwp.cyclicPrefix;
    ulTtiReqPdu->pucch_pdu.format_type =
        currUlSlot->ulInfo.schPucchInfo
            .pucchFormat; /* Supporting PUCCH Format 0 */
    ulTtiReqPdu->pucch_pdu.multi_slot_tx_indicator =
        0; /* No Multi Slot transmission */

    ulTtiReqPdu->pucch_pdu.prb_start =
        currUlSlot->ulInfo.schPucchInfo.fdAlloc.startPrb;
    ulTtiReqPdu->pucch_pdu.prb_size =
        currUlSlot->ulInfo.schPucchInfo.fdAlloc.numPrb;
    ulTtiReqPdu->pucch_pdu.start_symbol_index =
        currUlSlot->ulInfo.schPucchInfo.tdAlloc.startSymb;
    ulTtiReqPdu->pucch_pdu.nr_of_symbols =
        currUlSlot->ulInfo.schPucchInfo.tdAlloc.numSymb;
    ulTtiReqPdu->pucch_pdu.freq_hop_flag =
        currUlSlot->ulInfo.schPucchInfo.intraFreqHop;
    ulTtiReqPdu->pucch_pdu.second_hop_prb =
        currUlSlot->ulInfo.schPucchInfo.secondPrbHop;
    ulTtiReqPdu->pucch_pdu.group_hop_flag = 0;
    ulTtiReqPdu->pucch_pdu.sequence_hop_flag = 0;
    ulTtiReqPdu->pucch_pdu.hopping_id = 0;

    ulTtiReqPdu->pucch_pdu.initial_cyclic_shift =
        currUlSlot->ulInfo.schPucchInfo.initialCyclicShift;

    ulTtiReqPdu->pucch_pdu.data_scrambling_id =
        0; /* Valid for Format 2, 3, 4 */
    ulTtiReqPdu->pucch_pdu.time_domain_occ_idx =
        currUlSlot->ulInfo.schPucchInfo.timeDomOCC;
    ulTtiReqPdu->pucch_pdu.pre_dft_occ_idx =
        currUlSlot->ulInfo.schPucchInfo.occIdx; /* Valid for Format 4 only */
    ulTtiReqPdu->pucch_pdu.pre_dft_occ_len =
        currUlSlot->ulInfo.schPucchInfo.occLen; /* Valid for Format 4 only */
    ulTtiReqPdu->pucch_pdu.pi_2bpsk =
        currUlSlot->ulInfo.schPucchInfo.pi2BPSK;
    ulTtiReqPdu->pucch_pdu.add_dmrs_flag =
        currUlSlot->ulInfo.schPucchInfo
            .addDmrs; /* Valid for Format 3, 4 only */
    ulTtiReqPdu->pucch_pdu.dmrs_scrambling_id = 0; /* Valid for Format 2 */
    ulTtiReqPdu->pucch_pdu.dmrs_cyclic_shift = 0;  /* Valid for Format 4 */
    ulTtiReqPdu->pucch_pdu.sr_flag = currUlSlot->ulInfo.schPucchInfo.srFlag;
    ulTtiReqPdu->pucch_pdu.bit_len_harq =
        currUlSlot->ulInfo.schPucchInfo.harqInfo.harqBitLength;
    ulTtiReqPdu->pucch_pdu.bit_len_csi_part1 =
        0; /* Valid for Format 2, 3, 4 */
    ulTtiReqPdu->pucch_pdu.bit_len_csi_part2 =
        0; /* Valid for Format 2, 3, 4 */
    ulTtiReqPdu->pucch_pdu.beamforming.num_prgs =
        currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.numPrgs;
    ulTtiReqPdu->pucch_pdu.beamforming.prg_size =
        currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.prgSize;
    ulTtiReqPdu->pucch_pdu.beamforming.dig_bf_interface =
        currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.digBfInterfaces;
    ulTtiReqPdu->pucch_pdu.beamforming.prgs_list->dig_bf_interface_list->beam_idx =
        currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.prg[0].beamIdx[0];

    ulTtiReqPdu->pdu_size = sizeof(nfapi_nr_pucch_pdu_t);
   
    /* UL TTI Vendor PDU 
    ulTtiVendorPdu->pdu_type = FAPI_PUCCH_PDU_TYPE;
    ulTtiVendorPdu->pdu.pucch_pdu.nr_of_rx_ru = 1;
    ulTtiVendorPdu->pdu.pucch_pdu.group_id = 0;
    for (int i = 0; i < FAPI_VENDOR_MAX_RXRU_NUM; i++) {
      ulTtiVendorPdu->pdu.pucch_pdu.rx_ru_idx[i] = 0;
    }*/
  }
}

/*******************************************************************
 *
 * @brief fills PDCCH PDU required for nFAPI UL DCI REQ to OAI PHY
 *
 * @details
 *
 *    Function : OAI_OSC_fillUlDciPdcchPdu
 *
 *    Functionality:
 *         -Fills the Pdcch PDU info
 *
 * @params[in] Pointer to nFAPI DL TTI Req
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
uint8_t OAI_OSC_fillUlDciPdcchPdu(nfapi_nr_ul_dci_request_pdus_t *ulDciReqPdu, DlSchedInfo *dlInfo, uint8_t coreSetType)
{
   DU_LOG("\nDEBUG  -->  %s()\n", __FUNCTION__);
   if(ulDciReqPdu != NULLP)
   {
      memset(&ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15, 0, sizeof(nfapi_nr_dl_tti_pdcch_pdu_rel15_t));
      OAI_OSC_fillUlDciPdu(ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.dci_pdu, dlInfo->ulGrant);
      ulDciReqPdu->PDUType                          = PDCCH_PDU_TYPE;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.BWPSize           = dlInfo->ulGrant->bwpCfg.freqAlloc.numPrb;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.BWPStart          = dlInfo->ulGrant->bwpCfg.freqAlloc.startPrb;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.SubcarrierSpacing = dlInfo->ulGrant->bwpCfg.subcarrierSpacing; 
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.CyclicPrefix      = dlInfo->ulGrant->bwpCfg.cyclicPrefix; 
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.StartSymbolIndex  = dlInfo->ulGrant->coresetCfg.startSymbolIndex;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.DurationSymbols   = dlInfo->ulGrant->coresetCfg.durationSymbols;
      memcpy(ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.FreqDomainResource, dlInfo->ulGrant->coresetCfg.freqDomainResource, 6);
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.CceRegMappingType = dlInfo->ulGrant->coresetCfg.cceRegMappingType;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.RegBundleSize     = dlInfo->ulGrant->coresetCfg.regBundleSize;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.InterleaverSize   = dlInfo->ulGrant->coresetCfg.interleaverSize;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.ShiftIndex        = dlInfo->ulGrant->coresetCfg.shiftIndex;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.precoderGranularity = dlInfo->ulGrant->coresetCfg.precoderGranularity;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.numDlDci          = 1;
      ulDciReqPdu->pdcch_pdu.pdcch_pdu_rel15.CoreSetType       = coreSetType;

      /* Calculating PDU length. Considering only one Ul dci pdu for now */
      ulDciReqPdu->PDUSize = sizeof(nfapi_nr_dl_tti_pdcch_pdu_rel15_t);
   }
   return ROK;
}

/***********************************************************************
 *
 * @brief fills the SIB1 nFAPI TX-DATA request message
 *
 * @details
 *
 *    Function : OAI_OSC_fillSib1TxDataReq
 *
 *    Functionality:
 *         - fills the SIB1 nFAPI TX-DATA request message
 *
 * @params[in]    nfapi_nr_pdu_t *pduDesc
 * @params[in]    macCellCfg consist of SIB1 pdu
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t OAI_OSC_fillSib1TxDataReq(nfapi_nr_pdu_t *pduDesc, uint16_t pduIndex, MacCellCfg *macCellCfg,
      PdschCfg *pdschCfg)
{
   uint32_t payloadSize = 0;
   uint8_t *sib1Payload = NULLP;

   pduDesc[pduIndex].PDU_index = pduIndex;
   pduDesc[pduIndex].num_TLV = 1;

   /* fill the TLV */
   payloadSize = pdschCfg->codeword[0].tbSize;
   pduDesc[pduIndex].TLVs[0].tag = FAPI_TX_DATA_PAYLOAD; //pack direct
   pduDesc[pduIndex].TLVs[0].length = payloadSize;
   LWR_MAC_ALLOC(sib1Payload, payloadSize);
   if(sib1Payload == NULLP)
   {
      return RFAILED;
   }
   memcpy(sib1Payload, macCellCfg->cellCfg.sib1Cfg.sib1Pdu, macCellCfg->cellCfg.sib1Cfg.sib1PduLen);
   
   memcpy(pduDesc[pduIndex].TLVs[0].value.direct, sib1Payload, payloadSize);

   pduDesc[pduIndex].PDU_length = payloadSize; 

   LWR_MAC_FREE(sib1Payload, payloadSize);
   return ROK;
}

/***********************************************************************
 *
 * @brief fills the PAGE nFAPI TX-DATA request message
 *
 * @details
 *
 *    Function : OAI_OSC_fillPageTxDataReq
 *
 *    Functionality:
 *         - fills the Page nFAPI TX-DATA request message
 *
 * @params[in]    nfapi_nr_pdu_t *pduDesc
 * @params[in]    macCellCfg consist of SIB1 pdu
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t OAI_OSC_fillPageTxDataReq(nfapi_nr_pdu_t *pduDesc, uint16_t pduIndex, DlPageAlloc *pageAllocInfo)
{
   uint32_t payloadSize = 0;
   uint8_t *pagePayload = NULLP;

   pduDesc[pduIndex].PDU_index = pduIndex;
   pduDesc[pduIndex].num_TLV = 1;

   /* fill the TLV */
   payloadSize = pageAllocInfo->pageDlSch.tbInfo.tbSize;
   pduDesc[pduIndex].TLVs[0].tag = FAPI_TX_DATA_PAYLOAD; //pack direct
   pduDesc[pduIndex].TLVs[0].length = payloadSize;
   LWR_MAC_ALLOC(pagePayload, payloadSize);
   if(pagePayload == NULLP)
   {
      return RFAILED;
   }
   memcpy(pagePayload, pageAllocInfo->pageDlSch.dlPagePdu, pageAllocInfo->pageDlSch.dlPagePduLen);

   memcpy(pduDesc[pduIndex].TLVs[0].value.direct, pagePayload, payloadSize);
   pduDesc[pduIndex].PDU_length = payloadSize; 

   LWR_MAC_FREE(pagePayload, payloadSize);

   return ROK;
}

/***********************************************************************
 *
 * @brief fills the nFAPI RAR TX-DATA request message
 *
 * @details
 *
 *    Function : OAI_OSC_fillRarTxDataReq
 *
 *    Functionality:
 *         - fills the RAR nFAPI TX-DATA request message
 *
 * @params[in]    nfapi_nr_pdu_t *pduDesc
 * @params[in]    RarInfo *rarInfo
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t OAI_OSC_fillRarTxDataReq(nfapi_nr_pdu_t *pduDesc, uint16_t pduIndex, RarInfo *rarInfo, PdschCfg *pdschCfg)
{
   uint16_t payloadSize;
   uint8_t  *rarPayload = NULLP;

   pduDesc[pduIndex].PDU_index = pduIndex;
   pduDesc[pduIndex].num_TLV = 1;

   /* fill the TLV */
   payloadSize = pdschCfg->codeword[0].tbSize;
   pduDesc[pduIndex].TLVs[0].tag = FAPI_TX_DATA_PAYLOAD; //pack direct
   pduDesc[pduIndex].TLVs[0].length = payloadSize;
   LWR_MAC_ALLOC(rarPayload, payloadSize);
   if(rarPayload == NULLP)
   {
      return RFAILED;
   }
   
   memcpy(rarPayload, rarInfo->rarPdu, rarInfo->rarPduLen);

   memcpy(pduDesc[pduIndex].TLVs[0].value.direct, rarPayload, payloadSize);
   pduDesc[pduIndex].PDU_length = payloadSize;

   LWR_MAC_FREE(rarPayload, payloadSize);
   return ROK;
}

/***********************************************************************
 *
 * @brief fills the DL dedicated Msg nFAPI TX-DATA request message
 *
 * @details
 *
 *    Function : OAI_OSC_fillDlMsgTxDataReq
 *
 *    Functionality:
 *         - fills the Dl Dedicated Msg nFAPI TX-DATA request message
 *
 * @params[in]    nfapi_nr_pdu_t *pduDesc
 * @params[in]    DlMsgInfo *dlMsgInfo
 * @params[in]    uint32_t *msgLen
 * @params[in]    uint16_t pduIndex
 * @return ROK
 *
 * ********************************************************************/
uint8_t OAI_OSC_fillDlMsgTxDataReq(nfapi_nr_pdu_t *pduDesc, uint16_t pduIndex, DlMsgSchInfo *dlMsgSchInfo, PdschCfg *pdschCfg)
{
   uint16_t payloadSize;
   uint8_t  *dlMsgPayload = NULLP;

   pduDesc[pduIndex].PDU_index = pduIndex;
   pduDesc[pduIndex].num_TLV = 1;

   /* fill the TLV */
   payloadSize = pdschCfg->codeword[0].tbSize;
   pduDesc[pduIndex].TLVs[0].tag = FAPI_TX_DATA_PAYLOAD; //pack direct
   pduDesc[pduIndex].TLVs[0].length = payloadSize;
   LWR_MAC_ALLOC(dlMsgPayload, payloadSize);
   if(dlMsgPayload == NULLP)
   {
      return RFAILED;
   }
   memcpy(dlMsgPayload, dlMsgSchInfo->dlMsgPdu, dlMsgSchInfo->dlMsgPduLen);

   memcpy(pduDesc[pduIndex].TLVs[0].value.direct, dlMsgPayload, payloadSize);
   pduDesc[pduIndex].PDU_length = payloadSize;

   LWR_MAC_FREE(dlMsgPayload, payloadSize);

   return ROK;
}

/*******************************************************************
 *
 * @brief fills Dl DCI PDU required for nFAPI DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillSib1DlDciPdu
 *
 *    Functionality:
 *         -Fills the nFAPI Dl DCI PDU
 *
 * @params[in] Pointer to nfapi_nr_dl_dci_pdu_t
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/

void OAI_OSC_fillSib1DlDciPdu(nfapi_nr_dl_dci_pdu_t *dlDciPtr, PdcchCfg *sib1PdcchInfo)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes=0;
      uint8_t bytePos=0;
      uint8_t bitPos=0;

      uint16_t coreset0Size=0;
      uint16_t rbStart=0;
      uint16_t rbLen=0;
      uint32_t freqDomResAssign=0;
      uint32_t timeDomResAssign=0;
      uint8_t  VRB2PRBMap=0;
      uint32_t modNCodScheme=0;
      uint8_t  redundancyVer=0;
      uint32_t sysInfoInd=0;
      uint32_t reserved=0;

      /* Size(in bits) of each field in DCI format 0_1 
       * as mentioned in spec 38.214 */
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t redundancyVerSize    = 2;
      uint8_t sysInfoIndSize       = 1;
      uint8_t reservedSize         = 15;

      dlDciPtr->RNTI = sib1PdcchInfo->dci[0].rnti;
      dlDciPtr->ScramblingId = sib1PdcchInfo->dci[0].scramblingId;    
      dlDciPtr->ScramblingRNTI = sib1PdcchInfo->dci[0].scramblingRnti;
      dlDciPtr->CceIndex = sib1PdcchInfo->dci[0].cceIndex;
      dlDciPtr->AggregationLevel = sib1PdcchInfo->dci[0].aggregLevel;
      dlDciPtr->precodingAndBeamforming.num_prgs = sib1PdcchInfo->dci[0].beamPdcchInfo.numPrgs;
      dlDciPtr->precodingAndBeamforming.prg_size = sib1PdcchInfo->dci[0].beamPdcchInfo.prgSize;
      dlDciPtr->precodingAndBeamforming.dig_bf_interfaces = sib1PdcchInfo->dci[0].beamPdcchInfo.digBfInterfaces;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].pm_idx = sib1PdcchInfo->dci[0].beamPdcchInfo.prg[0].pmIdx;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].dig_bf_interface_list[0].beam_idx = sib1PdcchInfo->dci[0].beamPdcchInfo.prg[0].beamIdx[0];
      dlDciPtr->beta_PDCCH_1_0 = sib1PdcchInfo->dci[0].txPdcchPower.beta_pdcch_1_0;           
      dlDciPtr->powerControlOffsetSS = sib1PdcchInfo->dci[0].txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      coreset0Size= sib1PdcchInfo->coresetCfg.coreSetSize;
      rbStart = sib1PdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.startPrb;
      rbLen = sib1PdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coreset0Size - rbStart))
      {
	 if((rbLen - 1) <= floor(coreset0Size / 2))
	    freqDomResAssign = (coreset0Size * (rbLen-1)) + rbStart;
	 else
	    freqDomResAssign = (coreset0Size * (coreset0Size - rbLen + 1)) \
			       + (coreset0Size - 1 - rbStart);

	 freqDomResAssignSize = ceil(log2(coreset0Size * (coreset0Size + 1) / 2));
      }

      /* Fetching DCI field values */
      timeDomResAssign = sib1PdcchInfo->dci[0].pdschCfg.pdschTimeAlloc.rowIndex;
      VRB2PRBMap       = sib1PdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.vrbPrbMapping;
      modNCodScheme    = sib1PdcchInfo->dci[0].pdschCfg.codeword[0].mcsIndex;
      redundancyVer    = sib1PdcchInfo->dci[0].pdschCfg.codeword[0].rvIndex;
      sysInfoInd       = 0;           /* 0 for SIB1; 1 for SI messages */
      reserved         = 0;

      /* Reversing bits in each DCI field */
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      redundancyVer    = reverseBits(redundancyVer, redundancyVerSize);
      sysInfoInd       = reverseBits(sysInfoInd, sysInfoIndSize);

      /* Calulating total number of bytes in buffer */
      dlDciPtr->PayloadSizeBits = freqDomResAssignSize + timeDomResAssignSize\
				  + VRB2PRBMapSize + modNCodSchemeSize + redundancyVerSize\
				  + sysInfoIndSize + reservedSize;

      numBytes = dlDciPtr->PayloadSizeBits / 8;
      if(dlDciPtr->PayloadSizeBits % 8)
	 numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
	 DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
	 return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
	 dlDciPtr->Payload[bytePos] = 0;

      bytePos = numBytes - 1;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    redundancyVer, redundancyVerSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    sysInfoInd, sysInfoIndSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    reserved, reservedSize);
       
   }
} /* OAI_OSC_fillSib1DlDciPdu */

/*******************************************************************
 *
 * @brief fills RAR Dl DCI PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillRarDlDciPdu
 *
 *    Functionality:
 *         -Fills the nFAPI Dl DCI PDU
 *
 * @params[in] Pointer to nfapi_nr_dl_dci_pdu_t
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/

void OAI_OSC_fillRarDlDciPdu(nfapi_nr_dl_dci_pdu_t *dlDciPtr, PdcchCfg *rarPdcchInfo)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes =0;
      uint8_t bytePos =0;
      uint8_t bitPos =0;

      uint16_t coreset0Size =0;
      uint16_t rbStart =0;
      uint16_t rbLen =0;
      uint32_t freqDomResAssign =0;
      uint8_t timeDomResAssign =0;
      uint8_t  VRB2PRBMap =0;
      uint8_t modNCodScheme =0;
      uint8_t tbScaling =0;
      uint32_t reserved =0;

      /* Size(in bits) of each field in DCI format 1_0 */
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t tbScalingSize        = 2;
      uint8_t reservedSize         = 16;
      
      dlDciPtr->RNTI = rarPdcchInfo->dci[0].rnti;
      dlDciPtr->ScramblingId = rarPdcchInfo->dci[0].scramblingId;    
      dlDciPtr->ScramblingRNTI = rarPdcchInfo->dci[0].scramblingRnti;
      dlDciPtr->CceIndex = rarPdcchInfo->dci[0].cceIndex;
      dlDciPtr->AggregationLevel = rarPdcchInfo->dci[0].aggregLevel;
      dlDciPtr->precodingAndBeamforming.num_prgs = rarPdcchInfo->dci[0].beamPdcchInfo.numPrgs;
      dlDciPtr->precodingAndBeamforming.prg_size = rarPdcchInfo->dci[0].beamPdcchInfo.prgSize;
      dlDciPtr->precodingAndBeamforming.dig_bf_interfaces = rarPdcchInfo->dci[0].beamPdcchInfo.digBfInterfaces;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].pm_idx = rarPdcchInfo->dci[0].beamPdcchInfo.prg[0].pmIdx;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].dig_bf_interface_list[0].beam_idx = rarPdcchInfo->dci[0].beamPdcchInfo.prg[0].beamIdx[0];
      dlDciPtr->beta_PDCCH_1_0 = rarPdcchInfo->dci[0].txPdcchPower.beta_pdcch_1_0;           
      dlDciPtr->powerControlOffsetSS = rarPdcchInfo->dci[0].txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */

      /* TODO: Fill values of coreset0Size, rbStart and rbLen */
      coreset0Size= rarPdcchInfo->coresetCfg.coreSetSize;
      rbStart = rarPdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.startPrb;
      rbLen = rarPdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coreset0Size - rbStart))
      {
	 if((rbLen - 1) <= floor(coreset0Size / 2))
	    freqDomResAssign = (coreset0Size * (rbLen-1)) + rbStart;
	 else
	    freqDomResAssign = (coreset0Size * (coreset0Size - rbLen + 1)) \
			       + (coreset0Size - 1 - rbStart);

	 freqDomResAssignSize = ceil(log2(coreset0Size * (coreset0Size + 1) / 2));
      }

      /* Fetching DCI field values */
      timeDomResAssign = rarPdcchInfo->dci[0].pdschCfg.pdschTimeAlloc.rowIndex;
      VRB2PRBMap       = rarPdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.vrbPrbMapping;
      modNCodScheme    = rarPdcchInfo->dci[0].pdschCfg.codeword[0].mcsIndex;
      tbScaling        = 0; /* configured to 0 scaling */
      reserved         = 0;

      /* Reversing bits in each DCI field */
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      tbScaling        = reverseBits(tbScaling, tbScalingSize); 

      /* Calulating total number of bytes in buffer */
      dlDciPtr->PayloadSizeBits = freqDomResAssignSize + timeDomResAssignSize\
				  + VRB2PRBMapSize + modNCodSchemeSize + tbScalingSize + reservedSize;

      numBytes = dlDciPtr->PayloadSizeBits / 8;
      if(dlDciPtr->PayloadSizeBits % 8)
	 numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
	 DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
	 return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
	 dlDciPtr->Payload[bytePos] = 0;

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    tbScaling, tbScalingSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
	    reserved, reservedSize);
   }
} /* OAI_OSC_fillRarDlDciPdu */

/*******************************************************************
 *
 * @brief fills DL MSG DCI PDU required for nFAPI DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillDlMsgDlDciPdu
 *
 *    Functionality:
 *         -Fills the Dl MSG DCI PDU  
 *
 * @params[in] Pointer to nfapi_nr_dl_dci_pdu_t
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
void OAI_OSC_fillDlMsgDlDciPdu(nfapi_nr_dl_dci_pdu_t *dlDciPtr, PdcchCfg *pdcchInfo,\
      DlMsgSchInfo *dlMsgSchInfo)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes;
      uint8_t bytePos;
      uint8_t bitPos;

      uint16_t coresetSize = 0;
      uint16_t rbStart = 0;
      uint16_t rbLen = 0;
      uint8_t  dciFormatId;
      uint32_t freqDomResAssign;
      uint8_t  timeDomResAssign;
      uint8_t  VRB2PRBMap;
      uint8_t  modNCodScheme;
      uint8_t  ndi = 0;
      uint8_t  redundancyVer = 0;
      uint8_t  harqProcessNum = 0;
      uint8_t  dlAssignmentIdx = 0;
      uint8_t  pucchTpc = 0;
      uint8_t  pucchResoInd = 0;
      uint8_t  harqFeedbackInd = 0;

      /* Size(in bits) of each field in DCI format 1_0 */
      uint8_t dciFormatIdSize    = 1;
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t ndiSize              = 1;
      uint8_t redundancyVerSize    = 2;
      uint8_t harqProcessNumSize   = 4;
      uint8_t dlAssignmentIdxSize  = 2;
      uint8_t pucchTpcSize         = 2;
      uint8_t pucchResoIndSize     = 3;
      uint8_t harqFeedbackIndSize  = 3;

      dlDciPtr->RNTI = pdcchInfo->dci[0].rnti;
      dlDciPtr->ScramblingId = pdcchInfo->dci[0].scramblingId;
      dlDciPtr->ScramblingRNTI = pdcchInfo->dci[0].scramblingRnti;
      dlDciPtr->CceIndex = pdcchInfo->dci[0].cceIndex;
      dlDciPtr->AggregationLevel = pdcchInfo->dci[0].aggregLevel;
      dlDciPtr->precodingAndBeamforming.num_prgs = pdcchInfo->dci[0].beamPdcchInfo.numPrgs;
      dlDciPtr->precodingAndBeamforming.prg_size = pdcchInfo->dci[0].beamPdcchInfo.prgSize;
      dlDciPtr->precodingAndBeamforming.dig_bf_interfaces = pdcchInfo->dci[0].beamPdcchInfo.digBfInterfaces;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].pm_idx = pdcchInfo->dci[0].beamPdcchInfo.prg[0].pmIdx;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].dig_bf_interface_list[0].beam_idx = pdcchInfo->dci[0].beamPdcchInfo.prg[0].beamIdx[0];
      dlDciPtr->beta_PDCCH_1_0 = pdcchInfo->dci[0].txPdcchPower.beta_pdcch_1_0;
      dlDciPtr->powerControlOffsetSS = pdcchInfo->dci[0].txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      coresetSize = pdcchInfo->coresetCfg.coreSetSize;
      rbStart = pdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.startPrb;
      rbLen = pdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coresetSize - rbStart))
      {
         if((rbLen - 1) <= floor(coresetSize / 2))
            freqDomResAssign = (coresetSize * (rbLen-1)) + rbStart;
         else
            freqDomResAssign = (coresetSize * (coresetSize - rbLen + 1)) \
                               + (coresetSize - 1 - rbStart);

         freqDomResAssignSize = ceil(log2(coresetSize * (coresetSize + 1) / 2));
      }

      /* Fetching DCI field values */
      dciFormatId      = dlMsgSchInfo->dciFormatId;     /* Always set to 1 for DL */
      timeDomResAssign = pdcchInfo->dci[0].pdschCfg.pdschTimeAlloc.rowIndex -1;
      VRB2PRBMap       = pdcchInfo->dci[0].pdschCfg.pdschFreqAlloc.vrbPrbMapping;
      modNCodScheme    = pdcchInfo->dci[0].pdschCfg.codeword[0].mcsIndex;
      ndi              = dlMsgSchInfo->transportBlock[0].ndi;
      redundancyVer    = pdcchInfo->dci[0].pdschCfg.codeword[0].rvIndex;
      harqProcessNum   = dlMsgSchInfo->harqProcNum;
      dlAssignmentIdx  = dlMsgSchInfo->dlAssignIdx;
      pucchTpc         = dlMsgSchInfo->pucchTpc;
      pucchResoInd     = dlMsgSchInfo->pucchResInd;
      harqFeedbackInd  = dlMsgSchInfo->harqFeedbackInd;

      /* Reversing bits in each DCI field */
      dciFormatId      = reverseBits(dciFormatId, dciFormatIdSize);
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      ndi              = reverseBits(ndi, ndiSize);
      redundancyVer    = reverseBits(redundancyVer, redundancyVerSize);
      harqProcessNum   = reverseBits(harqProcessNum, harqProcessNumSize);
      dlAssignmentIdx  = reverseBits(dlAssignmentIdx , dlAssignmentIdxSize);
      pucchTpc         = reverseBits(pucchTpc, pucchTpcSize);
      pucchResoInd     = reverseBits(pucchResoInd, pucchResoIndSize);
      harqFeedbackInd  = reverseBits(harqFeedbackInd, harqFeedbackIndSize);


      /* Calulating total number of bytes in buffer */
      dlDciPtr->PayloadSizeBits = (dciFormatIdSize + freqDomResAssignSize\
            + timeDomResAssignSize + VRB2PRBMapSize + modNCodSchemeSize\
            + ndiSize + redundancyVerSize + harqProcessNumSize + dlAssignmentIdxSize\
            + pucchTpcSize + pucchResoIndSize + harqFeedbackIndSize);

      numBytes = dlDciPtr->PayloadSizeBits / 8;
      if(dlDciPtr->PayloadSizeBits % 8)
         numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
         DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
         return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
         dlDciPtr->Payload[bytePos] = 0;

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            dciFormatId, dciFormatIdSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            ndi, ndiSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            redundancyVer, redundancyVerSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            redundancyVer, redundancyVerSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            harqProcessNum, harqProcessNumSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            dlAssignmentIdx, dlAssignmentIdxSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            pucchTpc, pucchTpcSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            pucchResoInd, pucchResoIndSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            harqFeedbackInd, harqFeedbackIndSize);
   }
}

/*******************************************************************
 *
 * @brief fills bsr Ul DCI PDU required for nFAPI UL DCI Request to OAI PHY
 *
 * @details
 *
 *    Function : OAI_OSC_fillUlDciPdu
 *
 *    Functionality:
 *         -Fills the Ul DCI PDU, spec Ref:38.212, Table 7.3.1-1
 *
 * @params[in] Pointer to nfapi_nr_dl_dci_pdu_t
 *             Pointer to DciInfo
 * @return ROK
 *
 ******************************************************************/
void OAI_OSC_fillUlDciPdu(nfapi_nr_dl_dci_pdu_t *ulDciPtr, DciInfo *schDciInfo)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : UL_DCI_REQUEST\n");
#endif
   if(ulDciPtr != NULLP)
   {
      uint8_t numBytes =0;
      uint8_t bytePos =0;
      uint8_t bitPos =0;

      uint8_t  coreset1Size = 0;
      uint16_t rbStart = 0;
      uint16_t rbLen = 0;
      uint8_t  dciFormatId = 0;
      uint32_t freqDomResAssign =0;
      uint8_t  timeDomResAssign =0;
      uint8_t  freqHopFlag =0;
      uint8_t  modNCodScheme =0;
      uint8_t  ndi =0;
      uint8_t  redundancyVer = 0;
      uint8_t  harqProcessNum = 0;
      uint8_t  puschTpc = 0;
      uint8_t  ul_SlInd = 0;

      /* Size(in bits) of each field in DCI format 0_0 */
      uint8_t dciFormatIdSize      = 1;
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t freqHopFlagSize      = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t ndiSize              = 1;
      uint8_t redundancyVerSize    = 2;
      uint8_t harqProcessNumSize   = 4;
      uint8_t puschTpcSize         = 2;
      uint8_t ul_SlIndSize         = 1;

      ulDciPtr->RNTI                          = schDciInfo->dciInfo.rnti;
      ulDciPtr->ScramblingId                  = schDciInfo->dciInfo.scramblingId;    
      ulDciPtr->ScramblingRNTI                = schDciInfo->dciInfo.scramblingRnti;
      ulDciPtr->CceIndex                      = schDciInfo->dciInfo.cceIndex;
      ulDciPtr->AggregationLevel              = schDciInfo->dciInfo.aggregLevel;
      ulDciPtr->precodingAndBeamforming.num_prgs          = schDciInfo->dciInfo.beamPdcchInfo.numPrgs;
      ulDciPtr->precodingAndBeamforming.prg_size          = schDciInfo->dciInfo.beamPdcchInfo.prgSize;
      ulDciPtr->precodingAndBeamforming.dig_bf_interfaces  = schDciInfo->dciInfo.beamPdcchInfo.digBfInterfaces;
      ulDciPtr->precodingAndBeamforming.prgs_list[0].pm_idx = schDciInfo->dciInfo.beamPdcchInfo.prg[0].pmIdx;
      ulDciPtr->precodingAndBeamforming.prgs_list[0].dig_bf_interface_list[0].beam_idx = schDciInfo->dciInfo.beamPdcchInfo.prg[0].beamIdx[0];
      ulDciPtr->beta_PDCCH_1_0                = schDciInfo->dciInfo.txPdcchPower.beta_pdcch_1_0;           
      ulDciPtr->powerControlOffsetSS          = schDciInfo->dciInfo.txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset1Size = Size of coreset 1
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      if(schDciInfo->dciFormatInfo.formatType == FORMAT0_0)
      {
         coreset1Size = schDciInfo->coresetCfg.coreSetSize;
         rbLen = schDciInfo->dciFormatInfo.format.format0_0.freqAlloc.resAlloc.type1.numPrb;
         rbStart = schDciInfo->dciFormatInfo.format.format0_0.freqAlloc.resAlloc.type1.startPrb;

         if((rbLen >=1) && (rbLen <= coreset1Size - rbStart))
         {
            if((rbLen - 1) <= floor(coreset1Size / 2))
               freqDomResAssign = (coreset1Size * (rbLen-1)) + rbStart;
            else
               freqDomResAssign = (coreset1Size * (coreset1Size - rbLen + 1)) \
                                  + (coreset1Size - 1 - rbStart);

            freqDomResAssignSize = ceil(log2(coreset1Size * (coreset1Size + 1) / 2));
         }
         /* Fetching DCI field values */
         dciFormatId      = schDciInfo->dciFormatInfo.formatType; /* DCI indentifier for UL DCI */
         timeDomResAssign = schDciInfo->dciFormatInfo.format.format0_0.rowIndex;
         freqHopFlag      = schDciInfo->dciFormatInfo.format.format0_0.freqHopFlag; 
         modNCodScheme    = schDciInfo->dciFormatInfo.format.format0_0.mcs;
         ndi              = schDciInfo->dciFormatInfo.format.format0_0.ndi; 
         redundancyVer    = schDciInfo->dciFormatInfo.format.format0_0.rvIndex;
         harqProcessNum   = schDciInfo->dciFormatInfo.format.format0_0.harqProcId; 
         puschTpc         = schDciInfo->dciFormatInfo.format.format0_0.tpcCmd;
         ul_SlInd         = schDciInfo->dciFormatInfo.format.format0_0.sulIndicator;
     
         /* Reversing bits in each DCI field */
         dciFormatId      = reverseBits(dciFormatId, dciFormatIdSize);
         freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
         timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
         modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
         redundancyVer    = reverseBits(redundancyVer, redundancyVerSize);
         harqProcessNum   = reverseBits(harqProcessNum, harqProcessNumSize);
         puschTpc         = reverseBits(puschTpc, puschTpcSize);
         ul_SlInd         = reverseBits(ul_SlInd, ul_SlIndSize);
      }
      /* Calulating total number of bytes in buffer */
      ulDciPtr->PayloadSizeBits = (dciFormatIdSize + freqDomResAssignSize\
      + timeDomResAssignSize + freqHopFlagSize + modNCodSchemeSize + ndi \
      + redundancyVerSize + harqProcessNumSize + puschTpcSize + ul_SlIndSize);

      numBytes = ulDciPtr->PayloadSizeBits / 8;
      if(ulDciPtr->PayloadSizeBits % 8)
         numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
         DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
         return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
         ulDciPtr->Payload[bytePos] = 0;

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            dciFormatId, dciFormatIdSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            freqHopFlag, freqHopFlagSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            ndi, ndiSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            redundancyVer, redundancyVerSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            harqProcessNum, harqProcessNumSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            puschTpc, puschTpcSize);
      fillDlDciPayload(ulDciPtr->Payload, &bytePos, &bitPos,\
            ul_SlInd, ul_SlIndSize);
   }
} /* OAI_OSC_fillUlDciPdu */


/*******************************************************************
 *
 * @brief fills Dl DCI PDU for Paging required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillPageDlDciPdu
 *
 *    Functionality:
 *         -Fills the Dl DCI PDU for Paging
 *
 * @params[in] Pointer to nfapi_dl_dci_t
 *             Pointer to dlPageAlloc
 * @return ROK
 *
 ******************************************************************/

void OAI_OSC_fillPageDlDciPdu(nfapi_nr_dl_dci_pdu_t *dlDciPtr, DlPageAlloc *dlPageAlloc, MacCellCfg *macCellCfg)
{
   if(dlDciPtr != NULLP)
   {
      uint8_t numBytes=0;
      uint8_t bytePos=0;
      uint8_t bitPos=0;

      uint16_t coreset0Size     = 0;
      uint16_t rbStart          = 0;
      uint16_t rbLen            = 0;
      uint8_t  shortMsgInd      = 0;
      uint8_t  shortMsg         = 0;
      uint32_t freqDomResAssign = 0;
      uint32_t timeDomResAssign = 0;
      uint8_t  VRB2PRBMap       = 0;
      uint32_t modNCodScheme    = 0;
      uint8_t  tbScaling        = 0;
      uint32_t reserved         = 0;

      /* Size(in bits) of each field in DCI format 1_0 
       * as mentioned in spec 38.214 */
      uint8_t shortMsgIndSize      = 2;
      uint8_t shortMsgSize         = 8;
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t VRB2PRBMapSize       = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t tbScalingSize        = 2;
      uint8_t reservedSize         = 6;

      dlDciPtr->RNTI = P_RNTI;
      dlDciPtr->ScramblingId = macCellCfg->cellCfg.phyCellId;
      dlDciPtr->ScramblingRNTI = 0;
      dlDciPtr->CceIndex = dlPageAlloc->pageDlDci.cceIndex;
      dlDciPtr->AggregationLevel = dlPageAlloc->pageDlDci.aggregLevel;
      dlDciPtr->precodingAndBeamforming.num_prgs = 1;
      dlDciPtr->precodingAndBeamforming.prg_size = 1;
      dlDciPtr->precodingAndBeamforming.dig_bf_interfaces = 0;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].pm_idx = 0;
      dlDciPtr->precodingAndBeamforming.prgs_list[0].dig_bf_interface_list[0].beam_idx = 0;
      dlDciPtr->beta_PDCCH_1_0 = 0;
      dlDciPtr->powerControlOffsetSS = 0;

      /* Calculating freq domain resource allocation field value and size
       * coreset0Size = Size of coreset 0
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      coreset0Size = dlPageAlloc->pageDlDci.coreSetSize;
      rbStart = dlPageAlloc->pageDlSch.freqAlloc.startPrb;
      rbLen = dlPageAlloc->pageDlSch.freqAlloc.numPrb;

      if((rbLen >=1) && (rbLen <= coreset0Size - rbStart))
      {
         if((rbLen - 1) <= floor(coreset0Size / 2))
            freqDomResAssign = (coreset0Size * (rbLen-1)) + rbStart;
         else
            freqDomResAssign = (coreset0Size * (coreset0Size - rbLen + 1)) \
                               + (coreset0Size - 1 - rbStart);

         freqDomResAssignSize = ceil(log2(coreset0Size * (coreset0Size + 1) / 2));
      }

      /*Fetching DCI field values */

      /*Refer:38.212 - Table 7.3.1.2.1-1: Short Message indicator >*/
      if(dlPageAlloc->shortMsgInd != TRUE)
      {
         /*When Short Msg is absent*/
         shortMsgInd = 1;
         shortMsg    = 0;
      }
      else
      {
         /*Short Msg is Present*/
         if(dlPageAlloc->pageDlSch.dlPagePduLen == 0 || dlPageAlloc->pageDlSch.dlPagePdu == NULLP)
         {
            /*When Paging Msg is absent*/
            shortMsgInd = 2;
         }
         else
         {
            /*Both Short and Paging is present*/
            shortMsgInd = 3;
         }
         shortMsg = dlPageAlloc->shortMsg;
      }

      timeDomResAssign = 0;
      VRB2PRBMap       = dlPageAlloc->pageDlSch.vrbPrbMapping;
      modNCodScheme    = dlPageAlloc->pageDlSch.tbInfo.mcs;
      tbScaling        = 0;
      reserved         = 0;

      /* Reversing bits in each DCI field */
      shortMsgInd      = reverseBits(shortMsgInd, shortMsgIndSize);
      shortMsg         = reverseBits(shortMsg, shortMsgSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
      timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
      VRB2PRBMap       = reverseBits(VRB2PRBMap, VRB2PRBMapSize);
      modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
      tbScaling        = reverseBits(tbScaling, tbScalingSize); 

      /* Calulating total number of bytes in buffer */
      dlDciPtr->PayloadSizeBits = shortMsgIndSize + shortMsgSize + freqDomResAssignSize\
                                  + timeDomResAssignSize + VRB2PRBMapSize + modNCodSchemeSize\
                                  + tbScaling + reservedSize;

      numBytes = dlDciPtr->PayloadSizeBits / 8;
      if(dlDciPtr->PayloadSizeBits % 8)
      {
         numBytes += 1;
      }

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
         DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
         return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
      {
         dlDciPtr->Payload[bytePos] = 0;
      }

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            shortMsgInd, shortMsgIndSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            shortMsg, shortMsgSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            VRB2PRBMap, VRB2PRBMapSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            tbScaling, tbScalingSize);
      fillDlDciPayload(dlDciPtr->Payload, &bytePos, &bitPos,\
            reserved, reservedSize);
   }
} /* OAI_OSC_fillPageDlDciPdu */

/***********************************************************************
 *
 * @brief calculates the total size to be allocated for nFAPI UL TTI Req
 *
 * @details
 *
 *    Function : OAI_OSC_getnPdus
 *
 *    Functionality:
 *         -calculates the total pdu count to be allocated for nFAPI UL TTI Req
 *
 * @params[in] Pointer to nfapi Ul TTI Req
 *             Pointer to CurrUlSlot
 * @return count
 * ********************************************************************/
uint8_t OAI_OSC_getnPdus(nfapi_nr_ul_tti_request_t *ulTtiReq, MacUlSlot *currUlSlot)
{
   uint8_t pduCount = 0;

   if(ulTtiReq && currUlSlot)
   {
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PRACH)
      {
	 pduCount++;
	 ulTtiReq->rach_present++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PUSCH)
      {
	 pduCount++;
	 ulTtiReq->n_ulsch++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PUSCH_UCI)
      {
	 pduCount++;
	 ulTtiReq->n_ulsch++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_UCI)
      {
	 pduCount++;
	 ulTtiReq->n_ulcch++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_SRS)
      {
	 pduCount++;
      }
   }
   return pduCount;
}

/*******************************************************************
 *
 * @brief fills SSB PDU required for nFAPI DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillSsbPdu
 *
 *    Functionality:
 *         -Fills the SSB PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to nFAPI DL TTI Req
 *             Pointer to RgCellCb
 *             Pointer to msgLen of DL TTI Info
 * @return ROK
 *
 ******************************************************************/

uint8_t OAI_OSC_fillSsbPdu(nfapi_nr_dl_tti_request_pdu_t *dlTtiReqPdu, MacCellCfg *macCellCfg,
      MacDlSlot *currDlSlot, uint8_t ssbIdxCount, uint16_t sfn)
{
   uint32_t mibPayload = 0;
   if(dlTtiReqPdu != NULL)
   {
      dlTtiReqPdu->PDUType = SSB_PDU_TYPE;     /* SSB PDU */
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.PhysCellId = macCellCfg->cellCfg.phyCellId;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.BetaPss = macCellCfg->ssbCfg.betaPss;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.SsbBlockIndex = currDlSlot->dlInfo.brdcstAlloc.ssbInfo[ssbIdxCount].ssbIdx;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.SsbSubcarrierOffset = macCellCfg->ssbCfg.ssbScOffset;;
      /* ssbOfPdufstA to be filled in ssbCfg */
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.ssbOffsetPointA = macCellCfg->ssbCfg.ssbOffsetPointA;;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.bchPayloadFlag = macCellCfg->ssbCfg.bchPayloadFlag;
      /* Bit manipulation for SFN */
      setMibPdu(macCellCfg->ssbCfg.mibPdu, &mibPayload, sfn);
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.bchPayload = mibPayload;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.precoding_and_beamforming.num_prgs = 0;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.precoding_and_beamforming.prg_size = 0;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.precoding_and_beamforming.dig_bf_interfaces = 0;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.precoding_and_beamforming.prgs_list[0].pm_idx = 0;
      dlTtiReqPdu->ssb_pdu.ssb_pdu_rel15.precoding_and_beamforming. \
	 prgs_list[0].dig_bf_interface_list[0].beam_idx = macCellCfg->ssbCfg.beamId[0];
      dlTtiReqPdu->PDUSize = sizeof(nfapi_nr_dl_tti_ssb_pdu_rel15_t);  /* Size of SSB PDU */
      return ROK;
   }
   return RFAILED;
}

/*******************************************************************
 *
 * @brief fills Dl PDCCH Info from DL PageAlloc
 *
 * @details
 *
 *    Function : OAI_OSC_fillPagePdcchPdu
 *
 *    Functionality:
 *         -Fills the PdcchInfo
 *
 * @params[in] Pointer to DlPageAlloc
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
void OAI_OSC_fillPagePdcchPdu(nfapi_nr_dl_tti_request_pdu_t *dlTtiReqPdu, DlPageAlloc *pageAlloc, MacCellCfg *macCellCfg)
{
   if(dlTtiReqPdu != NULLP)
   {
      BwpCfg *bwp = NULLP;

      memset(&dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15, 0, sizeof(nfapi_nr_dl_tti_pdcch_pdu_rel15_t));
      bwp = &pageAlloc->bwp;
      OAI_OSC_fillPageDlDciPdu(dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.dci_pdu, pageAlloc, macCellCfg);

      dlTtiReqPdu->PDUType = PDCCH_PDU_TYPE;

      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.BWPSize           = bwp->freqAlloc.numPrb;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.BWPStart          = bwp->freqAlloc.startPrb;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.SubcarrierSpacing = bwp->subcarrierSpacing;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.CyclicPrefix      = bwp->cyclicPrefix;

      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.StartSymbolIndex    = pageAlloc->pageDlDci.ssStartSymbolIndex;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.DurationSymbols     = pageAlloc->pageDlDci.durationSymbols;
      memcpy(dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.FreqDomainResource, pageAlloc->pageDlDci.freqDomainResource, 6*sizeof(uint8_t));
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.CceRegMappingType   = pageAlloc->pageDlDci.cceRegMappingType;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.RegBundleSize       = pageAlloc->pageDlDci.cceReg.interleaved.regBundleSize;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.InterleaverSize     = pageAlloc->pageDlDci.cceReg.interleaved.interleaverSize;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.ShiftIndex          = pageAlloc->pageDlDci.cceReg.interleaved.shiftIndex;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.precoderGranularity = pageAlloc->pageDlDci.precoderGranularity;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.numDlDci            = 1;
      dlTtiReqPdu->pdcch_pdu.pdcch_pdu_rel15.CoreSetType         = CORESET_TYPE0;

      /* Calculating PDU length. Considering only one dl dci pdu for now */
      dlTtiReqPdu->PDUSize = sizeof(nfapi_nr_dl_tti_pdcch_pdu_rel15_t);
   }
}
/*******************************************************************
 *
 * @brief fills PDSCH PDU required for DL TTI info in MAC
 *
 * @details
 *
 *    Function : OAI_OSC_fillPagePdschPdu
 *
 *    Functionality:
 *         -Fills the Pdsch PDU info
 *          stored in MAC
 *
 * @params[in] Pointer to nFAPI DL TTI Req
 *             Pointer to PdschCfg
 *             Pointer to msgLen of DL TTI Info
 * @return ROK
 *
 ******************************************************************/
void OAI_OSC_fillPagePdschPdu(nfapi_nr_dl_tti_request_pdu_t *dlTtiReqPdu, DlPageAlloc *pageAlloc,
                       uint16_t pduIndex, MacCellCfg *macCellCfg)
{   
   uint8_t idx;

   if(dlTtiReqPdu != NULLP)
   {
      dlTtiReqPdu->PDUSize = PDSCH_PDU_TYPE;
      memset(&dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15, 0, sizeof(nfapi_nr_dl_tti_pdsch_pdu_rel15_t));
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.pduBitmap = 0;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rnti = P_RNTI;         
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.pduIndex = pduIndex;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.BWPSize = pageAlloc->bwp.freqAlloc.numPrb;       
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.BWPStart = pageAlloc->bwp.freqAlloc.startPrb;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.SubcarrierSpacing = pageAlloc->bwp.subcarrierSpacing;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.CyclicPrefix = pageAlloc->bwp.cyclicPrefix;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.NrOfCodewords = 1;
      for(idx = 0; idx < MAX_CODEWORDS ; idx++)
      { 
         dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.targetCodeRate[idx] = 308;
         dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.qamModOrder[idx] = 2;
         dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.mcsIndex[idx] = pageAlloc->pageDlSch.tbInfo.mcs;
         dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.mcsTable[idx] = 0;
         dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rvIndex[idx] = 0;
         dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.TBSize[idx] = pageAlloc->pageDlSch.tbInfo.tbSize;
      }
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dataScramblingId = macCellCfg->cellCfg.phyCellId;       
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.nrOfLayers = 1;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.transmissionScheme = 0;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.refPoint = 0;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dlDmrsSymbPos = DL_DMRS_SYMBOL_POS;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dmrsConfigType = pageAlloc->pageDlSch.dmrs.dmrsType;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dlDmrsScramblingId = macCellCfg->cellCfg.phyCellId;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.SCID = 0;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.numDmrsCdmGrpsNoData = 1;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dmrsPorts = 0x0001;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.resourceAlloc = 1;
      /* since we are using type-1, hence rbBitmap excluded */
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rbStart = pageAlloc->pageDlSch.freqAlloc.startPrb;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.rbSize = pageAlloc->pageDlSch.freqAlloc.numPrb;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.VRBtoPRBMapping = pageAlloc->pageDlSch.vrbPrbMapping;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.StartSymbolIndex = pageAlloc->pageDlSch.timeAlloc.startSymb;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.NrOfSymbols = pageAlloc->pageDlSch.timeAlloc.numSymb;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.num_prgs = 1;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.prg_size = 0;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.dig_bf_interfaces = 0;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.prgs_list[0]. \
         pm_idx = 0;
      dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.precodingAndBeamforming.prgs_list[0]. \
         dig_bf_interface_list[0].beam_idx = 0;
      // dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.powerControlOffset = pdschInfo->txPdschPower.powerControlOffset;  
      // dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.powerControlOffsetSS = pdschInfo->txPdschPower.powerControlOffsetSS;
      // dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.mappingType =   pdschInfo->dmrs.mappingType;
      // dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.nrOfDmrsSymbols = pdschInfo->dmrs.nrOfDmrsSymbols;
      // dlTtiReqPdu->pdsch_pdu.pdsch_pdu_rel15.dmrsAddPos = pdschInfo->dmrs.dmrsAddPos;

      dlTtiReqPdu->PDUSize = sizeof(nfapi_nr_dl_tti_pdsch_pdu_rel15_t);
   }
}

/*******************************************************************
 *
 * @brief Sends DL TTI Request to PHY
 *
 * @details
 *
 *    Function : fillDlTtiReq
 *
 *    Functionality:
 *         -Sends FAPI DL TTI req to PHY
 *
 * @params[in]    timing info
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint16_t fillDlTtiReq(SlotTimingInfo currTimingInfo)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : DL_TTI_REQUEST\n");
#endif

#ifdef INTEL_FAPI
   uint8_t idx =0;
   uint8_t nPdu = 0;
   uint8_t numPduEncoded = 0;
   uint8_t  ueIdx;
   uint16_t cellIdx =0;
   uint16_t pduIndex = 0;

   SlotTimingInfo dlTtiReqTimingInfo;
   MacDlSlot *currDlSlot = NULLP;
   MacCellCfg macCellCfg;
   RntiType rntiType;
   fapi_dl_tti_req_t *dlTtiReq = NULLP;
   fapi_msg_header_t *msgHeader = NULLP;
   p_fapi_api_queue_elem_t dlTtiElem;
   p_fapi_api_queue_elem_t headerElem;
   p_fapi_api_queue_elem_t prevElem;
   if(lwrMacCb.phyState == PHY_STATE_RUNNING)
   {
	   GET_CELL_IDX(currTimingInfo.cellId, cellIdx);
	   /* consider phy delay */
	   ADD_DELTA_TO_TIME(currTimingInfo,dlTtiReqTimingInfo,PHY_DELTA_DL, macCb.macCell[cellIdx]->numOfSlots);
	   dlTtiReqTimingInfo.cellId = currTimingInfo.cellId;

	   macCellCfg = macCb.macCell[cellIdx]->macCellCfg;

	   currDlSlot = &macCb.macCell[cellIdx]->dlSlot[dlTtiReqTimingInfo.slot]; 

           /* Vendor Message */
	   fapi_vendor_msg_t *vendorMsg;
	   p_fapi_api_queue_elem_t  vendorMsgQElem;
	   /* Allocte And fill Vendor msg */
	   LWR_MAC_ALLOC(vendorMsgQElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_vendor_msg_t)));  
	   if(!vendorMsgQElem)
	   {
		   DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for vendor msg in config req");
		   return RFAILED;
	   }
	   FILL_FAPI_LIST_ELEM(vendorMsgQElem, NULLP, FAPI_VENDOR_MESSAGE, 1, sizeof(fapi_vendor_msg_t)); 
	   vendorMsg = (fapi_vendor_msg_t *)(vendorMsgQElem + 1);
	   fillMsgHeader(&vendorMsg->header, FAPI_VENDOR_MESSAGE, sizeof(fapi_vendor_msg_t));

	   LWR_MAC_ALLOC(dlTtiElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_dl_tti_req_t)));
	   if(dlTtiElem)
	   {
		   FILL_FAPI_LIST_ELEM(dlTtiElem, NULLP, FAPI_DL_TTI_REQUEST, 1, \
				   sizeof(fapi_dl_tti_req_t));
		   /* Fill message header */
		   LWR_MAC_ALLOC(headerElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_msg_header_t)));
		   if(!headerElem)
		   {
			   DU_LOG("\nERROR  -->  LWR_MAC: Memory allocation failed for header in DL TTI req");
			   LWR_MAC_FREE(dlTtiElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_dl_tti_req_t)));
			   return RFAILED;
		   }

		   FILL_FAPI_LIST_ELEM(headerElem, dlTtiElem, FAPI_VENDOR_MSG_HEADER_IND, 1, \
				   sizeof(fapi_msg_header_t));
		   msgHeader = (fapi_msg_header_t *)(headerElem + 1);
		   msgHeader->num_msg = 2;
		   msgHeader->handle = 0;

		   /* Fill Dl TTI Request */
		   dlTtiReq = (fapi_dl_tti_req_t *)(dlTtiElem +1);
		   memset(dlTtiReq, 0, sizeof(fapi_dl_tti_req_t));
		   fillMsgHeader(&dlTtiReq->header, FAPI_DL_TTI_REQUEST, sizeof(fapi_dl_tti_req_t));

		   dlTtiReq->sfn  = dlTtiReqTimingInfo.sfn;
		   dlTtiReq->slot = dlTtiReqTimingInfo.slot;
		   dlTtiReq->nPdus = calcDlTtiReqPduCount(currDlSlot);  /* get total Pdus */
		   nPdu = dlTtiReq->nPdus;

                   vendorMsg->p7_req_vendor.dl_tti_req.num_pdus = nPdu;
                   vendorMsg->p7_req_vendor.dl_tti_req.sym = 0;

		   dlTtiReq->nGroup = 0;
		   if(dlTtiReq->nPdus > 0)
		   {
			   if(currDlSlot->dlInfo.isBroadcastPres)
			   {
				   if(currDlSlot->dlInfo.brdcstAlloc.ssbTransmissionMode)
				   {
					   if(dlTtiReq->pdus != NULLP)
					   {
						   for(idx = 0; idx < currDlSlot->dlInfo.brdcstAlloc.ssbIdxSupported; idx++)
						   {
							   fillSsbPdu(&dlTtiReq->pdus[numPduEncoded], &macCellCfg,\
									   currDlSlot, idx, dlTtiReq->sfn);
							   numPduEncoded++;
						   }
					   }
					   DU_LOG("\033[1;31m");
					   DU_LOG("\nDEBUG  -->  LWR_MAC: MIB sent..");
					   DU_LOG("\033[0m");
				   }

				   if(currDlSlot->dlInfo.brdcstAlloc.sib1TransmissionMode)
				   {
					   /* Filling SIB1 param */
					   if(numPduEncoded != nPdu)
					   {
                     if(currDlSlot->dlInfo.brdcstAlloc.crnti == SI_RNTI)
                        rntiType = SI_RNTI_TYPE;

						   /* PDCCH PDU */
						   fillPdcchPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded], 
								   currDlSlot, -1, rntiType, CORESET_TYPE0, MAX_NUM_UE);
						   numPduEncoded++;

						   /* PDSCH PDU */
						   fillPdschPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded],
								   &currDlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg->dci[0].pdschCfg,
								   currDlSlot->dlInfo.brdcstAlloc.sib1Alloc.bwp,
								   pduIndex);
						   dlTtiReq->ue_grp_info[dlTtiReq->nGroup].pduIdx[pduIndex] = pduIndex;
						   pduIndex++;
						   numPduEncoded++;
					   }
					   DU_LOG("\033[1;34m");
					   DU_LOG("\nDEBUG  -->  LWR_MAC: SIB1 sent...");
					   DU_LOG("\033[0m");
				   }
			   }

			   if(currDlSlot->pageAllocInfo != NULLP)
			   {
				   /* Filling DL Paging Alloc param */
				   if(numPduEncoded != nPdu)
				   {
					   rntiType = P_RNTI_TYPE;
					   fillPagePdcchPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded], \
                                   currDlSlot->pageAllocInfo, &macCellCfg);
					   numPduEncoded++;
					   fillPagePdschPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded],
							               currDlSlot->pageAllocInfo, pduIndex, &macCellCfg);
					   dlTtiReq->ue_grp_info[dlTtiReq->nGroup].pduIdx[pduIndex] = pduIndex;
					   pduIndex++;
					   numPduEncoded++;
				   }
				   DU_LOG("\033[1;34m");
				   DU_LOG("\nDEBUG  -->  LWR_MAC: PAGE sent...");
				   DU_LOG("\033[0m");
			   }

			   for(ueIdx=0; ueIdx<MAX_NUM_UE; ueIdx++)
			   {
				   if(currDlSlot->dlInfo.rarAlloc[ueIdx] != NULLP)
				   {
					   /* Filling RAR param */
					   rntiType = RA_RNTI_TYPE;
                  if(currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg)
                  {
						   fillPdcchPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded],
								   currDlSlot, -1, rntiType, CORESET_TYPE0, ueIdx);
						   numPduEncoded++;
                     MAC_FREE(currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg, sizeof(PdcchCfg));
                  }
					   if(currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg)
					   {
						   fillPdschPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded],
								   currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg,
								   currDlSlot->dlInfo.rarAlloc[ueIdx]->bwp,
								   pduIndex);
						   numPduEncoded++;
						   pduIndex++;

                     DU_LOG("\033[1;32m");
						   DU_LOG("\nDEBUG  -->  LWR_MAC: RAR sent...");
						   DU_LOG("\033[0m");
					   }
				   }

				   if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx] != NULLP)
				   {
                  if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg)  \
                  {
                     rntiType = C_RNTI_TYPE;
                     fillPdcchPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded],
                           currDlSlot, idx, rntiType, currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg->coresetCfg.coreSetType, ueIdx);
                     numPduEncoded++;
                  }

                  if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdu != NULLP)
                  {
                     if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg)
                     {
                        fillPdschPdu(&dlTtiReq->pdus[numPduEncoded], &vendorMsg->p7_req_vendor.dl_tti_req.pdus[numPduEncoded], \
                              currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg,\
                              currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->bwp, pduIndex);
                        numPduEncoded++;
                        pduIndex++;

                        DU_LOG("\033[1;32m");
                        if((macCb.macCell[cellIdx]->macRaCb[ueIdx].macMsg4Status))
                        {
                           DU_LOG("\nDEBUG  -->  LWR_MAC: MSG4 sent...");
                           MAC_FREE(macCb.macCell[cellIdx]->macRaCb[ueIdx].macMsg4Status, sizeof(bool));
                        }
                        else
                        {
                           DU_LOG("\nDEBUG  -->  LWR_MAC: DL MSG sent...");
                        }
                        DU_LOG("\033[0m");
                     }

                  }
                  MAC_FREE(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg,sizeof(PdcchCfg));
                  /*   else
                       {
                       MAC_FREE(currDlSlot->dlInfo.dlMsgAlloc[ueIdx], sizeof(DlMsgAlloc));
                       currDlSlot->dlInfo.dlMsgAlloc[ueIdx] = NULLP;
                       }
                       */
				   }
			   }

			   dlTtiReq->ue_grp_info[dlTtiReq->nGroup].nUe = MAX_NUM_UE_PER_TTI;
			   dlTtiReq->nGroup++;

#ifdef ODU_SLOT_IND_DEBUG_LOG	    
			   DU_LOG("\nDEBUG  -->  LWR_MAC: Sending DL TTI Request");
#endif	    

			   /* Intel L1 expects UL_TTI.request following DL_TTI.request */
			   fillUlTtiReq(currTimingInfo, dlTtiElem, &(vendorMsg->p7_req_vendor.ul_tti_req));
			   msgHeader->num_msg++;

			   /* Intel L1 expects UL_DCI.request following DL_TTI.request */
			   fillUlDciReq(dlTtiReqTimingInfo, dlTtiElem->p_next, &(vendorMsg->p7_req_vendor.ul_dci_req));
			   msgHeader->num_msg++;

			   /* send Tx-DATA req message */
			   sendTxDataReq(dlTtiReqTimingInfo, currDlSlot, dlTtiElem->p_next->p_next, &(vendorMsg->p7_req_vendor.tx_data_req));
			   if(dlTtiElem->p_next->p_next->p_next)
			   {
				   msgHeader->num_msg++;
				   prevElem = dlTtiElem->p_next->p_next->p_next;
			   }
			   else
				   prevElem = dlTtiElem->p_next->p_next;
		   }
		   else
		   {
#ifdef ODU_SLOT_IND_DEBUG_LOG	    
			   DU_LOG("\nDEBUG  -->  LWR_MAC: Sending DL TTI Request");
#endif	    
			   /* Intel L1 expects UL_TTI.request following DL_TTI.request */
			   fillUlTtiReq(currTimingInfo, dlTtiElem, &(vendorMsg->p7_req_vendor.ul_tti_req));
			   msgHeader->num_msg++;

			   /* Intel L1 expects UL_DCI.request following DL_TTI.request */
			   fillUlDciReq(dlTtiReqTimingInfo, dlTtiElem->p_next, &(vendorMsg->p7_req_vendor.ul_dci_req));
			   msgHeader->num_msg++;

			   prevElem = dlTtiElem->p_next->p_next;
		   }

		   if(macCb.macCell[cellIdx]->state == CELL_TO_BE_STOPPED)
		   {
			   /* Intel L1 expects UL_DCI.request following DL_TTI.request */
			   lwr_mac_procStopReqEvt(currTimingInfo, prevElem, &(vendorMsg->stop_req_vendor));
			   msgHeader->num_msg++;
			   macCb.macCell[cellIdx]->state = CELL_STOP_IN_PROGRESS;
            prevElem = prevElem->p_next;
		   }
		   prevElem->p_next = vendorMsgQElem;
		   LwrMacSendToL1(headerElem);
		   memset(currDlSlot, 0, sizeof(MacDlSlot));
		   return ROK;
	   }
	   else
	   {
		   DU_LOG("\nERROR  -->  LWR_MAC: Failed to allocate memory for DL TTI Request");
		   memset(currDlSlot, 0, sizeof(MacDlSlot));
		   return RFAILED;
	   }
   }
   else
   {
	   lwr_mac_procInvalidEvt(&currTimingInfo);
	   return RFAILED;
   }
#endif
   return ROK;
}

/*******************************************************************
 *
 * @brief Sends DL TTI Request to OAI PHY
 *
 * @details
 *
 *    Function : OAI_OSC_fillDlTtiReq
 *
 *    Functionality:
 *         -Fill PDU and Sends nFAPI DL TTI req to OAI PHY
 *
 * @params[in]    timing info
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint16_t OAI_OSC_fillDlTtiReq(SlotTimingInfo currTimingInfo)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : DL_TTI_REQUEST\n");
   DU_LOG("\nINFO  --> %s()\n", __FUNCTION__);
#endif
   uint8_t idx = 0;
   uint8_t nPdu = 0;
   uint8_t numPduEncoded = 0;
   uint8_t ueIdx;
   uint16_t cellIdx = 0;
   uint16_t pduIndex = 0;

   SlotTimingInfo dlTtiReqTimingInfo;
   MacDlSlot *currDlSlot = NULLP;
   MacCellCfg macCellCfg;
   RntiType rntiType;
   nfapi_nr_dl_tti_request_t *dlTtiReq = NULLP;
   if (lwrMacCb.phyState == PHY_STATE_RUNNING)
   {
      GET_CELL_IDX(currTimingInfo.cellId, cellIdx);
      /* consider phy delay */
      ADD_DELTA_TO_TIME(currTimingInfo, dlTtiReqTimingInfo, PHY_DELTA_DL, macCb.macCell[cellIdx]->numOfSlots);
      DU_LOG("\nINFO  ->  The current Timing Info : sfn : %d, slot : %d\n", currTimingInfo.sfn, currTimingInfo.slot);
      DU_LOG("\nINFO  ->  The DL TTI Timing Info : sfn : %d, slot : %d\n", dlTtiReqTimingInfo.sfn, dlTtiReqTimingInfo.slot);
      dlTtiReqTimingInfo.cellId = currTimingInfo.cellId;

      macCellCfg = macCb.macCell[cellIdx]->macCellCfg;

      currDlSlot = &macCb.macCell[cellIdx]->dlSlot[dlTtiReqTimingInfo.slot];
      dlTtiReq = (nfapi_nr_dl_tti_request_t *)malloc(sizeof(nfapi_nr_dl_tti_request_t));
      memset(dlTtiReq, 0, sizeof(nfapi_nr_dl_tti_request_t));
      
      /* Fill Dl TTI Request */
      nfapi_vnf_p7_config_t *p7_config = glb_vnf->p7_vnfs[0].config;
      dlTtiReq->header.message_id = NFAPI_NR_PHY_MSG_TYPE_DL_TTI_REQUEST;
      dlTtiReq->header.phy_id = 1;

      dlTtiReq->SFN = dlTtiReqTimingInfo.sfn;
      dlTtiReq->Slot = dlTtiReqTimingInfo.slot;
      dlTtiReq->dl_tti_request_body.nPDUs = calcDlTtiReqPduCount(currDlSlot); /* get total Pdus */
      nPdu = dlTtiReq->dl_tti_request_body.nPDUs;
      dlTtiReq->dl_tti_request_body.nGroup = 0;

      DU_LOG("\n[DEBUG] -->  dlTtiReq->dl_tti_request_body.nPDUs:%d\n",dlTtiReq->dl_tti_request_body.nPDUs);

      if (dlTtiReq->dl_tti_request_body.nPDUs > 0) // dlTtiReq->dl_tti_request_body.nPDUs > 0
      {
         if (currDlSlot->dlInfo.isBroadcastPres)
         {
            if (currDlSlot->dlInfo.brdcstAlloc.ssbTransmissionMode)
            {
               if (dlTtiReq->dl_tti_request_body.dl_tti_pdu_list != NULLP)
               {
                  for (idx = 0; idx < currDlSlot->dlInfo.brdcstAlloc.ssbIdxSupported; idx++)
                  {
                     OAI_OSC_fillSsbPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded], &macCellCfg,\
                                currDlSlot, idx, dlTtiReq->SFN);
                     numPduEncoded++;
                  }
               }
               DU_LOG("\033[1;31m");
               printf("\nDEBUG  -->  LWR_MAC: MIB sent..");
               DU_LOG("\033[0m");
            }

            if (currDlSlot->dlInfo.brdcstAlloc.sib1TransmissionMode)
            {
               /* Filling SIB1 param */
               if (numPduEncoded != nPdu)
               {
                  if(currDlSlot->dlInfo.brdcstAlloc.crnti == SI_RNTI)
                     rntiType = SI_RNTI_TYPE;
                  
                  /* PDCCH PDU */
                  OAI_OSC_fillPdcchPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded], currDlSlot, -1,\
                               rntiType, CORESET_TYPE0, MAX_NUM_UE);
                  numPduEncoded++;

                  /* PDSCH PDU */
                  OAI_OSC_fillPdschPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded],\
                               &currDlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg->dci[0].pdschCfg,\
                               currDlSlot->dlInfo.brdcstAlloc.sib1Alloc.bwp,\
                               pduIndex);
                  dlTtiReq->dl_tti_request_body.PduIdx[dlTtiReq->dl_tti_request_body.nGroup][pduIndex] = pduIndex;
                  pduIndex++;
                  numPduEncoded++;
               }
               DU_LOG("\033[1;34m");
               DU_LOG("\nDEBUG  -->  LWR_MAC: SIB1 sent...");
               DU_LOG("\033[0m");
            }
         }
/*
         //TODO: ADD Paging done
         if(currDlSlot->pageAllocInfo != NULLP)
         {
            // Filling DL Paging Alloc param
            if(numPduEncoded != nPdu)
            {
               rntiType = P_RNTI_TYPE;
               //TODO:OAI_OSC_fillPagePdcchPdu done
               OAI_OSC_fillPagePdcchPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded], \
                                 currDlSlot->pageAllocInfo, &macCellCfg);
               numPduEncoded++;
               //TODO:OAI_OSC_fillPagePdschPdu done
               OAI_OSC_fillPagePdschPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded], \
                                 currDlSlot->pageAllocInfo, pduIndex, &macCellCfg);
               dlTtiReq->dl_tti_request_body.PduIdx[pduIndex][0] = pduIndex;
               pduIndex++;
               numPduEncoded++;
            }
            DU_LOG("\033[1;34m");
            DU_LOG("\nDEBUG  -->  LWR_MAC: PAGE sent...");
            DU_LOG("\033[0m");
         }*/

         for (ueIdx = 0; ueIdx < MAX_NUM_UE; ueIdx++)
         {
            if (currDlSlot->dlInfo.rarAlloc[ueIdx] != NULLP)
            {
               /* Filling RAR param */
               rntiType = RA_RNTI_TYPE;
               if(currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg)
               {
                  OAI_OSC_fillPdcchPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded],\
                               currDlSlot, -1, rntiType, CORESET_TYPE0, ueIdx);
                  numPduEncoded++;
               }
				   if(currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg)
               {
                  OAI_OSC_fillPdschPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded],\
                               &currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg->dci[0].pdschCfg,\
                               currDlSlot->dlInfo.rarAlloc[ueIdx]->bwp,\
                               pduIndex);
                  numPduEncoded++;
                  pduIndex++;
                  MAC_FREE(currDlSlot->dlInfo.rarAlloc[ueIdx]->rarPdcchCfg, sizeof(PdcchCfg));

                  DU_LOG("\033[1;32m");
                  DU_LOG("\nDEBUG  -->  LWR_MAC: RAR sent...");
                  DU_LOG("\033[0m");
               }
            }
            
            /* Filling Msg4 param */
            if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx] != NULLP)
            {
               if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg) 
               {
                  rntiType = C_RNTI_TYPE;
                  OAI_OSC_fillPdcchPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded],\
                              currDlSlot, idx, rntiType, currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg->coresetCfg.coreSetType, ueIdx);            
                  numPduEncoded++;
               }

               if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdu != NULLP)
               {
                  if(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg)
                  {
                     OAI_OSC_fillPdschPdu(&dlTtiReq->dl_tti_request_body.dl_tti_pdu_list[numPduEncoded],\
                                     currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg,\
                                     currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->bwp, pduIndex);
                        numPduEncoded++;
                        pduIndex++;
                        

                        DU_LOG("\033[1;32m");
                        if((macCb.macCell[cellIdx]->macRaCb[ueIdx].macMsg4Status))
                        {
                           DU_LOG("\nDEBUG  -->  LWR_MAC: MSG4 sent...");
                           MAC_FREE(macCb.macCell[cellIdx]->macRaCb[ueIdx].macMsg4Status, sizeof(bool));
                        }
                        else
                        {
                           DU_LOG("\nDEBUG  -->  LWR_MAC: DL MSG sent...");
                        }
                        DU_LOG("\033[0m");
                  }
               }
                  MAC_FREE(currDlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdcchCfg,sizeof(PdcchCfg));
                  /*   else
                     {
                        MAC_FREE(currDlSlot->dlInfo.dlMsgAlloc[ueIdx], sizeof(DlMsgAlloc));
                        currDlSlot->dlInfo.dlMsgAlloc[ueIdx] = NULLP;
                     }
                     */
            }
            
         }

         dlTtiReq->dl_tti_request_body.nUe[dlTtiReq->dl_tti_request_body.nGroup] = MAX_NUM_UE_PER_TTI;
         dlTtiReq->dl_tti_request_body.nGroup=0; //testing
         
         int retval = nfapi_vnf_p7_nr_dl_config_req(p7_config, dlTtiReq);

#ifdef ODU_SLOT_IND_DEBUG_LOG
         DU_LOG("\nDEBUG  -->  LWR_MAC: Sending DL TTI Request");
#endif
         /* OAI L1 expects UL_TTI.request following DL_TTI.request */
         OAI_OSC_fillUlTtiReq(currTimingInfo);

         /* OAI L1 expects UL_DCI.request following DL_TTI.request */
         OAI_OSC_fillUlDciReq(dlTtiReqTimingInfo);

         /* send Tx-DATA req message */
         OAI_OSC_sendTxDataReq(dlTtiReqTimingInfo, currDlSlot);

         //int retval = nfapi_vnf_p7_nr_dl_config_req(p7_config, dlTtiReq);
      }
      else
      {
#ifdef ODU_SLOT_IND_DEBUG_LOG
         DU_LOG("\nDEBUG  -->  LWR_MAC: Sending DL TTI Request");
#endif
         /* OAI L1 expects UL_TTI.request following DL_TTI.request */
         // OAI_OSC_fillUlTtiReq(currTimingInfo);

         /* OAI L1 expects UL_DCI.request following DL_TTI.request */
         // OAI_OSC_fillUlDciReq(dlTtiReqTimingInfo);
      }
      memset(currDlSlot, 0, sizeof(MacDlSlot));
            return ROK;
   }
   else
   {
      DU_LOG("\nERROR  -->  Call lwr_mac_procInvalidEvt()\n");
      lwr_mac_procInvalidEvt(&currTimingInfo);
      return RFAILED;
   }
   return ROK;
}


/*******************************************************************
 *
 * @brief Sends TX data Request to PHY
 *
 * @details
 *
 *    Function : sendTxDataReq
 *
 *    Functionality:
 *         -Sends FAPI TX data req to PHY
 *
 * @params[in]    timing info
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint16_t sendTxDataReq(SlotTimingInfo currTimingInfo, MacDlSlot *dlSlot, p_fapi_api_queue_elem_t prevElem, fapi_vendor_tx_data_req_t *vendorTxDataReq)
{
#ifdef INTEL_FAPI
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : TX_DATA_REQ\n");
#endif

   uint8_t  nPdu = 0;
   uint8_t  ueIdx=0;
   uint16_t cellIdx=0;
   uint16_t pduIndex = 0;
   fapi_tx_data_req_t       *txDataReq =NULLP;
   p_fapi_api_queue_elem_t  txDataElem = 0;

   GET_CELL_IDX(currTimingInfo.cellId, cellIdx);

   /* send TX_Data request message */
   nPdu = calcTxDataReqPduCount(dlSlot);
   if(nPdu > 0)
   {
      LWR_MAC_ALLOC(txDataElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_tx_data_req_t)));
      if(txDataElem == NULLP)
      {
         DU_LOG("\nERROR  -->  LWR_MAC: Failed to allocate memory for TX data Request");
         return RFAILED;
      }

      FILL_FAPI_LIST_ELEM(txDataElem, NULLP, FAPI_TX_DATA_REQUEST, 1, \
            sizeof(fapi_tx_data_req_t));
      txDataReq = (fapi_tx_data_req_t *)(txDataElem +1);
      memset(txDataReq, 0, sizeof(fapi_tx_data_req_t));
      fillMsgHeader(&txDataReq->header, FAPI_TX_DATA_REQUEST, sizeof(fapi_tx_data_req_t));

      vendorTxDataReq->sym = 0;

      txDataReq->sfn  = currTimingInfo.sfn;
      txDataReq->slot = currTimingInfo.slot;
      if(dlSlot->dlInfo.brdcstAlloc.sib1TransmissionMode)
      {
         fillSib1TxDataReq(txDataReq->pdu_desc, pduIndex, &macCb.macCell[cellIdx]->macCellCfg, \
               &dlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg->dci[0].pdschCfg);
         pduIndex++;
         MAC_FREE(dlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg,sizeof(PdcchCfg));
         txDataReq->num_pdus++;
      }
      if(dlSlot->pageAllocInfo != NULLP)
      {
         fillPageTxDataReq(txDataReq->pdu_desc, pduIndex, dlSlot->pageAllocInfo);
         pduIndex++;
         txDataReq->num_pdus++;
         MAC_FREE(dlSlot->pageAllocInfo->pageDlSch.dlPagePdu, sizeof(dlSlot->pageAllocInfo->pageDlSch.dlPagePduLen));
         MAC_FREE(dlSlot->pageAllocInfo,sizeof(DlPageAlloc));
      }

      for(ueIdx=0; ueIdx<MAX_NUM_UE; ueIdx++)
      {
         if(dlSlot->dlInfo.rarAlloc[ueIdx] != NULLP)
         {
            if((dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg))
            {
               fillRarTxDataReq(txDataReq->pdu_desc, pduIndex, &dlSlot->dlInfo.rarAlloc[ueIdx]->rarInfo,\
                     dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg);
               pduIndex++;
               txDataReq->num_pdus++;
               MAC_FREE(dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg, sizeof(PdschCfg));
            }
            MAC_FREE(dlSlot->dlInfo.rarAlloc[ueIdx],sizeof(RarAlloc));
         }

         if(dlSlot->dlInfo.dlMsgAlloc[ueIdx] != NULLP)
         {
            if(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg) 
            {
               fillDlMsgTxDataReq(txDataReq->pdu_desc, pduIndex, \
                     dlSlot->dlInfo.dlMsgAlloc[ueIdx], \
                     dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg);
               pduIndex++;
               txDataReq->num_pdus++;
               MAC_FREE(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg,sizeof(PdschCfg));
            }
            MAC_FREE(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdu, \
                  dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPduLen);
            dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdu = NULLP;
            MAC_FREE(dlSlot->dlInfo.dlMsgAlloc[ueIdx], sizeof(DlMsgSchInfo));
         }
      }

      /* Fill message header */
      DU_LOG("\nDEBUG  -->  LWR_MAC: Sending TX DATA Request");
      prevElem->p_next = txDataElem;
   }
#endif
   return ROK;
}

/***********************************************************************
 *
 * @brief calculates the total size to be allocated for UL TTI Req
 *
 * @details
 *
 *    Function : getnPdus
 *
 *    Functionality:
 *         -calculates the total pdu count to be allocated for UL TTI Req
 *
 * @params[in] Pointer to fapi Ul TTI Req
 *             Pointer to CurrUlSlot
 * @return count
 * ********************************************************************/
#ifdef INTEL_FAPI
uint8_t getnPdus(fapi_ul_tti_req_t *ulTtiReq, MacUlSlot *currUlSlot)
{
   uint8_t pduCount = 0;

   if(ulTtiReq && currUlSlot)
   {
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PRACH)
      {
	 pduCount++;
	 ulTtiReq->rachPresent++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PUSCH)
      {
	 pduCount++;
	 ulTtiReq->nUlsch++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PUSCH_UCI)
      {
	 pduCount++;
	 ulTtiReq->nUlsch++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_UCI)
      {
	 pduCount++;
	 ulTtiReq->nUlcch++;
      }
      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_SRS)
      {
	 pduCount++;
      }
   }
   return pduCount;
}
#endif

/***********************************************************************
 *
 * @brief Set the value of zero correlation config in PRACH PDU
 *
 * @details
 *
 *    Function : setNumCs
 *
 *    Functionality:
 *         -Set the value of zero correlation config in PRACH PDU
 *
 * @params[in] Pointer to zero correlation config
 *             Pointer to MacCellCfg
 * ********************************************************************/

void setNumCs(uint16_t *numCs, MacCellCfg *macCellCfg)
{
   uint8_t idx;
   if(macCellCfg != NULLP)
   {
      idx = macCellCfg->prachCfg.fdm[0].zeroCorrZoneCfg; 
      *numCs = UnrestrictedSetNcsTable[idx];
   }
}

/***********************************************************************
 *
 * @brief Fills the PRACH PDU in UL TTI Request
 *
 * @details
 *
 *    Function : fillPrachPdu
 *
 *    Functionality:
 *         -Fills the PRACH PDU in UL TTI Request
 *
 * @params[in] Pointer to Prach Pdu
 *             Pointer to CurrUlSlot
 *             Pointer to macCellCfg
 *             Pointer to msgLen
 * ********************************************************************/

#ifdef INTEL_FAPI
void fillPrachPdu(fapi_ul_tti_req_pdu_t *ulTtiReqPdu, MacCellCfg *macCellCfg, MacUlSlot *currUlSlot)
{
   if(ulTtiReqPdu != NULLP)
   {
      ulTtiReqPdu->pduType = PRACH_PDU_TYPE; 
      ulTtiReqPdu->pdu.prach_pdu.physCellId = macCellCfg->cellCfg.phyCellId;
      ulTtiReqPdu->pdu.prach_pdu.numPrachOcas = \
         currUlSlot->ulInfo.prachSchInfo.numPrachOcas;
      ulTtiReqPdu->pdu.prach_pdu.prachFormat = \
	 currUlSlot->ulInfo.prachSchInfo.prachFormat;
      ulTtiReqPdu->pdu.prach_pdu.numRa = currUlSlot->ulInfo.prachSchInfo.numRa;
      ulTtiReqPdu->pdu.prach_pdu.prachStartSymbol = \
	 currUlSlot->ulInfo.prachSchInfo.prachStartSymb;
      setNumCs(&ulTtiReqPdu->pdu.prach_pdu.numCs, macCellCfg);
      ulTtiReqPdu->pdu.prach_pdu.beamforming.numPrgs = 0;
      ulTtiReqPdu->pdu.prach_pdu.beamforming.prgSize = 0;
      ulTtiReqPdu->pdu.prach_pdu.beamforming.digBfInterface = 0;
      ulTtiReqPdu->pdu.prach_pdu.beamforming.rx_bfi[0].beamIdx[0].beamidx = 0;
      ulTtiReqPdu->pduSize = sizeof(fapi_ul_prach_pdu_t); 
   }
}

/*******************************************************************
 *
 * @brief Filling PUSCH PDU in UL TTI Request
 *
 * @details
 *
 *    Function : fillPuschPdu
 *
 *    Functionality: Filling PUSCH PDU in UL TTI Request
 *
 * @params[in] 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
void fillPuschPdu(fapi_ul_tti_req_pdu_t *ulTtiReqPdu, fapi_vendor_ul_tti_req_pdu_t *ulTtiVendorPdu, MacCellCfg *macCellCfg, MacUlSlot *currUlSlot)
{
   if(ulTtiReqPdu != NULLP)
   {
      ulTtiReqPdu->pduType = PUSCH_PDU_TYPE;
      memset(&ulTtiReqPdu->pdu.pusch_pdu, 0, sizeof(fapi_ul_pusch_pdu_t));
      ulTtiReqPdu->pdu.pusch_pdu.pduBitMap = 1;
      ulTtiReqPdu->pdu.pusch_pdu.rnti = currUlSlot->ulInfo.crnti;
      /* TODO : Fill handle in raCb when scheduling pusch and access here */
      ulTtiReqPdu->pdu.pusch_pdu.handle = 100;
      ulTtiReqPdu->pdu.pusch_pdu.bwpSize = macCellCfg->cellCfg.initialUlBwp.bwp.numPrb;
      ulTtiReqPdu->pdu.pusch_pdu.bwpStart = macCellCfg->cellCfg.initialUlBwp.bwp.firstPrb;
      ulTtiReqPdu->pdu.pusch_pdu.subCarrierSpacing = \
         macCellCfg->cellCfg.initialUlBwp.bwp.scs;
      ulTtiReqPdu->pdu.pusch_pdu.cyclicPrefix = \
         macCellCfg->cellCfg.initialUlBwp.bwp.cyclicPrefix;
      ulTtiReqPdu->pdu.pusch_pdu.targetCodeRate = 308;
      ulTtiReqPdu->pdu.pusch_pdu.qamModOrder = currUlSlot->ulInfo.schPuschInfo.tbInfo.qamOrder;
      ulTtiReqPdu->pdu.pusch_pdu.mcsIndex = currUlSlot->ulInfo.schPuschInfo.tbInfo.mcs;
      ulTtiReqPdu->pdu.pusch_pdu.mcsTable = currUlSlot->ulInfo.schPuschInfo.tbInfo.mcsTable;
      ulTtiReqPdu->pdu.pusch_pdu.transformPrecoding = 1;
      ulTtiReqPdu->pdu.pusch_pdu.dataScramblingId = currUlSlot->ulInfo.cellId;
      ulTtiReqPdu->pdu.pusch_pdu.nrOfLayers = 1;
      ulTtiReqPdu->pdu.pusch_pdu.ulDmrsSymbPos = 4;
      ulTtiReqPdu->pdu.pusch_pdu.dmrsConfigType = 0;
      ulTtiReqPdu->pdu.pusch_pdu.ulDmrsScramblingId = currUlSlot->ulInfo.cellId;
      ulTtiReqPdu->pdu.pusch_pdu.scid = 0;
      ulTtiReqPdu->pdu.pusch_pdu.numDmrsCdmGrpsNoData = 1;
      ulTtiReqPdu->pdu.pusch_pdu.dmrsPorts = 0;
      ulTtiReqPdu->pdu.pusch_pdu.resourceAlloc = \
	 currUlSlot->ulInfo.schPuschInfo.fdAlloc.resAllocType;
      ulTtiReqPdu->pdu.pusch_pdu.rbStart = \
         currUlSlot->ulInfo.schPuschInfo.fdAlloc.resAlloc.type1.startPrb;
      ulTtiReqPdu->pdu.pusch_pdu.rbSize = \
	 currUlSlot->ulInfo.schPuschInfo.fdAlloc.resAlloc.type1.numPrb;
      ulTtiReqPdu->pdu.pusch_pdu.vrbToPrbMapping = 0;
      ulTtiReqPdu->pdu.pusch_pdu.frequencyHopping = 0;
      ulTtiReqPdu->pdu.pusch_pdu.txDirectCurrentLocation = 0;
      ulTtiReqPdu->pdu.pusch_pdu.uplinkFrequencyShift7p5khz = 0;
      ulTtiReqPdu->pdu.pusch_pdu.startSymbIndex = \
         currUlSlot->ulInfo.schPuschInfo.tdAlloc.startSymb;
      ulTtiReqPdu->pdu.pusch_pdu.nrOfSymbols = \
         currUlSlot->ulInfo.schPuschInfo.tdAlloc.numSymb;
#ifdef INTEL_FAPI
      ulTtiReqPdu->pdu.pusch_pdu.mappingType = \
         currUlSlot->ulInfo.schPuschInfo.dmrsMappingType;
      ulTtiReqPdu->pdu.pusch_pdu.nrOfDmrsSymbols = \
         currUlSlot->ulInfo.schPuschInfo.nrOfDmrsSymbols;
      ulTtiReqPdu->pdu.pusch_pdu.dmrsAddPos = \
         currUlSlot->ulInfo.schPuschInfo.dmrsAddPos;
#endif
      ulTtiReqPdu->pdu.pusch_pdu.puschData.rvIndex = \
         currUlSlot->ulInfo.schPuschInfo.tbInfo.rv;
      ulTtiReqPdu->pdu.pusch_pdu.puschData.harqProcessId = \
         currUlSlot->ulInfo.schPuschInfo.harqProcId;
      ulTtiReqPdu->pdu.pusch_pdu.puschData.newDataIndicator = \
         currUlSlot->ulInfo.schPuschInfo.tbInfo.ndi;
      ulTtiReqPdu->pdu.pusch_pdu.puschData.tbSize = \
         currUlSlot->ulInfo.schPuschInfo.tbInfo.tbSize;
      /* numCb is 0 for new transmission */
      ulTtiReqPdu->pdu.pusch_pdu.puschData.numCb = 0;

      ulTtiReqPdu->pduSize = sizeof(fapi_ul_pusch_pdu_t);

      /* UL TTI Vendor PDU */
      ulTtiVendorPdu->pdu_type = FAPI_PUSCH_PDU_TYPE;
      ulTtiVendorPdu->pdu.pusch_pdu.nr_of_antenna_ports=1;
      ulTtiVendorPdu->pdu.pusch_pdu.nr_of_rx_ru=1;
      for(int i =0; i< FAPI_VENDOR_MAX_RXRU_NUM; i++)
      {
	      ulTtiVendorPdu->pdu.pusch_pdu.rx_ru_idx[i]=0;
      }
   }
}

/*******************************************************************
 *
 * @brief Fill PUCCH PDU in Ul TTI Request
 *
 * @details
 *
 *    Function : fillPucchPdu
 *
 *    Functionality: Fill PUCCH PDU in Ul TTI Request
 *
 * @params[in] 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
void fillPucchPdu(fapi_ul_tti_req_pdu_t *ulTtiReqPdu, fapi_vendor_ul_tti_req_pdu_t *ulTtiVendorPdu, MacCellCfg *macCellCfg,\
      MacUlSlot *currUlSlot)
{
   if(ulTtiReqPdu != NULLP)
   {
      ulTtiReqPdu->pduType                  = PUCCH_PDU_TYPE;
      memset(&ulTtiReqPdu->pdu.pucch_pdu, 0, sizeof(fapi_ul_pucch_pdu_t));
      ulTtiReqPdu->pdu.pucch_pdu.rnti         = currUlSlot->ulInfo.crnti;
      /* TODO : Fill handle in raCb when scheduling pucch and access here */
      ulTtiReqPdu->pdu.pucch_pdu.handle       = 100;
      ulTtiReqPdu->pdu.pucch_pdu.bwpSize      = macCellCfg->cellCfg.initialUlBwp.bwp.numPrb;
      ulTtiReqPdu->pdu.pucch_pdu.bwpStart     = macCellCfg->cellCfg.initialUlBwp.bwp.firstPrb;
      ulTtiReqPdu->pdu.pucch_pdu.subCarrierSpacing = macCellCfg->cellCfg.initialUlBwp.bwp.scs;
      ulTtiReqPdu->pdu.pucch_pdu.cyclicPrefix = macCellCfg->cellCfg.initialUlBwp.bwp.cyclicPrefix;
      ulTtiReqPdu->pdu.pucch_pdu.formatType   = currUlSlot->ulInfo.schPucchInfo.pucchFormat; /* Supporting PUCCH Format 0 */
      ulTtiReqPdu->pdu.pucch_pdu.multiSlotTxIndicator = 0; /* No Multi Slot transmission */
      
      ulTtiReqPdu->pdu.pucch_pdu.prbStart     = currUlSlot->ulInfo.schPucchInfo.fdAlloc.startPrb;
      ulTtiReqPdu->pdu.pucch_pdu.prbSize      = currUlSlot->ulInfo.schPucchInfo.fdAlloc.numPrb;
      ulTtiReqPdu->pdu.pucch_pdu.startSymbolIndex = currUlSlot->ulInfo.schPucchInfo.tdAlloc.startSymb;
      ulTtiReqPdu->pdu.pucch_pdu.nrOfSymbols  = currUlSlot->ulInfo.schPucchInfo.tdAlloc.numSymb;
      ulTtiReqPdu->pdu.pucch_pdu.freqHopFlag  = currUlSlot->ulInfo.schPucchInfo.intraFreqHop;
      ulTtiReqPdu->pdu.pucch_pdu.secondHopPrb = currUlSlot->ulInfo.schPucchInfo.secondPrbHop;
      ulTtiReqPdu->pdu.pucch_pdu.groupHopFlag = 0;     
      ulTtiReqPdu->pdu.pucch_pdu.sequenceHopFlag = 0;
      ulTtiReqPdu->pdu.pucch_pdu.hoppingId    = 0;

      ulTtiReqPdu->pdu.pucch_pdu.initialCyclicShift = currUlSlot->ulInfo.schPucchInfo.initialCyclicShift;

      ulTtiReqPdu->pdu.pucch_pdu.dataScramblingId = 0; /* Valid for Format 2, 3, 4 */
      ulTtiReqPdu->pdu.pucch_pdu.timeDomainOccIdx = currUlSlot->ulInfo.schPucchInfo.timeDomOCC; 
      ulTtiReqPdu->pdu.pucch_pdu.preDftOccIdx = currUlSlot->ulInfo.schPucchInfo.occIdx; /* Valid for Format 4 only */
      ulTtiReqPdu->pdu.pucch_pdu.preDftOccLen = currUlSlot->ulInfo.schPucchInfo.occLen; /* Valid for Format 4 only */
      ulTtiReqPdu->pdu.pucch_pdu.pi2Bpsk = currUlSlot->ulInfo.schPucchInfo.pi2BPSK;
      ulTtiReqPdu->pdu.pucch_pdu.addDmrsFlag = currUlSlot->ulInfo.schPucchInfo.addDmrs;/* Valid for Format 3, 4 only */
      ulTtiReqPdu->pdu.pucch_pdu.dmrsScramblingId = 0; /* Valid for Format 2 */
      ulTtiReqPdu->pdu.pucch_pdu.dmrsCyclicShift  = 0; /* Valid for Format 4 */
      ulTtiReqPdu->pdu.pucch_pdu.srFlag           = currUlSlot->ulInfo.schPucchInfo.srFlag;
      ulTtiReqPdu->pdu.pucch_pdu.bitLenHarq       = currUlSlot->ulInfo.schPucchInfo.harqInfo.harqBitLength;
      ulTtiReqPdu->pdu.pucch_pdu.bitLenCsiPart1   = 0; /* Valid for Format 2, 3, 4 */
      ulTtiReqPdu->pdu.pucch_pdu.bitLenCsiPart2   = 0; /* Valid for Format 2, 3, 4 */
      ulTtiReqPdu->pdu.pucch_pdu.beamforming.numPrgs = currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.numPrgs; 
      ulTtiReqPdu->pdu.pucch_pdu.beamforming.prgSize = currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.prgSize;
      ulTtiReqPdu->pdu.pucch_pdu.beamforming.digBfInterface = currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.digBfInterfaces;
      ulTtiReqPdu->pdu.pucch_pdu.beamforming.rx_bfi[0].beamIdx[0].beamidx = currUlSlot->ulInfo.schPucchInfo.beamPucchInfo.prg[0].beamIdx[0];

      ulTtiReqPdu->pduSize = sizeof(fapi_ul_pucch_pdu_t);

      /* UL TTI Vendor PDU */
      ulTtiVendorPdu->pdu_type = FAPI_PUCCH_PDU_TYPE;
      ulTtiVendorPdu->pdu.pucch_pdu.nr_of_rx_ru=1;
      ulTtiVendorPdu->pdu.pucch_pdu.group_id=0;
      for(int i =0; i<FAPI_VENDOR_MAX_RXRU_NUM; i++)
      {
	      ulTtiVendorPdu->pdu.pucch_pdu.rx_ru_idx[i]=0;
      }
   }
}

#endif

/*******************************************************************
 *
 * @brief Sends UL TTI Request to PHY
 *
 * @details
 *
 *    Function : fillUlTtiReq
 *
 *    Functionality:
 *         -Sends FAPI Param req to PHY
 *
 * @params[in]  Pointer to CmLteTimingInfo
 * @return ROK     - success
 *         RFAILED - failure
 *
 ******************************************************************/
uint16_t fillUlTtiReq(SlotTimingInfo currTimingInfo, p_fapi_api_queue_elem_t prevElem, fapi_vendor_ul_tti_req_t* vendorUlTti)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : UL_TTI_REQUEST\n");
#endif

#ifdef INTEL_FAPI
   uint16_t   cellIdx =0;
   uint8_t    pduIdx = -1;
   SlotTimingInfo ulTtiReqTimingInfo;
   MacUlSlot *currUlSlot = NULLP;
   MacCellCfg macCellCfg;
   fapi_ul_tti_req_t *ulTtiReq = NULLP;
   p_fapi_api_queue_elem_t ulTtiElem;

   if(lwrMacCb.phyState == PHY_STATE_RUNNING)
   {
      GET_CELL_IDX(currTimingInfo.cellId, cellIdx);
      macCellCfg = macCb.macCell[cellIdx]->macCellCfg;

      /* add PHY delta */
      ADD_DELTA_TO_TIME(currTimingInfo,ulTtiReqTimingInfo,PHY_DELTA_UL, macCb.macCell[cellIdx]->numOfSlots);
      currUlSlot = &macCb.macCell[cellIdx]->ulSlot[ulTtiReqTimingInfo.slot % macCb.macCell[cellIdx]->numOfSlots];

      LWR_MAC_ALLOC(ulTtiElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_ul_tti_req_t)));
      if(ulTtiElem)
      {
	      FILL_FAPI_LIST_ELEM(ulTtiElem, NULLP, FAPI_UL_TTI_REQUEST, 1, \
			      sizeof(fapi_ul_tti_req_t));
	      ulTtiReq = (fapi_ul_tti_req_t *)(ulTtiElem +1);
	      memset(ulTtiReq, 0, sizeof(fapi_ul_tti_req_t));
	      fillMsgHeader(&ulTtiReq->header, FAPI_UL_TTI_REQUEST, sizeof(fapi_ul_tti_req_t));
	      ulTtiReq->sfn  = ulTtiReqTimingInfo.sfn;
	      ulTtiReq->slot = ulTtiReqTimingInfo.slot;
	      ulTtiReq->nPdus = getnPdus(ulTtiReq, currUlSlot);
	      vendorUlTti->num_ul_pdu =  ulTtiReq->nPdus;
	      vendorUlTti->sym = 0;
	      ulTtiReq->nGroup = 0;
	      if(ulTtiReq->nPdus > 0)
	      {
		      /* Fill Prach Pdu */
		      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PRACH)
		      {
			      pduIdx++;
			      fillPrachPdu(&ulTtiReq->pdus[pduIdx], &macCellCfg, currUlSlot);
			      ulTtiReq->rachPresent++;
		      }

		      /* Fill PUSCH PDU */
		      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PUSCH)
		      {
			      pduIdx++;
			      fillPuschPdu(&ulTtiReq->pdus[pduIdx], &vendorUlTti->ul_pdus[pduIdx], &macCellCfg, currUlSlot);
			      ulTtiReq->nUlsch++;
		      }
		      /* Fill PUCCH PDU */
		      if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_UCI)
		      {
			      pduIdx++;
			      fillPucchPdu(&ulTtiReq->pdus[pduIdx], &vendorUlTti->ul_pdus[pduIdx], &macCellCfg, currUlSlot);
			      ulTtiReq->nUlcch++;
		      }
	      } 

#ifdef ODU_SLOT_IND_DEBUG_LOG
	      DU_LOG("\nDEBUG  -->  LWR_MAC: Sending UL TTI Request");
#endif
	      prevElem->p_next = ulTtiElem;

	      memset(currUlSlot, 0, sizeof(MacUlSlot));
	      return ROK;
      }
      else
      {
	      DU_LOG("\nERROR  -->  LWR_MAC: Failed to allocate memory for UL TTI Request");
	      memset(currUlSlot, 0, sizeof(MacUlSlot));
	      return RFAILED;
      }
   }
   else
   {
	   lwr_mac_procInvalidEvt(&currTimingInfo);
   }
#endif
   return ROK;
}
/*******************************************************************
 *
 * @brief Sends UL TTI Request to OAI PHY
 *
 * @details
 *
 *    Function : OAI_OSC_fillUlTtiReq
 *
 *    Functionality:
 *         -Sends nFAPI Param req to OAI PHY
 *
 * @params[in]  Pointer to CmLteTimingInfo
 * @return ROK     - success
 *         RFAILED - failure
 *
 ******************************************************************/
uint16_t OAI_OSC_fillUlTtiReq(SlotTimingInfo currTimingInfo)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nINFO  -->  %s()\n", __FUNCTION__);
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : UL_TTI_REQUEST\n");
#endif
   uint16_t   cellIdx =0;
   uint8_t    pduIdx = -1;
   SlotTimingInfo ulTtiReqTimingInfo;
   MacUlSlot *currUlSlot = NULLP;
   MacCellCfg macCellCfg;
   nfapi_nr_ul_tti_request_t *ulTtiReq;

   if(lwrMacCb.phyState == PHY_STATE_RUNNING)
   {
      GET_CELL_IDX(currTimingInfo.cellId, cellIdx);
      macCellCfg = macCb.macCell[cellIdx]->macCellCfg;

      /* add PHY delta */
      ADD_DELTA_TO_TIME(currTimingInfo,ulTtiReqTimingInfo,PHY_DELTA_UL, macCb.macCell[cellIdx]->numOfSlots);
      currUlSlot = &macCb.macCell[cellIdx]->ulSlot[ulTtiReqTimingInfo.slot % macCb.macCell[cellIdx]->numOfSlots];
      ulTtiReq = (nfapi_nr_ul_tti_request_t *)malloc(sizeof(nfapi_nr_ul_tti_request_t));

      memset(ulTtiReq, 0, sizeof(nfapi_nr_ul_tti_request_t));
      ulTtiReq->header.phy_id = 1; // DJP HACK TODO FIXME - need to pass this around!!!!
      ulTtiReq->header.message_id = NFAPI_NR_PHY_MSG_TYPE_UL_TTI_REQUEST;
      ulTtiReq->header.message_length = sizeof(nfapi_nr_ul_tti_request_t);
      ulTtiReq->SFN = ulTtiReqTimingInfo.sfn;
      ulTtiReq->Slot = ulTtiReqTimingInfo.slot;
      ulTtiReq->n_pdus = OAI_OSC_getnPdus(ulTtiReq, currUlSlot);
      ulTtiReq->n_group = 0;
      DU_LOG("\nINFO  -->  ulTtiReq->n_pdus :%d\n",ulTtiReq->n_pdus); 

	   if(ulTtiReq->n_pdus > 0)
	   {
		   /* Fill Prach Pdu */
		   if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PRACH)
		   {
            pduIdx++;
            OAI_OSC_fillPrachPdu(&ulTtiReq->pdus_list[pduIdx], &macCellCfg, currUlSlot);
		   }
		   /* Fill PUSCH PDU */
		   if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_PUSCH)
		   {
            pduIdx++;
            OAI_OSC_fillPuschPdu(&ulTtiReq->pdus_list[pduIdx], &macCellCfg, currUlSlot);
            ulTtiReq->n_ulsch++;
		   }
		   /* Fill PUCCH PDU */
		   if(currUlSlot->ulInfo.dataType & SCH_DATATYPE_UCI)
		   {
            pduIdx++;
            OAI_OSC_fillPucchPdu(&ulTtiReq->pdus_list[pduIdx], &macCellCfg, currUlSlot);
            ulTtiReq->n_ulcch++;
		   }

         nfapi_vnf_p7_config_t *p7_config = glb_vnf->p7_vnfs[0].config; // vnf need to be global variable

         int retval = nfapi_vnf_p7_ul_tti_req(p7_config, ulTtiReq);
         if (retval != 0)
         {
            DU_LOG("\nERROR  -->  LWR_MAC: Failed to Send UL TTI Request");
            memset(currUlSlot, 0, sizeof(MacUlSlot));
            return RFAILED;
         }
	   }
      else
      {
	      memset(currUlSlot, 0, sizeof(MacUlSlot));
	      return RFAILED;
      }
   }
   else
   {
	   lwr_mac_procInvalidEvt(&currTimingInfo);
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Sends UL DCI Request to OAI PHY
 *
 * @details
 *
 *    Function : OAI_OSC_fillUlDciReq
 *
 *    Functionality:
 *         -Sends nFAPI Ul Dci req to OAI PHY
 *
 * @params[in]  Pointer to CmLteTimingInfo
 * @return ROK     - success
 *         RFAILED - failure
 *
 ******************************************************************/
uint16_t OAI_OSC_fillUlDciReq(SlotTimingInfo currTimingInfo)
{
   DU_LOG("\nINFO  -->  %s()\n", __FUNCTION__);
   uint8_t      cellIdx =0;
   uint8_t      numPduEncoded = 0;
   SlotTimingInfo  ulDciReqTimingInfo ={0};
   MacDlSlot    *currDlSlot = NULLP;
   nfapi_nr_ul_dci_request_t *ulDciReq = NULLP;

   if(lwrMacCb.phyState == PHY_STATE_RUNNING)
   { 
      nfapi_vnf_p7_config_t *p7_config = glb_vnf->p7_vnfs[0].config;
      GET_CELL_IDX(currTimingInfo.cellId, cellIdx);
      memcpy(&ulDciReqTimingInfo, &currTimingInfo, sizeof(SlotTimingInfo));
      currDlSlot = &macCb.macCell[cellIdx]->dlSlot[ulDciReqTimingInfo.slot % macCb.macCell[cellIdx]->numOfSlots];
      
      ulDciReq = (nfapi_nr_ul_dci_request_t *)malloc(sizeof(nfapi_nr_ul_dci_request_t));
      memset(ulDciReq, 0, sizeof(nfapi_nr_ul_dci_request_t));

      ulDciReq->header.message_id = NFAPI_NR_PHY_MSG_TYPE_UL_DCI_REQUEST;
      ulDciReq->header.phy_id = 1;

      ulDciReq->SFN = ulDciReqTimingInfo.sfn;
      ulDciReq->Slot = ulDciReqTimingInfo.slot;
   
      DU_LOG("\nDEBUG  -> Info : sfn : %d, slot : %d\n", ulDciReq->SFN, ulDciReq->Slot);

      if(currDlSlot->dlInfo.ulGrant != NULLP)
      {
         ulDciReq->numPdus = 1;  // No. of PDCCH PDUs
         if(ulDciReq->numPdus > 0)
         {
            DU_LOG("\nDEBUG  ->  currDlSlot->dlInfo.ulGrant != NULLP \n");
            /* Fill PDCCH configuration Pdu */
            OAI_OSC_fillUlDciPdcchPdu(&ulDciReq->ul_dci_pdu_list[numPduEncoded], &currDlSlot->dlInfo, CORESET_TYPE1);
            numPduEncoded++;
	         /* free UL GRANT at SCH */
	         MAC_FREE(currDlSlot->dlInfo.ulGrant, sizeof(DciInfo));
            int retval = nfapi_vnf_p7_ul_dci_req(p7_config, ulDciReq);
         }
#ifdef ODU_SLOT_IND_DEBUG_LOG
	       DU_LOG("\nDEBUG  -->  LWR_MAC: Sending UL DCI Request");
#endif
	  }
   }
   else
   {
       lwr_mac_procInvalidEvt(&currTimingInfo);
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Sends TX data Request to OAI PHY
 *
 * @details
 *
 *    Function : OAI_OSC_sendTxDataReq
 *
 *    Functionality:
 *         -Sends nFAPI TX data req to OAI PHY
 *
 * @params[in]    timing info
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint16_t OAI_OSC_sendTxDataReq(SlotTimingInfo currTimingInfo, MacDlSlot *dlSlot)
{

#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : TX_DATA_REQ\n");
   DU_LOG("INFO  -->  %s()\n", __FUNCTION__);
#endif
   uint8_t  nPdu = 0;
   uint8_t  ueIdx=0;
   uint16_t cellIdx=0;
   uint16_t pduIndex = 0;
   nfapi_nr_tx_data_request_t *txDataReq = NULLP;

   if(lwrMacCb.phyState == PHY_STATE_RUNNING)
   {
      GET_CELL_IDX(currTimingInfo.cellId, cellIdx);

      txDataReq = (nfapi_nr_tx_data_request_t *)malloc(sizeof(nfapi_nr_tx_data_request_t));
      memset(txDataReq, 0, sizeof(nfapi_nr_tx_data_request_t));

      nfapi_vnf_p7_config_t *p7_config = glb_vnf->p7_vnfs[0].config;
      /* Fill message header */
      txDataReq->header.message_id = NFAPI_NR_PHY_MSG_TYPE_TX_DATA_REQUEST;
      txDataReq->header.phy_id = 1;
      txDataReq->SFN = currTimingInfo.sfn;
      txDataReq->Slot = currTimingInfo.slot;
      /* send TX_Data request message */
      nPdu = calcTxDataReqPduCount(dlSlot);
      if(nPdu > 0)
      {
         if(dlSlot->dlInfo.brdcstAlloc.sib1TransmissionMode)
         {
            OAI_OSC_fillSib1TxDataReq(txDataReq->pdu_list, pduIndex, &macCb.macCell[cellIdx]->macCellCfg, \
                  &dlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg->dci[0].pdschCfg);
            pduIndex++;
            MAC_FREE(dlSlot->dlInfo.brdcstAlloc.sib1Alloc.sib1PdcchCfg,sizeof(PdcchCfg));
            txDataReq->Number_of_PDUs++;
         }
         if(dlSlot->pageAllocInfo != NULLP)
         {
            OAI_OSC_fillPageTxDataReq(txDataReq->pdu_list, pduIndex, dlSlot->pageAllocInfo);
            pduIndex++;
            txDataReq->Number_of_PDUs++;
            MAC_FREE(dlSlot->pageAllocInfo->pageDlSch.dlPagePdu, sizeof(dlSlot->pageAllocInfo->pageDlSch.dlPagePduLen));
            MAC_FREE(dlSlot->pageAllocInfo,sizeof(DlPageAlloc));
         }

         for(ueIdx=0; ueIdx<MAX_NUM_UE; ueIdx++)
         {
            if(dlSlot->dlInfo.rarAlloc[ueIdx] != NULLP)
            {
               if((dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg))
               {
                  OAI_OSC_fillRarTxDataReq(txDataReq->pdu_list, pduIndex, &dlSlot->dlInfo.rarAlloc[ueIdx]->rarInfo,\
                        dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg);
                  pduIndex++;
                  txDataReq->Number_of_PDUs++;
                  MAC_FREE(dlSlot->dlInfo.rarAlloc[ueIdx]->rarPdschCfg, sizeof(PdschCfg));
               }
               MAC_FREE(dlSlot->dlInfo.rarAlloc[ueIdx],sizeof(RarAlloc));
            }

            if(dlSlot->dlInfo.dlMsgAlloc[ueIdx] != NULLP)
            {
               if(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg) 
               {
                  OAI_OSC_fillDlMsgTxDataReq(txDataReq->pdu_list, pduIndex, \
                        dlSlot->dlInfo.dlMsgAlloc[ueIdx], \
                        dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg);
                  pduIndex++;
                  txDataReq->Number_of_PDUs++;
                  MAC_FREE(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdschCfg,sizeof(PdschCfg));
               }
               MAC_FREE(dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdu, \
                     dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPduLen);
               dlSlot->dlInfo.dlMsgAlloc[ueIdx]->dlMsgPdu = NULLP;
               MAC_FREE(dlSlot->dlInfo.dlMsgAlloc[ueIdx], sizeof(DlMsgSchInfo));
            }
         }
         int retval = nfapi_vnf_p7_tx_data_req(p7_config, txDataReq);
         DU_LOG("\nDEBUG  -->  LWR_MAC: Sending TX DATA Request");
      }
   }
   return ROK;
}

#ifdef INTEL_FAPI
/*******************************************************************
 *
 * @brief fills bsr Ul DCI PDU required for UL DCI Request to PHY
 *
 * @details
 *
 *    Function : fillUlDciPdu
 *
 *    Functionality:
 *         -Fills the Ul DCI PDU, spec Ref:38.212, Table 7.3.1-1
 *
 * @params[in] Pointer to fapi_dl_dci_t
 *             Pointer to DciInfo
 * @return ROK
 *
 ******************************************************************/
void fillUlDciPdu(fapi_dl_dci_t *ulDciPtr, DciInfo *schDciInfo)
{
#ifdef CALL_FLOW_DEBUG_LOG
   DU_LOG("\nCall Flow: ENTMAC -> ENTLWRMAC : UL_DCI_REQUEST\n");
#endif
   if(ulDciPtr != NULLP)
   {
      uint8_t numBytes =0;
      uint8_t bytePos =0;
      uint8_t bitPos =0;

      uint8_t  coreset1Size = 0;
      uint16_t rbStart = 0;
      uint16_t rbLen = 0;
      uint8_t  dciFormatId = 0;
      uint32_t freqDomResAssign =0;
      uint8_t  timeDomResAssign =0;
      uint8_t  freqHopFlag =0;
      uint8_t  modNCodScheme =0;
      uint8_t  ndi =0;
      uint8_t  redundancyVer = 0;
      uint8_t  harqProcessNum = 0;
      uint8_t  puschTpc = 0;
      uint8_t  ul_SlInd = 0;

      /* Size(in bits) of each field in DCI format 0_0 */
      uint8_t dciFormatIdSize      = 1;
      uint8_t freqDomResAssignSize = 0;
      uint8_t timeDomResAssignSize = 4;
      uint8_t freqHopFlagSize      = 1;
      uint8_t modNCodSchemeSize    = 5;
      uint8_t ndiSize              = 1;
      uint8_t redundancyVerSize    = 2;
      uint8_t harqProcessNumSize   = 4;
      uint8_t puschTpcSize         = 2;
      uint8_t ul_SlIndSize         = 1;

      ulDciPtr->rnti                          = schDciInfo->dciInfo.rnti;
      ulDciPtr->scramblingId                  = schDciInfo->dciInfo.scramblingId;    
      ulDciPtr->scramblingRnti                = schDciInfo->dciInfo.scramblingRnti;
      ulDciPtr->cceIndex                      = schDciInfo->dciInfo.cceIndex;
      ulDciPtr->aggregationLevel              = schDciInfo->dciInfo.aggregLevel;
      ulDciPtr->pc_and_bform.numPrgs          = schDciInfo->dciInfo.beamPdcchInfo.numPrgs;
      ulDciPtr->pc_and_bform.prgSize          = schDciInfo->dciInfo.beamPdcchInfo.prgSize;
      ulDciPtr->pc_and_bform.digBfInterfaces  = schDciInfo->dciInfo.beamPdcchInfo.digBfInterfaces;
      ulDciPtr->pc_and_bform.pmi_bfi[0].pmIdx = schDciInfo->dciInfo.beamPdcchInfo.prg[0].pmIdx;
      ulDciPtr->pc_and_bform.pmi_bfi[0].beamIdx[0].beamidx = schDciInfo->dciInfo.beamPdcchInfo.prg[0].beamIdx[0];
      ulDciPtr->beta_pdcch_1_0                = schDciInfo->dciInfo.txPdcchPower.beta_pdcch_1_0;           
      ulDciPtr->powerControlOffsetSS          = schDciInfo->dciInfo.txPdcchPower.powerControlOffsetSS;

      /* Calculating freq domain resource allocation field value and size
       * coreset1Size = Size of coreset 1
       * RBStart = Starting Virtual Rsource block
       * RBLen = length of contiguously allocted RBs
       * Spec 38.214 Sec 5.1.2.2.2
       */
      if(schDciInfo->dciFormatInfo.formatType == FORMAT0_0)
      {
         coreset1Size = schDciInfo->coresetCfg.coreSetSize;
         rbLen = schDciInfo->dciFormatInfo.format.format0_0.freqAlloc.resAlloc.type1.numPrb;
         rbStart = schDciInfo->dciFormatInfo.format.format0_0.freqAlloc.resAlloc.type1.startPrb;

         if((rbLen >=1) && (rbLen <= coreset1Size - rbStart))
         {
            if((rbLen - 1) <= floor(coreset1Size / 2))
               freqDomResAssign = (coreset1Size * (rbLen-1)) + rbStart;
            else
               freqDomResAssign = (coreset1Size * (coreset1Size - rbLen + 1)) \
                                  + (coreset1Size - 1 - rbStart);

            freqDomResAssignSize = ceil(log2(coreset1Size * (coreset1Size + 1) / 2));
         }
         /* Fetching DCI field values */
         dciFormatId      = schDciInfo->dciFormatInfo.formatType; /* DCI indentifier for UL DCI */
         timeDomResAssign = schDciInfo->dciFormatInfo.format.format0_0.rowIndex;
         freqHopFlag      = schDciInfo->dciFormatInfo.format.format0_0.freqHopFlag; 
         modNCodScheme    = schDciInfo->dciFormatInfo.format.format0_0.mcs;
         ndi              = schDciInfo->dciFormatInfo.format.format0_0.ndi; 
         redundancyVer    = schDciInfo->dciFormatInfo.format.format0_0.rvIndex;
         harqProcessNum   = schDciInfo->dciFormatInfo.format.format0_0.harqProcId; 
         puschTpc         = schDciInfo->dciFormatInfo.format.format0_0.tpcCmd;
         ul_SlInd         = schDciInfo->dciFormatInfo.format.format0_0.sulIndicator;
     
         /* Reversing bits in each DCI field */
         dciFormatId      = reverseBits(dciFormatId, dciFormatIdSize);
         freqDomResAssign = reverseBits(freqDomResAssign, freqDomResAssignSize);
         timeDomResAssign = reverseBits(timeDomResAssign, timeDomResAssignSize);
         modNCodScheme    = reverseBits(modNCodScheme, modNCodSchemeSize);
         redundancyVer    = reverseBits(redundancyVer, redundancyVerSize);
         harqProcessNum   = reverseBits(harqProcessNum, harqProcessNumSize);
         puschTpc         = reverseBits(puschTpc, puschTpcSize);
         ul_SlInd         = reverseBits(ul_SlInd, ul_SlIndSize);
      }
      /* Calulating total number of bytes in buffer */
      ulDciPtr->payloadSizeBits = (dciFormatIdSize + freqDomResAssignSize\
      + timeDomResAssignSize + freqHopFlagSize + modNCodSchemeSize + ndi \
      + redundancyVerSize + harqProcessNumSize + puschTpcSize + ul_SlIndSize);

      numBytes = ulDciPtr->payloadSizeBits / 8;
      if(ulDciPtr->payloadSizeBits % 8)
         numBytes += 1;

      if(numBytes > FAPI_DCI_PAYLOAD_BYTE_LEN)
      {
         DU_LOG("\nERROR  -->  LWR_MAC : Total bytes for DCI is more than expected");
         return;
      }

      /* Initialize buffer */
      for(bytePos = 0; bytePos < numBytes; bytePos++)
         ulDciPtr->payload[bytePos] = 0;

      bytePos = numBytes - 1;
      bitPos = 0;

      /* Packing DCI format fields */
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            dciFormatId, dciFormatIdSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            freqDomResAssign, freqDomResAssignSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            timeDomResAssign, timeDomResAssignSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            freqHopFlag, freqHopFlagSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            modNCodScheme, modNCodSchemeSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            ndi, ndiSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            redundancyVer, redundancyVerSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            harqProcessNum, harqProcessNumSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            puschTpc, puschTpcSize);
      fillDlDciPayload(ulDciPtr->payload, &bytePos, &bitPos,\
            ul_SlInd, ul_SlIndSize);
   }
} /* fillUlDciPdu */

/*******************************************************************
 *
 * @brief fills PDCCH PDU required for UL DCI REQ to PHY
 *
 * @details
 *
 *    Function : fillUlDciPdcchPdu
 *
 *    Functionality:
 *         -Fills the Pdcch PDU info
 *
 * @params[in] Pointer to FAPI DL TTI Req
 *             Pointer to PdcchCfg
 * @return ROK
 *
 ******************************************************************/
uint8_t fillUlDciPdcchPdu(fapi_dci_pdu_t *ulDciReqPdu, fapi_vendor_dci_pdu_t *vendorUlDciPdu, DlSchedInfo *dlInfo, uint8_t coreSetType)
{
   if(ulDciReqPdu != NULLP)
   {
      memset(&ulDciReqPdu->pdcchPduConfig, 0, sizeof(fapi_dl_pdcch_pdu_t));
      fillUlDciPdu(ulDciReqPdu->pdcchPduConfig.dlDci, dlInfo->ulGrant);
      ulDciReqPdu->pduType                          = PDCCH_PDU_TYPE;
      ulDciReqPdu->pdcchPduConfig.bwpSize           = dlInfo->ulGrant->bwpCfg.freqAlloc.numPrb;
      ulDciReqPdu->pdcchPduConfig.bwpStart          = dlInfo->ulGrant->bwpCfg.freqAlloc.startPrb;
      ulDciReqPdu->pdcchPduConfig.subCarrierSpacing = dlInfo->ulGrant->bwpCfg.subcarrierSpacing; 
      ulDciReqPdu->pdcchPduConfig.cyclicPrefix      = dlInfo->ulGrant->bwpCfg.cyclicPrefix; 
      ulDciReqPdu->pdcchPduConfig.startSymbolIndex  = dlInfo->ulGrant->coresetCfg.startSymbolIndex;
      ulDciReqPdu->pdcchPduConfig.durationSymbols   = dlInfo->ulGrant->coresetCfg.durationSymbols;
      memcpy(ulDciReqPdu->pdcchPduConfig.freqDomainResource, dlInfo->ulGrant->coresetCfg.freqDomainResource, 6);
      ulDciReqPdu->pdcchPduConfig.cceRegMappingType = dlInfo->ulGrant->coresetCfg.cceRegMappingType;
      ulDciReqPdu->pdcchPduConfig.regBundleSize     = dlInfo->ulGrant->coresetCfg.regBundleSize;
      ulDciReqPdu->pdcchPduConfig.interleaverSize   = dlInfo->ulGrant->coresetCfg.interleaverSize;
      ulDciReqPdu->pdcchPduConfig.shiftIndex        = dlInfo->ulGrant->coresetCfg.shiftIndex;
      ulDciReqPdu->pdcchPduConfig.precoderGranularity = dlInfo->ulGrant->coresetCfg.precoderGranularity;
      ulDciReqPdu->pdcchPduConfig.numDlDci          = 1;
      ulDciReqPdu->pdcchPduConfig.coreSetType       = coreSetType;

      /* Calculating PDU length. Considering only one Ul dci pdu for now */
      ulDciReqPdu->pduSize = sizeof(fapi_dl_pdcch_pdu_t);

      /* Vendor UL DCI PDU */
      vendorUlDciPdu->pdcch_pdu_config.num_dl_dci = ulDciReqPdu->pdcchPduConfig.numDlDci;
      vendorUlDciPdu->pdcch_pdu_config.dl_dci[0].epre_ratio_of_pdcch_to_ssb = 0;
      vendorUlDciPdu->pdcch_pdu_config.dl_dci[0].epre_ratio_of_dmrs_to_ssb = 0;
   }
   return ROK;
}
#endif
/*******************************************************************
 *
 * @brief Sends UL DCI Request to PHY
 *
 * @details
 *
 *    Function : fillUlDciReq
 *
 *    Functionality:
 *         -Sends FAPI Ul Dci req to PHY
 *
 * @params[in]  Pointer to CmLteTimingInfo
 * @return ROK     - success
 *         RFAILED - failure
 *
 ******************************************************************/
uint16_t fillUlDciReq(SlotTimingInfo currTimingInfo, p_fapi_api_queue_elem_t prevElem, fapi_vendor_ul_dci_req_t *vendorUlDciReq)
{
#ifdef INTEL_FAPI
   uint8_t      cellIdx =0;
   uint8_t      numPduEncoded = 0;
   SlotTimingInfo  ulDciReqTimingInfo ={0};
   MacDlSlot    *currDlSlot = NULLP;
   fapi_ul_dci_req_t        *ulDciReq =NULLP;
   p_fapi_api_queue_elem_t  ulDciElem;

   if(lwrMacCb.phyState == PHY_STATE_RUNNING)
   {
      GET_CELL_IDX(currTimingInfo.cellId, cellIdx);
      memcpy(&ulDciReqTimingInfo, &currTimingInfo, sizeof(SlotTimingInfo));
      currDlSlot = &macCb.macCell[cellIdx]->dlSlot[ulDciReqTimingInfo.slot % macCb.macCell[cellIdx]->numOfSlots];

         LWR_MAC_ALLOC(ulDciElem, (sizeof(fapi_api_queue_elem_t) + sizeof(fapi_ul_dci_req_t)));
         if(ulDciElem)
	 {
	    FILL_FAPI_LIST_ELEM(ulDciElem, NULLP, FAPI_UL_DCI_REQUEST, 1, \
	       sizeof(fapi_ul_dci_req_t));
	    ulDciReq = (fapi_ul_dci_req_t *)(ulDciElem +1);
	    memset(ulDciReq, 0, sizeof(fapi_ul_dci_req_t));
	    fillMsgHeader(&ulDciReq->header, FAPI_UL_DCI_REQUEST, sizeof(fapi_ul_dci_req_t));

            ulDciReq->sfn  = ulDciReqTimingInfo.sfn;
            ulDciReq->slot = ulDciReqTimingInfo.slot;
          if(currDlSlot->dlInfo.ulGrant != NULLP)
          {
            vendorUlDciReq->sym = 0;
            ulDciReq->numPdus = 1;  // No. of PDCCH PDUs
            vendorUlDciReq->num_pdus = ulDciReq->numPdus;
            if(ulDciReq->numPdus > 0)
            {
               /* Fill PDCCH configuration Pdu */
               fillUlDciPdcchPdu(&ulDciReq->pdus[numPduEncoded], &vendorUlDciReq->pdus[numPduEncoded], &currDlSlot->dlInfo, CORESET_TYPE1);
               numPduEncoded++;
	       /* free UL GRANT at SCH */
	       MAC_FREE(currDlSlot->dlInfo.ulGrant, sizeof(DciInfo));
            }
#ifdef ODU_SLOT_IND_DEBUG_LOG
	       DU_LOG("\nDEBUG  -->  LWR_MAC: Sending UL DCI Request");
#endif
	 }
               prevElem->p_next = ulDciElem;
      }
   }
   else
   {
       lwr_mac_procInvalidEvt(&currTimingInfo);
   }
#endif
   return ROK;
}

#ifndef NFAPI
lwrMacFsmHdlr fapiEvtHdlr[MAX_STATE][MAX_EVENT] =
{
   {
      /* PHY_STATE_IDLE */
#ifdef INTEL_TIMER_MODE 
      lwr_mac_procIqSamplesReqEvt,
#endif
      lwr_mac_procParamReqEvt,
      lwr_mac_procParamRspEvt,
      lwr_mac_procConfigReqEvt,
      lwr_mac_procConfigRspEvt,
      lwr_mac_procInvalidEvt,
      lwr_mac_procInvalidEvt,
   },
   {
      /* PHY_STATE_CONFIGURED */
#ifdef INTEL_TIMER_MODE
      lwr_mac_procInvalidEvt,
#endif
      lwr_mac_procParamReqEvt,
      lwr_mac_procParamRspEvt,
      lwr_mac_procConfigReqEvt,
      lwr_mac_procConfigRspEvt,
      lwr_mac_procStartReqEvt,
      lwr_mac_procInvalidEvt,
   },
   {
      /* PHY_STATE_RUNNING */
#ifdef INTEL_TIMER_MODE
      lwr_mac_procInvalidEvt,
#endif
      lwr_mac_procInvalidEvt,
      lwr_mac_procInvalidEvt,
      lwr_mac_procConfigReqEvt,
      lwr_mac_procConfigRspEvt,
      lwr_mac_procInvalidEvt,
      lwr_mac_procInvalidEvt,
   }
};
#else // define NFAPI
/*******************************************************************
 *
 * @brief Start the VNF: sctp socket and wait for PNF connection
 *
 * @details
 *
 *    Function : lwr_mac_procVnfCfgStartEvt
 *
 *    Functionality:
 *         -Start the VNF Event
 *
 * @params[in]  Pointer to nfapi_vnf_config_t
 * @return ROK     - success
 *         RFAILED - failure
 *
 ******************************************************************/
uint8_t intgr_lwr_mac_procVnfCfgStartEvt(void *msg)
{
   DU_LOG("\nINFO  --> LWR_MAC [NFAPI_TRACE_INFO] : lwr_mac_procVnfCfgStartEvt");

   vnf_cfg_t *msg_p5_p7_cfg = (vnf_cfg_t *)msg;
   glb_config = msg_p5_p7_cfg->config;
   glb_vnf = msg_p5_p7_cfg->vnf;

   if (!glb_config)
   {
      DU_LOG("\nERROR  --> LWR_MAC [NFAPI_TRACE_ERROR] : VNF Start Event with wrong message");
      return RFAILED;
   }

   pthread_t vnf_start_thread;
   pthread_create(&vnf_start_thread, NULL, &nfapi_nr_vnf_start, glb_config);

   return ROK;
}

uint8_t intgr_lwr_mac_procConfigRspEvt(uint32_t msgLen, void *msg)
{
   DU_LOG("\nINFO  -->  Func : %s \n", __FUNCTION__);
   sendToLowerMac(CONFIG_RESPONSE, msgLen, msg);
   return 0;
}

uint8_t intgr_lwr_mac_procStartRspEvt(uint32_t msgLen, void *msg)
{
   return 0;
}

uint8_t intgr_lwr_mac_procPhyStartEvt(void *msg)
{
   return ROK;
}

uint8_t intgr_lwr_mac_procPNFParamReqEvt(void *msg)
{
   return ROK;
}

uint8_t intgr_lwr_mac_procPNFParamRspEvt(void *msg)
{
   return ROK;
}

uint8_t intgr_lwr_mac_procPNFConfigReqEvt(void *msg)
{
   return ROK;
}

uint8_t intgr_lwr_mac_procPNFConfigRspEvt(void *msg)
{
   return ROK;
}

uint8_t intgr_lwr_mac_procPNFStartReqEvt(void *msg)
{
   return ROK;
}

uint8_t intgr_lwr_mac_procPNFStartRspEvt(void *msg)
{
   return ROK;
}

lwrMacFsmHdlr nfapiEvtHdlr[PNF_MAX_STATE][MAX_EVENT] = {
    {
/* PNF_STATE_IDLE */
#ifdef INTEL_TIMER_MODE
        lwr_mac_procIqSamplesReqEvt,
#endif
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procConfigReqEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        intgr_lwr_mac_procVnfCfgStartEvt,
        intgr_lwr_mac_procPNFParamReqEvt,
        intgr_lwr_mac_procPNFParamRspEvt,
        intgr_lwr_mac_procPNFConfigReqEvt,
        intgr_lwr_mac_procPNFConfigRspEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
    },
    {
/* PNF_STATE_CONFIGURED */
#ifdef INTEL_TIMER_MODE
        lwr_mac_procIqSamplesReqEvt,
#endif
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procConfigReqEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        intgr_lwr_mac_procPNFConfigReqEvt, //  oai PNFParamReq // done
        intgr_lwr_mac_procPNFConfigRspEvt, //  oai PNFConfigRsp
        intgr_lwr_mac_procPNFStartReqEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
    },
    {
/* PNF_STATE_RUNNING */
#ifdef INTEL_TIMER_MODE
        lwr_mac_procIqSamplesReqEvt,
#endif
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procConfigReqEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        lwr_mac_procInvalidEvt,
        intgr_lwr_mac_procPNFStartRspEvt,
    },
};

lwrMacFsmHdlr fapiEvtHdlr[MAX_STATE][MAX_EVENT] =
    {
        {
/* PHY_STATE_IDLE */
#ifdef INTEL_TIMER_MODE
            lwr_mac_procIqSamplesReqEvt,
#endif
            lwr_mac_procParamReqEvt,
            lwr_mac_procParamRspEvt,
            lwr_mac_procConfigReqEvt,
            lwr_mac_procConfigRspEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
        },
        {
/* PHY_STATE_CONFIGURED */
#ifdef INTEL_TIMER_MODE
            lwr_mac_procInvalidEvt,
#endif
            lwr_mac_procParamReqEvt,
            lwr_mac_procParamRspEvt,
            lwr_mac_procConfigReqEvt,
            lwr_mac_procConfigRspEvt,
            lwr_mac_procStartReqEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
        },
        {
/* PHY_STATE_RUNNING */
#ifdef INTEL_TIMER_MODE
            lwr_mac_procInvalidEvt,
#endif
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procConfigReqEvt,
            lwr_mac_procConfigRspEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
            lwr_mac_procInvalidEvt,
        },
};
#endif // NFAPI


/*******************************************************************
 *
 * @brief Sends message to LWR_MAC Fsm Event Handler
 *
 * @details
 *
 *    Function : sendToLowerMac
 *
 *    Functionality:
 *         -Sends message to LowerMac
 *
 * @params[in] Message Type
 *             Message Length
 *             Messaga Pointer
 *
 * @return void
 *
 ******************************************************************/
void sendToLowerMac(uint16_t msgType, uint32_t msgLen, void *msg)
{
   lwrMacCb.event = msgType;

   DU_LOG("\nINFO  -->  LWR_MAC : phyState : %2d in msgType : %d", lwrMacCb.phyState, msgType);

#ifdef NFAPI
   DU_LOG("\nINFO  -->  NFAPI : pnfState : %2d", lwrMacCb.pnfState);
   if(!pnf_state_lock->flag)
      nfapiEvtHdlr[lwrMacCb.pnfState][lwrMacCb.event](msg);
   else
#endif
   {
      fapiEvtHdlr[lwrMacCb.phyState][lwrMacCb.event](msg);
   }
}

/**********************************************************************
  End of file
 **********************************************************************/
