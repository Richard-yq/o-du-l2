/*
 * Copyright 2022 MWNl NCU.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _VNF_INTGR_MWNL_H_
#define _VNF_INTGR_MWNL_H_

#include <stdint.h>
#include "nfapi_nr_interface_scf.h"

#define ORAN_OAI_FAPI_MAX_NUM_TLVS_CONFIG 1500

typedef struct
{
        uint16_t tag;
        uint16_t length; // 5G FAPI Table 3-7 Fixed part
} ORAN_OAI_fapi_tl_t;

typedef struct
{
        uint16_t msg_id;
        uint16_t pad;
        uint32_t length; // Length of the message body in bytes  5G FAPI Table 3-3
} ORAN_OAI_fapi_msg_t;

typedef struct
{
        ORAN_OAI_fapi_tl_t tl;
        uint32_t value; // TLV with unsigned 32 bit value
} ORAN_OAI_fapi_uint32_tlv_t;

typedef struct
{
        ORAN_OAI_fapi_msg_t header;
        uint8_t number_of_tlvs;
        uint8_t pad[3];
        ORAN_OAI_fapi_uint32_tlv_t tlvs[ORAN_OAI_FAPI_MAX_NUM_TLVS_CONFIG]; // 5G FAPI Table 3-17
} ORAN_OAI_fapi_config_req_t;

/* CONFIG.RESPONSE */
typedef struct
{
        nfapi_p4_p5_message_header_t header;
        uint8_t error_code;
        // uint8_t num_invalid_tlvs;
        //  TODO: add list of invalid/unsupported TLVs (see Table 3.18)
        nfapi_vendor_extension_tlv_t vendor_extension;
} ORAN_OAI_nfapi_nr_config_response_scf_t;

// Updated per 5G FAPI
typedef struct
{
        // fapi_msg_t header;
        // uint8_t error_code;
        // uint8_t number_of_invalid_tlvs;
        // uint8_t number_of_inv_tlvs_idle_only;
        // uint8_t number_of_inv_tlvs_running_only;
        // uint8_t number_of_missing_tlvs;
        // uint8_t pad[3];
        // fapi_uint16_tlv_t tlvs[4 * FAPI_MAX_NUM_TLVS_CONFIG];   // 5G FAPI Table 3-18
} ORAN_OAI_fapi_config_resp_t;

// Define in vnf.c
extern nfapi_nr_param_response_scf_t *intgr_resp;
extern int intgr_p5_idx;

// Define in lwr_mac_fsm.c
extern ORAN_OAI_fapi_config_req_t *intgr_fapi_config;

#endif