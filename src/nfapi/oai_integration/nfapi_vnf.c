/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "nfapi_nr_interface_scf.h"
#include "nfapi_vnf_interface.h"
#include "nfapi_vnf.h"
#include "nfapi.h"
#include "vendor_ext.h"

#include "queue_t.h"
#include "gnb_ind_vars.h"

/* ======== small cell integration ======== */
#include "type_def.h"
#ifdef NFAPI
#include "du_log.h"

#define _Assert_Exit_                           \
    fprintf(stderr, "\nExiting execution\n");   \
    fflush(stdout);                             \
    fflush(stderr);                             \
    abort();

#define _Assert_(cOND, aCTION, fORMAT, aRGS...)             \
do {                                                        \
    if (!(cOND)) {                                          \
        fprintf(stderr, "\nAssertion ("#cOND") failed!\n"   \
                "In %s() %s:%d\n" fORMAT,                   \
                __FUNCTION__, __FILE__, __LINE__, ##aRGS);  \
        aCTION;                                             \
    }						\
} while(0)

#define AssertFatal(cOND, fORMAT, aRGS...)          _Assert_(cOND, _Assert_Exit_, fORMAT, ##aRGS)
#endif //NFAPI
/* ========================================== */


#define TEST

int vnf_pack_vendor_extension_tlv(void *ve, uint8_t **ppWritePackedMsg, uint8_t *end, nfapi_p4_p5_codec_config_t *codec) {
  nfapi_tl_t *tlv = (nfapi_tl_t *)ve;
  switch(tlv->tag) {
    case VENDOR_EXT_TLV_2_TAG: {
      vendor_ext_tlv_2 *ve = (vendor_ext_tlv_2 *)tlv;

      if(!push32(ve->dummy, ppWritePackedMsg, end))
        return 0;

      return 1;
    }
    break;
  }
  return -1;
}

int vnf_unpack_vendor_extension_tlv(nfapi_tl_t *tl, uint8_t **ppReadPackedMessage, uint8_t *end, void **ve, nfapi_p4_p5_codec_config_t *codec) {
  return -1;
}

int pnf_connection_indication_cb(nfapi_vnf_config_t *config, int p5_idx) {
  DU_LOG("\nINFO   --> [VNF] pnf connection indication idx:%d\n", p5_idx);
  nfapi_pnf_param_request_t req;
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_PNF_PARAM_REQUEST;
  nfapi_vnf_pnf_param_req(config, p5_idx, &req);
  return 0;
}

int pnf_nr_connection_indication_cb(nfapi_vnf_config_t *config, int p5_idx) {
  DU_LOG("\nINFO   --> [VNF] pnf connection indication idx:%d\n", p5_idx);
  nfapi_nr_pnf_param_request_t req;
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_NR_PHY_MSG_TYPE_PNF_PARAM_REQUEST;
  nfapi_nr_vnf_pnf_param_req(config, p5_idx, &req);
  return 0;
}

int pnf_disconnection_indication_cb(nfapi_vnf_config_t *config, int p5_idx) {
  DU_LOG("\nINFO   --> [VNF] pnf disconnection indication idx:%d\n", p5_idx);
  vnf_info *vnf = (vnf_info *)(config->user_data);
  pnf_info *pnf = vnf->pnfs;
  phy_info *phy = pnf->phys;
  vnf_p7_info *p7_vnf = vnf->p7_vnfs;
  nfapi_vnf_p7_del_pnf((p7_vnf->config), phy->id);
  return 0;
}

int pnf_nr_param_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_pnf_param_response_t *resp) {
  DU_LOG("\nINFO   --> [VNF] pnf param response idx:%d error:%d\n", p5_idx, resp->error_code);
  vnf_info *vnf = (vnf_info *)(config->user_data);
  pnf_info *pnf = vnf->pnfs;

  for(int i = 0; i < resp->pnf_phy.number_of_phys; ++i) {
    phy_info phy;
    memset(&phy,0,sizeof(phy));
    phy.index = resp->pnf_phy.phy[i].phy_config_index;
    DU_LOG("\nINFO   --> [VNF] (PHY:%d) phy_config_idx:%d\n", i, resp->pnf_phy.phy[i].phy_config_index);
    nfapi_vnf_allocate_phy(config, p5_idx, &(phy.id));

    for(int j = 0; j < resp->pnf_phy.phy[i].number_of_rfs; ++j) {
      DU_LOG("\nINFO   --> [VNF] (PHY:%d) (RF%d) %d\n", i, j, resp->pnf_phy.phy[i].rf_config[j].rf_config_index);
      phy.rfs[0] = resp->pnf_phy.phy[i].rf_config[j].rf_config_index;
    }

    pnf->phys[0] = phy;
  }
  nfapi_nr_pnf_config_request_t req;
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_PNF_CONFIG_REQUEST;
  req.pnf_phy_rf_config.tl.tag = NFAPI_PNF_PHY_RF_TAG;
  req.pnf_phy_rf_config.number_phy_rf_config_info = 2; // pnf.phys.size();
  DU_LOG("\nINFO   --> Hard coded num phy rf to 2\n");

  for(unsigned i = 0; i < 2; ++i) {
    req.pnf_phy_rf_config.phy_rf_config[i].phy_id = pnf->phys[i].id;
    req.pnf_phy_rf_config.phy_rf_config[i].phy_config_index = pnf->phys[i].index;
    req.pnf_phy_rf_config.phy_rf_config[i].rf_config_index = pnf->phys[i].rfs[0];
  }

  nfapi_nr_vnf_pnf_config_req(config, p5_idx, &req);
  return 0;
}

int pnf_param_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_pnf_param_response_t *resp) {
  DU_LOG("\nINFO   --> [VNF] pnf param response idx:%d error:%d\n", p5_idx, resp->error_code);
  vnf_info *vnf = (vnf_info *)(config->user_data);
  pnf_info *pnf = vnf->pnfs;

  for(int i = 0; i < resp->pnf_phy.number_of_phys; ++i) {
    phy_info phy;
    memset(&phy,0,sizeof(phy));
    phy.index = resp->pnf_phy.phy[i].phy_config_index;
    DU_LOG("\nINFO   --> [VNF] (PHY:%d) phy_config_idx:%d\n", i, resp->pnf_phy.phy[i].phy_config_index);
    nfapi_vnf_allocate_phy(config, p5_idx, &(phy.id));

    for(int j = 0; j < resp->pnf_phy.phy[i].number_of_rfs; ++j) {
      DU_LOG("\nINFO   --> [VNF] (PHY:%d) (RF%d) %d\n", i, j, resp->pnf_phy.phy[i].rf_config[j].rf_config_index);
      phy.rfs[0] = resp->pnf_phy.phy[i].rf_config[j].rf_config_index;
    }

    pnf->phys[0] = phy;
  }
  for(int i = 0; i < resp->pnf_rf.number_of_rfs; ++i) {
    rf_info rf;
    memset(&rf,0,sizeof(rf));
    rf.index = resp->pnf_rf.rf[i].rf_config_index;
    DU_LOG("\nINFO   --> [VNF] (RF:%d) rf_config_idx:%d\n", i, resp->pnf_rf.rf[i].rf_config_index);
    pnf->rfs[0] = rf;
  }
  nfapi_pnf_config_request_t req;
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_PNF_CONFIG_REQUEST;
  req.pnf_phy_rf_config.tl.tag = NFAPI_PNF_PHY_RF_TAG;
  req.pnf_phy_rf_config.number_phy_rf_config_info = 2; // pnf.phys.size();
  DU_LOG("\nINFO   --> Hard coded num phy rf to 2\n");

  for(unsigned i = 0; i < 2; ++i) {
    req.pnf_phy_rf_config.phy_rf_config[i].phy_id = pnf->phys[i].id;
    req.pnf_phy_rf_config.phy_rf_config[i].phy_config_index = pnf->phys[i].index;
    req.pnf_phy_rf_config.phy_rf_config[i].rf_config_index = pnf->phys[i].rfs[0];
  }

  nfapi_vnf_pnf_config_req(config, p5_idx, &req);
  return 0;
}

int pnf_nr_config_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_pnf_config_response_t *resp) {
  DU_LOG("\nINFO   --> [VNF] pnf config response idx:%d resp[header[phy_id:%u message_id:%02x message_length:%u]]\n", p5_idx, resp->header.phy_id, resp->header.message_id, resp->header.message_length);

  if(1) {
    nfapi_nr_pnf_start_request_t req;
    memset(&req, 0, sizeof(req));
    req.header.phy_id = resp->header.phy_id;
    req.header.message_id = NFAPI_PNF_START_REQUEST;
    nfapi_nr_vnf_pnf_start_req(config, p5_idx, &req);
  } else {
    vendor_ext_p5_req req;
    memset(&req, 0, sizeof(req));
    req.header.message_id = P5_VENDOR_EXT_REQ;
    req.dummy1 = 45;
    req.dummy2 = 1977;
    nfapi_vnf_vendor_extension(config, p5_idx, &req.header);
  }

  return 0;
}

int pnf_config_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_pnf_config_response_t *resp) {
  DU_LOG("\nINFO   --> [VNF] pnf config response idx:%d resp[header[phy_id:%u message_id:%02x message_length:%u]]\n", p5_idx, resp->header.phy_id, resp->header.message_id, resp->header.message_length);

  if(1) {
    nfapi_pnf_start_request_t req;
    memset(&req, 0, sizeof(req));
    req.header.phy_id = resp->header.phy_id;
    req.header.message_id = NFAPI_PNF_START_REQUEST;
    nfapi_vnf_pnf_start_req(config, p5_idx, &req);
  } else {
    vendor_ext_p5_req req;
    memset(&req, 0, sizeof(req));
    req.header.message_id = P5_VENDOR_EXT_REQ;
    req.dummy1 = 45;
    req.dummy2 = 1977;
    nfapi_vnf_vendor_extension(config, p5_idx, &req.header);
  }

  return 0;
}

int phy_sync_indication(struct nfapi_vnf_p7_config *config, uint8_t sync) {
  return(0);
}


int phy_slot_indication(struct nfapi_vnf_p7_config *config, uint16_t phy_id, uint16_t sfn, uint16_t slot) {
  static uint8_t first_time = 1;

  if (first_time) {
    DU_LOG("\nINFO   --> [VNF] slot indication %d\n", NFAPI_SFNSLOT2DEC(sfn, slot));
    first_time = 0;
  }
  return 0;
}

int phy_subframe_indication(struct nfapi_vnf_p7_config *config, uint16_t phy_id, uint16_t sfn_sf) {
  static uint8_t first_time = 1;

  if (first_time) {
    DU_LOG("\nINFO   --> [VNF] subframe indication %d\n", NFAPI_SFNSF2DEC(sfn_sf));
    first_time = 0;
  }
  return 0;
}

int phy_rach_indication(struct nfapi_vnf_p7_config *config, nfapi_rach_indication_t *ind) {
  DU_LOG("\nINFO   --> %s() NFAPI SFN/SF:%d number_of_preambles:%u\n", __FUNCTION__, NFAPI_SFNSF2DEC(ind->sfn_sf), ind->rach_indication_body.number_of_preambles);
  return 1;
}

int phy_nr_rach_indication(nfapi_nr_rach_indication_t *ind)
{ 
  /* ======== small cell integration ======= */
  #ifdef NFAPI
  if(1)
  #else
  if(NFAPI_MODE == NFAPI_MODE_VNF)
  #endif
  /* ======================================= */
  {
    nfapi_nr_rach_indication_t *rach_ind = calloc(1, sizeof(*rach_ind));
    rach_ind->header.message_id = ind->header.message_id;
    rach_ind->number_of_pdus = ind->number_of_pdus;
    rach_ind->sfn = ind->sfn;
    rach_ind->slot = ind->slot;
    rach_ind->pdu_list = calloc(rach_ind->number_of_pdus, sizeof(*rach_ind->pdu_list));
    AssertFatal(rach_ind->pdu_list != NULL, "Memory not allocated for rach_ind->pdu_list in phy_nr_rach_indication.");
    for (int i = 0; i < ind->number_of_pdus; i++)
    {
      rach_ind->pdu_list[i].num_preamble = ind->pdu_list[i].num_preamble;
      rach_ind->pdu_list[i].freq_index = ind->pdu_list[i].freq_index;
      rach_ind->pdu_list[i].symbol_index = ind->pdu_list[i].symbol_index;
      rach_ind->pdu_list[i].preamble_list = calloc(ind->pdu_list[i].num_preamble, sizeof(nfapi_nr_prach_indication_preamble_t));
      AssertFatal(rach_ind->pdu_list[i].preamble_list != NULL, "Memory not allocated for rach_ind->pdu_list[i].preamble_list  in phy_nr_rach_indication.");
      for (int j = 0; j < ind->number_of_pdus; j++)
      {
        rach_ind->pdu_list[i].preamble_list[j].preamble_index = ind->pdu_list[i].preamble_list[j].preamble_index;
        rach_ind->pdu_list[i].preamble_list[j].timing_advance = ind->pdu_list[i].preamble_list[j].timing_advance;
      }
    }
    if (!put_queue(&gnb_rach_ind_queue, rach_ind))
    {
      for (int i = 0; i < ind->number_of_pdus; i++)
      {
        free(rach_ind->pdu_list[i].preamble_list);
      }
      free(rach_ind->pdu_list);
      free(rach_ind);
    }
  }
  else {
    DU_LOG("\nINFO   --> [VNF] not NFAPI_MODE_VNF(2)\n");
  }
  return 1;
}

int phy_nr_uci_indication(nfapi_nr_uci_indication_t *ind)
{
  DU_LOG("\nINFO   --> [VNF] In %s() NFAPI SFN/SF: %d/%d number_of_pdus :%u\n",
          __FUNCTION__,ind->sfn, ind->slot, ind->num_ucis);
  /* ======== small cell integration ======= */
  #ifdef NFAPI
  if(1)
  #else
  if(NFAPI_MODE == NFAPI_MODE_VNF)
  #endif
  /* ======================================= */
  {
    nfapi_nr_uci_indication_t *uci_ind = calloc(1, sizeof(*uci_ind));
    AssertFatal(uci_ind != NULL, "Memory not allocated for uci_ind in phy_nr_uci_indication.");
    *uci_ind = *ind;

    uci_ind->uci_list = calloc(NFAPI_NR_UCI_IND_MAX_PDU, sizeof(nfapi_nr_uci_t));
    AssertFatal(uci_ind->uci_list != NULL, "Memory not allocated for uci_ind->uci_list in phy_nr_uci_indication.");
    for (int i = 0; i < ind->num_ucis; i++)
    {
      uci_ind->uci_list[i] = ind->uci_list[i];

      switch (uci_ind->uci_list[i].pdu_type) {
        case NFAPI_NR_UCI_PUSCH_PDU_TYPE:
          DU_LOG("\nINFO   --> [VNF] %s(): unhandled NFAPI_NR_UCI_PUSCH_PDU_TYPE\n", __func__);
          break;

        case NFAPI_NR_UCI_FORMAT_0_1_PDU_TYPE: {
          break;
        }

        case NFAPI_NR_UCI_FORMAT_2_3_4_PDU_TYPE: {
          nfapi_nr_uci_pucch_pdu_format_2_3_4_t *uci_ind_pdu = &uci_ind->uci_list[i].pucch_pdu_format_2_3_4;
          nfapi_nr_uci_pucch_pdu_format_2_3_4_t *ind_pdu = &ind->uci_list[i].pucch_pdu_format_2_3_4;
          *uci_ind_pdu = *ind_pdu;
          if (ind_pdu->harq.harq_payload) {
            uci_ind_pdu->harq.harq_payload = calloc(1, sizeof(*uci_ind_pdu->harq.harq_payload));
            AssertFatal(uci_ind_pdu->harq.harq_payload != NULL, "Memory not allocated for uci_ind_pdu->harq.harq_payload in phy_nr_uci_indication.");
            *uci_ind_pdu->harq.harq_payload = *ind_pdu->harq.harq_payload;
          }
          if (ind_pdu->sr.sr_payload) {
            uci_ind_pdu->sr.sr_payload = calloc(1, sizeof(*uci_ind_pdu->sr.sr_payload));
            AssertFatal(uci_ind_pdu->sr.sr_payload != NULL, "Memory not allocated for uci_ind_pdu->sr.sr_payload in phy_nr_uci_indication.");
            *uci_ind_pdu->sr.sr_payload = *ind_pdu->sr.sr_payload;
          }
          if (ind_pdu->csi_part1.csi_part1_payload) {
            uci_ind_pdu->csi_part1.csi_part1_payload = calloc(1, sizeof(*uci_ind_pdu->csi_part1.csi_part1_payload));
            AssertFatal(uci_ind_pdu->csi_part1.csi_part1_payload != NULL, "Memory not allocated for uci_ind_pdu->csi_part1.csi_part1_payload in phy_nr_uci_indication.");
            *uci_ind_pdu->csi_part1.csi_part1_payload = *ind_pdu->csi_part1.csi_part1_payload;
          }
          if (ind_pdu->csi_part2.csi_part2_payload) {
            uci_ind_pdu->csi_part2.csi_part2_payload = calloc(1, sizeof(*uci_ind_pdu->csi_part2.csi_part2_payload));
            AssertFatal(uci_ind_pdu->csi_part2.csi_part2_payload != NULL, "Memory not allocated for uci_ind_pdu->csi_part2.csi_part2_payload in phy_nr_uci_indication.");
            *uci_ind_pdu->csi_part2.csi_part2_payload = *ind_pdu->csi_part2.csi_part2_payload;
          }
          break;
        }
      }
    }

    if (!put_queue(&gnb_uci_ind_queue, uci_ind))
    {
      DU_LOG("\nINFO   --> [VNF] Put_queue failed for uci_ind\n");
      for (int i = 0; i < ind->num_ucis; i++)
      {
          if (uci_ind->uci_list[i].pdu_type == NFAPI_NR_UCI_FORMAT_0_1_PDU_TYPE)
          {
          }
          if (uci_ind->uci_list[i].pdu_type == NFAPI_NR_UCI_FORMAT_2_3_4_PDU_TYPE)
          {
            free(uci_ind->uci_list[i].pucch_pdu_format_2_3_4.harq.harq_payload);
            free(uci_ind->uci_list[i].pucch_pdu_format_2_3_4.csi_part1.csi_part1_payload);
            free(uci_ind->uci_list[i].pucch_pdu_format_2_3_4.csi_part2.csi_part2_payload);
          }
      }
      free(uci_ind->uci_list);
      uci_ind->uci_list = NULL;
      free(uci_ind);
      uci_ind = NULL;
    }
  }
  else {
    DU_LOG("\nINFO   --> [VNF] not NFAPI_MODE_VNF(2)\n");
  }
  return 1;
}

int phy_harq_indication(struct nfapi_vnf_p7_config *config, nfapi_harq_indication_t *ind) {
  return 1;
}

int phy_crc_indication(struct nfapi_vnf_p7_config *config, nfapi_crc_indication_t *ind) {
  return 1;
}

int phy_nr_crc_indication(nfapi_nr_crc_indication_t *ind) {

  DU_LOG("\nINFO   --> [VNF] In %s() NFAPI SFN/SF: %d/%d number_of_pdus :%u\n",
          __FUNCTION__,ind->sfn, ind->slot, ind->number_crcs);

  /* ======== small cell integration ======= */
  #ifdef NFAPI
  if(1)
  #else
  if(NFAPI_MODE == NFAPI_MODE_VNF)
  #endif
  /* ======================================= */
  {
    nfapi_nr_crc_indication_t *crc_ind = calloc(1, sizeof(*crc_ind));
    crc_ind->header.message_id = ind->header.message_id;
    crc_ind->number_crcs = ind->number_crcs;
    crc_ind->sfn = ind->sfn;
    crc_ind->slot = ind->slot;
    if (ind->number_crcs > 0) {
      crc_ind->crc_list = calloc(NFAPI_NR_CRC_IND_MAX_PDU, sizeof(nfapi_nr_crc_t));
      AssertFatal(crc_ind->crc_list != NULL, "Memory not allocated for crc_ind->crc_list in phy_nr_crc_indication.");
    }
    for (int j = 0; j < ind->number_crcs; j++)
    {
      crc_ind->crc_list[j].handle = ind->crc_list[j].handle;
      crc_ind->crc_list[j].harq_id = ind->crc_list[j].harq_id;
      crc_ind->crc_list[j].num_cb = ind->crc_list[j].num_cb;
      crc_ind->crc_list[j].rnti = ind->crc_list[j].rnti;
      crc_ind->crc_list[j].tb_crc_status = ind->crc_list[j].tb_crc_status;
      crc_ind->crc_list[j].timing_advance = ind->crc_list[j].timing_advance;
      crc_ind->crc_list[j].ul_cqi = ind->crc_list[j].ul_cqi;
      DU_LOG("\nINFO   --> [VNF] Received crc_ind.harq_id = %d for %d index SFN SLot %u %u with rnti %x\n",
                    ind->crc_list[j].harq_id, j, ind->sfn, ind->slot, ind->crc_list[j].rnti);
    }
    if (!put_queue(&gnb_crc_ind_queue, crc_ind))
    {
      DU_LOG("\nINFO   --> [VNF] Put_queue failed for crc_ind\n");
      free(crc_ind->crc_list);
      free(crc_ind);
    }
  }
  else
  {
    DU_LOG("\nINFO   --> [VNF] NFAPI_MODE = %d not NFAPI_MODE_VNF(2)\n", nfapi_getmode());
  }
  return 1;
}

int phy_rx_indication(struct nfapi_vnf_p7_config *config, nfapi_rx_indication_t *ind) {
  return 1;
}

int phy_nr_rx_data_indication(nfapi_nr_rx_data_indication_t *ind) {

  DU_LOG("\nINFO   --> [VNF] In %s() NFAPI SFN/SF: %d/%d number_of_pdus :%u, and pdu %p\n",
          __FUNCTION__,ind->sfn, ind->slot, ind->number_of_pdus, ind->pdu_list[0].pdu);

  /* ======== small cell integration ======= */
  #ifdef NFAPI
  if(1)
  #else
  if(NFAPI_MODE == NFAPI_MODE_VNF)
  #endif
  /* ======================================= */
  {
    nfapi_nr_rx_data_indication_t *rx_ind = calloc(1, sizeof(*rx_ind));
    rx_ind->header.message_id = ind->header.message_id;
    rx_ind->sfn = ind->sfn;
    rx_ind->slot = ind->slot;
    rx_ind->number_of_pdus = ind->number_of_pdus;

    if (ind->number_of_pdus > 0) {
      rx_ind->pdu_list = calloc(NFAPI_NR_RX_DATA_IND_MAX_PDU, sizeof(nfapi_nr_rx_data_pdu_t));
      AssertFatal(rx_ind->pdu_list != NULL, "Memory not allocated for rx_ind->pdu_list in phy_nr_rx_data_indication.");
    }
    for (int j = 0; j < ind->number_of_pdus; j++)
    {
      rx_ind->pdu_list[j].handle = ind->pdu_list[j].handle;
      rx_ind->pdu_list[j].harq_id = ind->pdu_list[j].harq_id;
      rx_ind->pdu_list[j].pdu = ind->pdu_list[j].pdu;
      rx_ind->pdu_list[j].pdu_length = ind->pdu_list[j].pdu_length;
      rx_ind->pdu_list[j].rnti = ind->pdu_list[j].rnti;
      rx_ind->pdu_list[j].timing_advance = ind->pdu_list[j].timing_advance;
      rx_ind->pdu_list[j].ul_cqi = ind->pdu_list[j].ul_cqi;
      rx_ind->pdu_list[j].rssi = ind->pdu_list[j].rssi;
    }
    if (!put_queue(&gnb_rx_ind_queue, rx_ind))
    {
      DU_LOG("\nINFO   --> [VNF] Put_queue failed for rx_ind\n");
      free(rx_ind->pdu_list);
      free(rx_ind);
    }
  }
  else
  {
    DU_LOG("\nINFO   --> [VNF] NFAPI_MODE = %d not NFAPI_MODE_VNF(2)\n", nfapi_getmode());
  }
  return 1;
}

int phy_srs_indication(struct nfapi_vnf_p7_config *config, nfapi_srs_indication_t *ind) {
  // vnf_p7_info* p7_vnf = (vnf_p7_info*)(config->user_data);
  //mac_srs_ind(p7_vnf->mac, ind);
  return 1;
}

int phy_sr_indication(struct nfapi_vnf_p7_config *config, nfapi_sr_indication_t *ind) {
  return 1;
}

static bool is_ue_same(uint16_t ue_id_1, uint16_t ue_id_2)
{
  return (ue_id_1 == ue_id_2);
}

static void analyze_cqi_pdus_for_duplicates(nfapi_cqi_indication_t *ind)
{
  uint16_t num_cqis = ind->cqi_indication_body.number_of_cqis;
  // assert(num_cqis <= NFAPI_CQI_IND_MAX_PDU); NFAPI
  for (int i = 0; i < num_cqis; i++)
  {
    nfapi_cqi_indication_pdu_t *src_pdu = &ind->cqi_indication_body.cqi_pdu_list[i];

    DU_LOG("\nINFO   --> CQI_IND[PDU:%d][rnti:%x cqi:%d channel:%d]\n", i, src_pdu->rx_ue_information.rnti,
          src_pdu->ul_cqi_information.ul_cqi, src_pdu->ul_cqi_information.channel);

    for (int j = i + 1; j < num_cqis; j++)
    {
      uint16_t rnti_i = ind->cqi_indication_body.cqi_pdu_list[i].rx_ue_information.rnti;
      uint16_t rnti_j = ind->cqi_indication_body.cqi_pdu_list[j].rx_ue_information.rnti;
      if (is_ue_same(rnti_i, rnti_j))
      {
        DU_LOG("\nINFO   --> [VNF] Problem, two cqis received from a single UE for rnti %x\n",
              rnti_i);
        //abort(); This will be fixed in merge request which handles multiple CQIs.
      }
    }
  }
}

int phy_cqi_indication(struct nfapi_vnf_p7_config *config, nfapi_cqi_indication_t *ind) {
  return 1;
}

//NR phy indication

/* ======== small cell integration ======== */
#ifdef NFAPI
void add_slot(uint16_t *frameP, uint16_t *slotP, int offset)
{
	uint16_t num_slots = 20; // set based on numerlogy (fixing for 1)

    *frameP    = (*frameP + ((*slotP + offset) / num_slots))%1024; 

    *slotP = ((*slotP + offset) % num_slots);
}

uint32_t sfnslot_add_slot(uint16_t sfn, uint16_t slot, int offset)
{
  uint32_t new_sfnslot;
  add_slot(&sfn, &slot, offset);

  new_sfnslot = sfn<<6|slot;

  return new_sfnslot;
}
#endif //NFAPI
/* ====================================== */

int phy_nr_slot_indication(nfapi_nr_slot_indication_scf_t *ind) {

  uint8_t vnf_slot_ahead = 0;
  uint32_t vnf_sfn_slot = sfnslot_add_slot(ind->sfn, ind->slot, vnf_slot_ahead);
  uint16_t vnf_sfn = NFAPI_SFNSLOT2SFN(vnf_sfn_slot);
  uint8_t vnf_slot = NFAPI_SFNSLOT2SLOT(vnf_sfn_slot);
  DU_LOG("\nINFO   --> VNF SFN/Slot %d.%d \n", vnf_sfn, vnf_slot);

  nfapi_nr_slot_indication_scf_t *nr_slot_ind = calloc(1, sizeof(*nr_slot_ind));
  nr_slot_ind->header = ind->header;
  nr_slot_ind->sfn = vnf_sfn;
  nr_slot_ind->slot = vnf_slot;
  if (!put_queue(&gnb_slot_ind_queue, nr_slot_ind))
  {
    DU_LOG("\nINFO   --> [VNF] Put_queue failed for slot_ind\n");
    free(nr_slot_ind);
    nr_slot_ind = NULL;
  }

  return 1;
}


int phy_nr_srs_indication(nfapi_nr_srs_indication_t *ind) {
  return 1;
}
//end NR phy indication

int phy_lbt_dl_indication(struct nfapi_vnf_p7_config *config, nfapi_lbt_dl_indication_t *ind) {
  return 1;
}

int phy_nb_harq_indication(struct nfapi_vnf_p7_config *config, nfapi_nb_harq_indication_t *ind) {
  return 1;
}

int phy_nrach_indication(struct nfapi_vnf_p7_config *config, nfapi_nrach_indication_t *ind) {
  return 1;
}

void *vnf_allocate(size_t size) {
  return (void *)malloc(size);
}

void vnf_deallocate(void *ptr) {
  free(ptr);
}


void vnf_trace(nfapi_trace_level_t nfapi_level, const char *message, ...) {
  va_list args;
  va_start(args, message);
  va_end(args);
}

int phy_vendor_ext(struct nfapi_vnf_p7_config *config, nfapi_p7_message_header_t *msg) {
  if(msg->message_id == P7_VENDOR_EXT_IND) {
  } else {
    DU_LOG("\nINFO   --> [VNF] unknown %02x\n", msg->message_id);
  }

  return 0;
}

nfapi_p7_message_header_t *phy_allocate_p7_vendor_ext(uint16_t message_id, uint16_t *msg_size) {
  if(message_id == P7_VENDOR_EXT_IND) {
    *msg_size = sizeof(vendor_ext_p7_ind);
    return (nfapi_p7_message_header_t *)malloc(sizeof(vendor_ext_p7_ind));
  }

  return 0;
}

void phy_deallocate_p7_vendor_ext(nfapi_p7_message_header_t *header) {
  free(header);
}

int phy_unpack_vendor_extension_tlv(nfapi_tl_t *tl, uint8_t **ppReadPackedMessage, uint8_t *end, void **ve, nfapi_p7_codec_config_t *codec) {
  (void)tl;
  (void)ppReadPackedMessage;
  (void)ve;
  return -1;
}

int phy_pack_vendor_extension_tlv(void *ve, uint8_t **ppWritePackedMsg, uint8_t *end, nfapi_p7_codec_config_t *codec) {
  nfapi_tl_t *tlv = (nfapi_tl_t *)ve;

  switch(tlv->tag) {
    case VENDOR_EXT_TLV_1_TAG: {
      vendor_ext_tlv_1 *ve = (vendor_ext_tlv_1 *)tlv;

      if(!push32(ve->dummy, ppWritePackedMsg, end))
        return 0;

      return 1;
    }
    break;

    default:
      return -1;
      break;
  }
}

int phy_unpack_p7_vendor_extension(nfapi_p7_message_header_t *header, uint8_t **ppReadPackedMessage, uint8_t *end, nfapi_p7_codec_config_t *config) {
  if(header->message_id == P7_VENDOR_EXT_IND) {
    vendor_ext_p7_ind *req = (vendor_ext_p7_ind *)(header);

    if(!pull16(ppReadPackedMessage, &req->error_code, end))
      return 0;
  }

  return 1;
}

int phy_pack_p7_vendor_extension(nfapi_p7_message_header_t *header, uint8_t **ppWritePackedMsg, uint8_t *end, nfapi_p7_codec_config_t *config) {
  if(header->message_id == P7_VENDOR_EXT_REQ) {
    vendor_ext_p7_req *req = (vendor_ext_p7_req *)(header);

    if(!(push16(req->dummy1, ppWritePackedMsg, end) &&
         push16(req->dummy2, ppWritePackedMsg, end)))
      return 0;
  }

  return 1;
}

int vnf_pack_p4_p5_vendor_extension(nfapi_p4_p5_message_header_t *header, uint8_t **ppWritePackedMsg, uint8_t *end, nfapi_p4_p5_codec_config_t *codec) {
  if(header->message_id == P5_VENDOR_EXT_REQ) {
    vendor_ext_p5_req *req = (vendor_ext_p5_req *)(header);
    return (!(push16(req->dummy1, ppWritePackedMsg, end) &&
              push16(req->dummy2, ppWritePackedMsg, end)));
  }

  return 0;
}

static pthread_t vnf_start_pthread;
static pthread_t vnf_p7_start_pthread;

void *vnf_nr_p7_start_thread(void *ptr) {
  DU_LOG("\nINFO   --> %s()\n", __FUNCTION__);
  pthread_setname_np(pthread_self(), "VNF_P7");
  nfapi_vnf_p7_config_t *config = (nfapi_vnf_p7_config_t *)ptr;
  nfapi_nr_vnf_p7_start(config);
  return config;
}

void *vnf_p7_start_thread(void *ptr) {
  DU_LOG("\nINFO   --> %s()\n", __FUNCTION__);
  pthread_setname_np(pthread_self(), "VNF_P7");
  nfapi_vnf_p7_config_t *config = (nfapi_vnf_p7_config_t *)ptr;
  nfapi_vnf_p7_start(config);
  return config;
}

/* ======== small cell integration ======== */
#ifdef NFAPI
void set_priority(int priority)
{
  struct sched_param param =
  {
    .sched_priority = priority,
  };
  fprintf(stderr, "Calling sched_setscheduler(%d)\n", priority);
  if (sched_setscheduler(0, SCHED_RR, &param) == -1)
  {
    fprintf(stderr, "sched_setscheduler: %s\n", strerror(errno));
    abort();
  }
}

void set_thread_priority(int priority)
{
    set_priority(priority);

    pthread_attr_t ptAttr;
    if (pthread_attr_setschedpolicy(&ptAttr, SCHED_RR) != 0)
    {
        printf("\n[NFAPI VNF]  ->  Failed to set pthread sched policy SCHED_RR");
    }

    pthread_attr_setinheritsched(&ptAttr, PTHREAD_EXPLICIT_SCHED);
    struct sched_param thread_params;
    thread_params.sched_priority = 20;

    if (pthread_attr_setschedparam(&ptAttr, &thread_params) != 0)
    {
        printf("\n[NFAPI VNF]  ->  failed to set sched param");
    }
}
#endif //NFAPI
/* ================================ */

void *vnf_nr_p7_thread_start(void *ptr) {
  set_thread_priority(79);
  init_queue(&gnb_rach_ind_queue);
  init_queue(&gnb_rx_ind_queue);
  init_queue(&gnb_crc_ind_queue);
  init_queue(&gnb_uci_ind_queue);
  init_queue(&gnb_slot_ind_queue);

  vnf_p7_info *p7_vnf = (vnf_p7_info *)ptr;
  p7_vnf->config->port = p7_vnf->local_port;
  p7_vnf->config->sync_indication = &phy_sync_indication;
  p7_vnf->config->slot_indication = &phy_slot_indication;
  p7_vnf->config->harq_indication = &phy_harq_indication;
  p7_vnf->config->nr_crc_indication = &phy_nr_crc_indication;
  p7_vnf->config->nr_rx_data_indication = &phy_nr_rx_data_indication;
  p7_vnf->config->nr_rach_indication = &phy_nr_rach_indication;
  p7_vnf->config->nr_uci_indication = &phy_nr_uci_indication;
  p7_vnf->config->srs_indication = &phy_srs_indication;
  p7_vnf->config->sr_indication = &phy_sr_indication;
  p7_vnf->config->cqi_indication = &phy_cqi_indication;
  p7_vnf->config->lbt_dl_indication = &phy_lbt_dl_indication;
  p7_vnf->config->nb_harq_indication = &phy_nb_harq_indication;
  p7_vnf->config->nrach_indication = &phy_nrach_indication;
  p7_vnf->config->nr_slot_indication = &phy_nr_slot_indication;
  p7_vnf->config->nr_srs_indication = &phy_nr_srs_indication;
  p7_vnf->config->malloc = &vnf_allocate;
  p7_vnf->config->free = &vnf_deallocate;
  p7_vnf->config->vendor_ext = &phy_vendor_ext;
  p7_vnf->config->user_data = p7_vnf;
  p7_vnf->mac->user_data = p7_vnf;
  p7_vnf->config->codec_config.unpack_p7_vendor_extension = &phy_unpack_p7_vendor_extension;
  p7_vnf->config->codec_config.pack_p7_vendor_extension = &phy_pack_p7_vendor_extension;
  p7_vnf->config->codec_config.unpack_vendor_extension_tlv = &phy_unpack_vendor_extension_tlv;
  p7_vnf->config->codec_config.pack_vendor_extension_tlv = &phy_pack_vendor_extension_tlv;
  p7_vnf->config->codec_config.allocate = &vnf_allocate;
  p7_vnf->config->codec_config.deallocate = &vnf_deallocate;
  p7_vnf->config->allocate_p7_vendor_ext = &phy_allocate_p7_vendor_ext;
  p7_vnf->config->deallocate_p7_vendor_ext = &phy_deallocate_p7_vendor_ext;
  DU_LOG("\nINFO   --> [VNF] Creating VNF NFAPI P7 start thread %s\n", __FUNCTION__);
  pthread_create(&vnf_p7_start_pthread, NULL, &vnf_nr_p7_start_thread, p7_vnf->config);
  return 0;
}

void *vnf_p7_thread_start(void *ptr) {
  set_thread_priority(79);
  vnf_p7_info *p7_vnf = (vnf_p7_info *)ptr;
  p7_vnf->config->port = p7_vnf->local_port;
  p7_vnf->config->sync_indication = &phy_sync_indication;
  p7_vnf->config->subframe_indication = &phy_subframe_indication;
  p7_vnf->config->slot_indication = &phy_slot_indication;

  p7_vnf->config->harq_indication = &phy_harq_indication;
  p7_vnf->config->crc_indication = &phy_crc_indication;
  p7_vnf->config->rx_indication = &phy_rx_indication;
  p7_vnf->config->rach_indication = &phy_rach_indication;
  p7_vnf->config->srs_indication = &phy_srs_indication;
  p7_vnf->config->sr_indication = &phy_sr_indication;
  p7_vnf->config->cqi_indication = &phy_cqi_indication;
  p7_vnf->config->lbt_dl_indication = &phy_lbt_dl_indication;
  p7_vnf->config->nb_harq_indication = &phy_nb_harq_indication;
  p7_vnf->config->nrach_indication = &phy_nrach_indication;
  p7_vnf->config->malloc = &vnf_allocate;
  p7_vnf->config->free = &vnf_deallocate;
  p7_vnf->config->vendor_ext = &phy_vendor_ext;
  p7_vnf->config->user_data = p7_vnf;
  p7_vnf->mac->user_data = p7_vnf;
  p7_vnf->config->codec_config.unpack_p7_vendor_extension = &phy_unpack_p7_vendor_extension;
  p7_vnf->config->codec_config.pack_p7_vendor_extension = &phy_pack_p7_vendor_extension;
  p7_vnf->config->codec_config.unpack_vendor_extension_tlv = &phy_unpack_vendor_extension_tlv;
  p7_vnf->config->codec_config.pack_vendor_extension_tlv = &phy_pack_vendor_extension_tlv;
  p7_vnf->config->codec_config.allocate = &vnf_allocate;
  p7_vnf->config->codec_config.deallocate = &vnf_deallocate;
  p7_vnf->config->allocate_p7_vendor_ext = &phy_allocate_p7_vendor_ext;
  p7_vnf->config->deallocate_p7_vendor_ext = &phy_deallocate_p7_vendor_ext;
  DU_LOG("\nINFO   --> [VNF] Creating VNF NFAPI P7 start thread %s\n", __FUNCTION__);
  pthread_create(&vnf_p7_start_pthread, NULL, &vnf_p7_start_thread, p7_vnf->config);
  return 0;
}

int pnf_nr_start_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_pnf_start_response_t *resp) {
  vnf_info *vnf = (vnf_info *)(config->user_data);
  vnf_p7_info *p7_vnf = vnf->p7_vnfs;
  pnf_info *pnf = vnf->pnfs;
  nfapi_nr_param_request_scf_t req;
  DU_LOG("\nINFO   --> [VNF] pnf start response idx:%d config:%p user_data:%p p7_vnf[config:%p thread_started:%d]\n", p5_idx, config, config->user_data, vnf->p7_vnfs[0].config, vnf->p7_vnfs[0].thread_started);

  if(p7_vnf->thread_started == 0) {
    pthread_t vnf_p7_thread;
    pthread_create(&vnf_p7_thread, NULL, &vnf_nr_p7_thread_start, p7_vnf);
    p7_vnf->thread_started = 1;
  } else {
    printf("\n[NFAPI VNF]  ->  P7 thread has been running");
  }

  // start all the phys in the pnf.
  DU_LOG("\nINFO   --> [VNF] Sending NFAPI_VNF_PARAM_REQUEST phy_id:%d\n", pnf->phys[0].id);
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_NR_PHY_MSG_TYPE_PARAM_REQUEST;
  req.header.phy_id = pnf->phys[0].id;
  nfapi_nr_vnf_param_req(config, p5_idx, &req);
  return 0;
}

int pnf_start_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_pnf_start_response_t *resp) {
  vnf_info *vnf = (vnf_info *)(config->user_data);
  vnf_p7_info *p7_vnf = vnf->p7_vnfs;
  pnf_info *pnf = vnf->pnfs;
  nfapi_param_request_t req;
  DU_LOG("\nINFO   --> [VNF] pnf start response idx:%d config:%p user_data:%p p7_vnf[config:%p thread_started:%d]\n", p5_idx, config, config->user_data, vnf->p7_vnfs[0].config, vnf->p7_vnfs[0].thread_started);

  if(p7_vnf->thread_started == 0) {
    pthread_t vnf_p7_thread;
    pthread_create(&vnf_p7_thread, NULL, &vnf_p7_thread_start, p7_vnf);
    p7_vnf->thread_started = 1;
  } else {
    // P7 thread already running.
  }

  // start all the phys in the pnf.
  DU_LOG("\nINFO   --> [VNF] Sending NFAPI_VNF_PARAM_REQUEST phy_id:%d\n", pnf->phys[0].id);
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_PARAM_REQUEST;
  req.header.phy_id = pnf->phys[0].id;
  nfapi_vnf_param_req(config, p5_idx, &req);
  return 0;
}

extern uint32_t to_earfcn(int eutra_bandP,uint32_t dl_CarrierFreq,uint32_t bw);

int nr_param_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_param_response_scf_t *resp) {

  printf("\nINFO   --> [VNF] Received NFAPI_PARAM_RESP idx:%d phy_id:%d\n", p5_idx, resp->header.phy_id);
  /* ======== small cell integration ======== */
  vnf_info *vnf = (vnf_info *)(config->user_data);
  vnf_p7_info *p7_vnf = vnf->p7_vnfs;
  pnf_info *pnf = vnf->pnfs;
  phy_info *phy = pnf->phys;
  struct sockaddr_in pnf_p7_sockaddr;
  phy->remote_port = resp->nfapi_config.p7_pnf_port.value;
  //phy->remote_port = 32123;//resp->nfapi_config.p7_pnf_port.value;
  memcpy(&pnf_p7_sockaddr.sin_addr.s_addr, &(resp->nfapi_config.p7_pnf_address_ipv4.address[0]), 4);
  phy->remote_addr = inet_ntoa(pnf_p7_sockaddr.sin_addr);
  // for now just 1
  printf("\nINFO   --> [VNF] %d.%d pnf p7 %s:%d timing %u %u %u %u\n", p5_idx, phy->id, phy->remote_addr, phy->remote_port, p7_vnf->timing_window, p7_vnf->periodic_timing_period, p7_vnf->aperiodic_timing_enabled,
         p7_vnf->periodic_timing_period);
  nfapi_vnf_p7_add_pnf((p7_vnf->config), phy->remote_addr, phy->remote_port, phy->id);
  nfapi_nr_config_request_scf_t *req = (nfapi_nr_config_request_scf_t*) calloc(1, sizeof(nfapi_nr_config_request_scf_t));   
  req->header.message_id = NFAPI_NR_PHY_MSG_TYPE_CONFIG_REQUEST;
  req->header.phy_id = phy->id;
  printf("\nINFO   --> [VNF] Send NFAPI_CONFIG_REQUEST\n");
  //DU_LOG("\nINFO   --> \n NR bandP =%d\n",req->nfapi_config.rf_bands.rf_band[0]);

  req->nfapi_config.p7_vnf_port.tl.tag = NFAPI_NR_NFAPI_P7_VNF_PORT_TAG;
  req->nfapi_config.p7_vnf_port.value = p7_vnf->local_port;
  req->num_tlv++;
  printf("\nINFO   --> [VNF] Local_port:%d\n", ntohs(p7_vnf->local_port));
  req->nfapi_config.p7_vnf_address_ipv4.tl.tag = NFAPI_NR_NFAPI_P7_VNF_ADDRESS_IPV4_TAG;
  struct sockaddr_in vnf_p7_sockaddr;
  vnf_p7_sockaddr.sin_addr.s_addr = inet_addr(p7_vnf->local_addr);
  memcpy(&(req->nfapi_config.p7_vnf_address_ipv4.address[0]), &vnf_p7_sockaddr.sin_addr.s_addr, 4);
  req->num_tlv++;
  printf("\nINFO   --> [VNF] Local_addr:%s\n", p7_vnf->local_addr);
  req->nfapi_config.timing_window.tl.tag = NFAPI_NR_NFAPI_TIMING_WINDOW_TAG;
  req->nfapi_config.timing_window.value = p7_vnf->timing_window;
  printf("\nINFO   --> \n[VNF]Timing window tag : %d Timing window:%u\n",NFAPI_NR_NFAPI_TIMING_WINDOW_TAG, p7_vnf->timing_window);
  req->num_tlv++;

  if(p7_vnf->periodic_timing_enabled || p7_vnf->aperiodic_timing_enabled) {
    req->nfapi_config.timing_info_mode.tl.tag = NFAPI_NR_NFAPI_TIMING_INFO_MODE_TAG;
    req->nfapi_config.timing_info_mode.value = (p7_vnf->aperiodic_timing_enabled << 1) | (p7_vnf->periodic_timing_enabled);
    req->num_tlv++;

    if(p7_vnf->periodic_timing_enabled) {
      req->nfapi_config.timing_info_period.tl.tag = NFAPI_NR_NFAPI_TIMING_INFO_PERIOD_TAG;
      req->nfapi_config.timing_info_period.value = p7_vnf->periodic_timing_period;
      req->num_tlv++;
    }
  }
//TODO: Assign tag and value for P7 message offsets
req->nfapi_config.dl_tti_timing_offset.tl.tag = NFAPI_NR_NFAPI_DL_TTI_TIMING_OFFSET;
req->nfapi_config.ul_tti_timing_offset.tl.tag = NFAPI_NR_NFAPI_UL_TTI_TIMING_OFFSET;
req->nfapi_config.ul_dci_timing_offset.tl.tag = NFAPI_NR_NFAPI_UL_DCI_TIMING_OFFSET;
req->nfapi_config.tx_data_timing_offset.tl.tag = NFAPI_NR_NFAPI_TX_DATA_TIMING_OFFSET;

  nfapi_nr_vnf_config_req(config, p5_idx, req);
  printf("[VNF] Sent NFAPI_VNF_CONFIG_REQ num_tlv:%u\n",req->num_tlv);
  return 0;
}


int param_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_param_response_t *resp) {
  return 0;
}

int nr_config_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_config_response_scf_t *resp) {
  nfapi_nr_start_request_scf_t req;
  DU_LOG("\nINFO   --> [VNF] Received NFAPI_CONFIG_RESP idx:%d phy_id:%d\n", p5_idx, resp->header.phy_id);
  DU_LOG("\nINFO   --> [VNF] Calling oai_enb_init()\n");
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_NR_PHY_MSG_TYPE_START_REQUEST;
  req.header.phy_id = resp->header.phy_id;
  nfapi_nr_vnf_start_req(config, p5_idx, &req);
  return 0;
}

int config_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_config_response_t *resp) {
  nfapi_start_request_t req;
  DU_LOG("\nINFO   --> [VNF] Received NFAPI_CONFIG_RESP idx:%d phy_id:%d\n", p5_idx, resp->header.phy_id);
  DU_LOG("\nINFO   --> [VNF] Calling oai_enb_init()\n");
  // oai_enb_init();
  memset(&req, 0, sizeof(req));
  req.header.message_id = NFAPI_START_REQUEST;
  req.header.phy_id = resp->header.phy_id;
  nfapi_vnf_start_req(config, p5_idx, &req);
  DU_LOG("\nINFO   --> [VNF] Send NFAPI_VNF_START_REQUEST idx:%d phy_id:%d\n", p5_idx, resp->header.phy_id);
  return 0;
}

int start_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_start_response_t *resp) {
  DU_LOG("\nINFO   --> [VNF] Received NFAPI_START_RESP idx:%d phy_id:%d\n", p5_idx, resp->header.phy_id);
  vnf_info *vnf = (vnf_info *)(config->user_data);
  pnf_info *pnf = vnf->pnfs;
  phy_info *phy = pnf->phys;
  vnf_p7_info *p7_vnf = vnf->p7_vnfs;
  nfapi_vnf_p7_add_pnf((p7_vnf->config), phy->remote_addr, htons(phy->remote_port), phy->id);
  return 0;
}

int nr_start_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_start_response_scf_t *resp) {
  DU_LOG("\nINFO   --> [VNF] Received NFAPI_START_RESP idx:%d phy_id:%d\n", p5_idx, resp->header.phy_id);
  vnf_info *vnf = (vnf_info *)(config->user_data);
  pnf_info *pnf = vnf->pnfs;
  phy_info *phy = pnf->phys;
  vnf_p7_info *p7_vnf = vnf->p7_vnfs;

 nfapi_vnf_p7_add_pnf((p7_vnf->config), phy->remote_addr, phy->remote_port, phy->id);
  printf("\nINFO  -->  Received NFAPI_START_RESP  DONE!!!\n");
  return 0;
}

int vendor_ext_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_p4_p5_message_header_t *msg) {
  DU_LOG("\nINFO   --> [VNF] %s\n", __FUNCTION__);

  switch(msg->message_id) {
    case P5_VENDOR_EXT_RSP: {
      vendor_ext_p5_rsp *rsp = (vendor_ext_p5_rsp *)msg;
      DU_LOG("\nINFO   --> [VNF] P5_VENDOR_EXT_RSP error_code:%d\n", rsp->error_code);
      // send the start request
      nfapi_pnf_start_request_t req;
      memset(&req, 0, sizeof(req));
      req.header.message_id = NFAPI_PNF_START_REQUEST;
      nfapi_vnf_pnf_start_req(config, p5_idx, &req);
    }
    break;
  }

  return 0;
}

int vnf_unpack_p4_p5_vendor_extension(nfapi_p4_p5_message_header_t *header, uint8_t **ppReadPackedMessage, uint8_t *end, nfapi_p4_p5_codec_config_t *codec) {
  if(header->message_id == P5_VENDOR_EXT_RSP) {
    vendor_ext_p5_rsp *req = (vendor_ext_p5_rsp *)(header);
    return(!pull16(ppReadPackedMessage, &req->error_code, end));
  }

  return 0;
}

nfapi_p4_p5_message_header_t *vnf_allocate_p4_p5_vendor_ext(uint16_t message_id, uint16_t *msg_size) {
  if(message_id == P5_VENDOR_EXT_RSP) {
    *msg_size = sizeof(vendor_ext_p5_rsp);
    return (nfapi_p4_p5_message_header_t *)malloc(sizeof(vendor_ext_p5_rsp));
  }

  return 0;
}

void vnf_deallocate_p4_p5_vendor_ext(nfapi_p4_p5_message_header_t *header) {
  free(header);
}

nfapi_vnf_config_t *config = 0;

void vnf_nr_start_thread(void *ptr) {
  DU_LOG("\nINFO   --> [VNF] VNF NFAPI thread - nfapi_vnf_start()%s\n", __FUNCTION__);
  pthread_setname_np(pthread_self(), "VNF");
  config = (nfapi_vnf_config_t *)ptr;
  nfapi_nr_vnf_start(config);
}

void vnf_start_thread(void *ptr) {
  DU_LOG("\nINFO   --> [VNF] VNF NFAPI thread - nfapi_vnf_start()%s\n", __FUNCTION__);
  pthread_setname_np(pthread_self(), "VNF");
  config = (nfapi_vnf_config_t *)ptr;
  nfapi_vnf_start(config);
}

static vnf_info vnf;

void configure_nr_nfapi_vnf(char *vnf_addr, int vnf_p5_port, char *pnf_ip_addr, int pnf_p7_port, int vnf_p7_port) {
  memset(&vnf, 0, sizeof(vnf));
  memset(vnf.p7_vnfs, 0, sizeof(vnf.p7_vnfs));
  vnf.p7_vnfs[0].timing_window = 30;
  vnf.p7_vnfs[0].periodic_timing_enabled = 0;
  vnf.p7_vnfs[0].aperiodic_timing_enabled = 0;
  vnf.p7_vnfs[0].periodic_timing_period = 1;
  vnf.p7_vnfs[0].config = nfapi_vnf_p7_config_create();
  DU_LOG("\nINFO   --> [VNF] %s() vnf.p7_vnfs[0].config:%p VNF ADDRESS:%s:%d\n", __FUNCTION__, vnf.p7_vnfs[0].config, vnf_addr, vnf_p5_port);
  strcpy(vnf.p7_vnfs[0].local_addr, vnf_addr);
  vnf.p7_vnfs[0].local_port = vnf_p7_port;
  vnf.p7_vnfs[0].mac = (mac_t *)malloc(sizeof(mac_t));
  nfapi_vnf_config_t *config = nfapi_vnf_config_create();
  config->malloc = malloc;
  config->free = free;
  config->vnf_p5_port = vnf_p5_port;
  config->vnf_ipv4 = 1;
  config->vnf_ipv6 = 0;
  config->pnf_list = 0;
  config->phy_list = 0;

  config->pnf_nr_connection_indication = &pnf_nr_connection_indication_cb;
  config->pnf_disconnect_indication = &pnf_disconnection_indication_cb;

  config->pnf_nr_param_resp = &pnf_nr_param_resp_cb;
  config->pnf_nr_config_resp = &pnf_nr_config_resp_cb;
  config->pnf_nr_start_resp = &pnf_nr_start_resp_cb;
  config->nr_param_resp = &nr_param_resp_cb;
  config->nr_config_resp = &nr_config_resp_cb;
  config->nr_start_resp = &nr_start_resp_cb;
  config->vendor_ext = &vendor_ext_cb;
  config->user_data = &vnf;
  // To allow custom vendor extentions to be added to nfapi
  config->codec_config.unpack_vendor_extension_tlv = &vnf_unpack_vendor_extension_tlv;
  config->codec_config.pack_vendor_extension_tlv = &vnf_pack_vendor_extension_tlv;
  config->codec_config.unpack_p4_p5_vendor_extension = &vnf_unpack_p4_p5_vendor_extension;
  config->codec_config.pack_p4_p5_vendor_extension = &vnf_pack_p4_p5_vendor_extension;
  config->allocate_p4_p5_vendor_ext = &vnf_allocate_p4_p5_vendor_ext;
  config->deallocate_p4_p5_vendor_ext = &vnf_deallocate_p4_p5_vendor_ext;
  config->codec_config.allocate = &vnf_allocate;
  config->codec_config.deallocate = &vnf_deallocate;
  // memset(&UL_RCC_INFO,0,sizeof(UL_RCC_IND_t));
  DU_LOG("\nINFO   --> [VNF] Creating VNF NFAPI start thread %s\n", __FUNCTION__);
  pthread_create(&vnf_start_pthread, NULL, (void *)&vnf_nr_start_thread, config);
  DU_LOG("\nINFO   --> [VNF] Created VNF NFAPI start thread %s\n", __FUNCTION__);
}


void configure_nfapi_vnf(char *vnf_addr, int vnf_p5_port, char *pnf_ip_addr, int pnf_p7_port, int vnf_p7_port) {
  memset(&vnf, 0, sizeof(vnf));
  memset(vnf.p7_vnfs, 0, sizeof(vnf.p7_vnfs));
  vnf.p7_vnfs[0].timing_window = 32;
  vnf.p7_vnfs[0].periodic_timing_enabled = 1;
  vnf.p7_vnfs[0].aperiodic_timing_enabled = 0;
  vnf.p7_vnfs[0].periodic_timing_period = 10;
  vnf.p7_vnfs[0].config = nfapi_vnf_p7_config_create();
  DU_LOG("\nINFO   --> [VNF] %s() vnf.p7_vnfs[0].config:%p VNF ADDRESS:%s:%d\n", __FUNCTION__, vnf.p7_vnfs[0].config, vnf_addr, vnf_p5_port);
  strcpy(vnf.p7_vnfs[0].local_addr, vnf_addr);
  vnf.p7_vnfs[0].local_port = vnf_p7_port;
  vnf.p7_vnfs[0].mac = (mac_t *)malloc(sizeof(mac_t));
  nfapi_vnf_config_t *config = nfapi_vnf_config_create();
  config->malloc = malloc;
  config->free = free;
  config->vnf_p5_port = vnf_p5_port;
  config->vnf_ipv4 = 1;
  config->vnf_ipv6 = 0;
  config->pnf_list = 0;
  config->phy_list = 0;

  config->pnf_connection_indication = &pnf_connection_indication_cb;
  config->pnf_disconnect_indication = &pnf_disconnection_indication_cb;

  config->pnf_param_resp = &pnf_param_resp_cb;
  config->pnf_config_resp = &pnf_config_resp_cb;
  config->pnf_start_resp = &pnf_start_resp_cb;
  config->param_resp = &param_resp_cb;
  config->config_resp = &config_resp_cb;
  config->start_resp = &start_resp_cb;
  config->vendor_ext = &vendor_ext_cb;
  config->user_data = &vnf;
  // To allow custom vendor extentions to be added to nfapi
  config->codec_config.unpack_vendor_extension_tlv = &vnf_unpack_vendor_extension_tlv;
  config->codec_config.pack_vendor_extension_tlv = &vnf_pack_vendor_extension_tlv;
  config->codec_config.unpack_p4_p5_vendor_extension = &vnf_unpack_p4_p5_vendor_extension;
  config->codec_config.pack_p4_p5_vendor_extension = &vnf_pack_p4_p5_vendor_extension;
  config->allocate_p4_p5_vendor_ext = &vnf_allocate_p4_p5_vendor_ext;
  config->deallocate_p4_p5_vendor_ext = &vnf_deallocate_p4_p5_vendor_ext;
  config->codec_config.allocate = &vnf_allocate;
  config->codec_config.deallocate = &vnf_deallocate;
  // memset(&UL_RCC_INFO,0,sizeof(UL_RCC_IND_t));
  DU_LOG("\nINFO   --> [VNF] Creating VNF NFAPI start thread %s\n", __FUNCTION__);
  pthread_create(&vnf_start_pthread, NULL, (void *)&vnf_start_thread, config);
  DU_LOG("\nINFO   --> [VNF] Created VNF NFAPI start thread %s\n", __FUNCTION__);
}

int oai_nfapi_dl_config_req(nfapi_dl_config_request_t *dl_config_req) {
 return 1;
}

int oai_nfapi_dl_tti_req(nfapi_nr_dl_tti_request_t *dl_config_req)
{
 return 1;
}

int oai_nfapi_tx_data_req(nfapi_nr_tx_data_request_t *tx_data_req)
{
 return 1;
}

int oai_nfapi_tx_req(nfapi_tx_request_t *tx_req)
{
 return 1;
}

int oai_nfapi_ul_dci_req(nfapi_nr_ul_dci_request_t *ul_dci_req) {
  return 1;
}

int oai_nfapi_hi_dci0_req(nfapi_hi_dci0_request_t *hi_dci0_req) {
  return 1;
}

static void remove_ul_config_req_pdu(int index, nfapi_ul_config_request_t *ul_config_req)
{
  int num_pdus = ul_config_req->ul_config_request_body.number_of_pdus;
  nfapi_ul_config_request_pdu_t *pdu_list = ul_config_req->ul_config_request_body.ul_config_pdu_list;

  if (index >= num_pdus || index < 0)
  {
    DU_LOG("\nINFO   --> [VNF] %s() Unable to drop bad ul_config_req PDU\n", __FUNCTION__);
    abort();
  }

  for(int i = index; i + 1 < num_pdus; i++)
  {
    pdu_list[i] = pdu_list[i + 1];
  }

  ul_config_req->ul_config_request_body.number_of_pdus--;
}

int oai_nfapi_ul_tti_req(nfapi_nr_ul_tti_request_t *ul_tti_req) {
 return 1;
}

int oai_nfapi_ul_config_req(nfapi_ul_config_request_t *ul_config_req) {
  return 1;
}

int oai_nfapi_ue_release_req(nfapi_ue_release_request_t *release_req){
   return 1;
}
