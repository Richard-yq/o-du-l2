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


void configure_nfapi_vnf(char *vnf_addr, int vnf_p5_port, char *pnf_ip_addr, int pnf_p7_port, int vnf_p7_port);
void configure_nr_nfapi_vnf(char *vnf_addr, int vnf_p5_port, char *pnf_ip_addr, int pnf_p7_port, int vnf_p7_port);


/* ======== small cell integration ======== */
#ifdef NFAPI
int pnf_nr_connection_indication_cb(nfapi_vnf_config_t *config, int p5_idx);
int pnf_disconnection_indication_cb(nfapi_vnf_config_t *config, int p5_idx);
int pnf_nr_param_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_pnf_param_response_t *resp);

int pnf_nr_config_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_pnf_config_response_t *resp);
int pnf_nr_start_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_pnf_start_response_t *resp);

// After pnf is set to running state
int nr_param_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_param_response_scf_t *resp);
int nr_config_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_config_response_scf_t *resp);
int nr_start_resp_cb(nfapi_vnf_config_t *config, int p5_idx, nfapi_nr_start_response_scf_t *resp);
#endif // NFAPI

uint32_t sfnslot_add_slot(uint16_t sfn, uint16_t slot, int offset);

