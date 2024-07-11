/*
 * Copyright 2017 Cisco Systems, Inc.
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


#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "vnf_p7.h"
#include "nfapi_vnf.h"
/* ======= SCF =======*/
#include <pthread.h>
#include "../cm/common_def.h"
#include "../5gnrmac/lwr_mac_phy.h"
#include "nfapi_nr_interface_scf.h"

pthread_mutex_t UL_INFO_mutex;
// end
/* ======== small cell integration ======== */
/*
#include "common/ran_context.h"

#include "openair1/PHY/defs_gNB.h"
*/
/* ======================================== */
#define FAPI2_IP_DSCP	0

// extern RAN_CONTEXT_t RC;

nfapi_vnf_p7_config_t* nfapi_vnf_p7_config_create()
{
	vnf_p7_t* _this = (vnf_p7_t*)calloc(1, sizeof(vnf_p7_t));

	if(_this == 0)
		return 0;

	// todo : initialize
	_this->_public.segment_size = 1400;
	_this->_public.max_num_segments = 8;
	_this->_public.checksum_enabled = 1;

	_this->_public.malloc = &malloc;
	_this->_public.free = &free;

	_this->_public.codec_config.allocate = &malloc;
	_this->_public.codec_config.deallocate = &free;


	return (nfapi_vnf_p7_config_t*)_this;
}

void nfapi_vnf_p7_config_destory(nfapi_vnf_p7_config_t* config)
{
	free(config);
}


struct timespec timespec_add(struct timespec lhs, struct timespec rhs)
{
	struct timespec result;

	result.tv_sec = lhs.tv_sec + rhs.tv_sec;
	result.tv_nsec = lhs.tv_nsec + rhs.tv_nsec;

	if(result.tv_nsec > 1e9)
	{
		result.tv_sec++;
		result.tv_nsec-= 1e9;
	}

	return result;
}

struct timespec timespec_sub(struct timespec lhs, struct timespec rhs)
{
	struct timespec result;
	if ((lhs.tv_nsec-rhs.tv_nsec)<0)
	{
		result.tv_sec = lhs.tv_sec-rhs.tv_sec-1;
		result.tv_nsec = 1000000000+lhs.tv_nsec-rhs.tv_nsec;
	}
	else
	{
		result.tv_sec = lhs.tv_sec-rhs.tv_sec;
		result.tv_nsec = lhs.tv_nsec-rhs.tv_nsec;
	}
	return result;
}

/* ======== small cell integration ======== */
static bool rx_ind_has_rnti(nfapi_nr_rx_data_indication_t *rx_ind, uint16_t rnti) {
  for (int i = 0; i < rx_ind->number_of_pdus; i++) {
    if (rnti == rx_ind->pdu_list[i].rnti) {
      return true;
    }
  }
  return false;
}

static void remove_crc_pdu(nfapi_nr_crc_indication_t *crc_ind, int index) {
  memmove(crc_ind->crc_list + index,
          crc_ind->crc_list + index + 1,
          sizeof(*crc_ind->crc_list) * (crc_ind->number_crcs - index - 1));
  crc_ind->number_crcs--;
}

static bool crc_ind_has_rnti(nfapi_nr_crc_indication_t *crc_ind, uint16_t rnti) {
  for (int i = 0; i < crc_ind->number_crcs; i++) {
    if (rnti == crc_ind->crc_list[i].rnti) {
      return true;
    }
  }
  return false;
}

static void remove_rx_pdu(nfapi_nr_rx_data_indication_t *rx_ind, int index) {
  memmove(rx_ind->pdu_list + index,
          rx_ind->pdu_list + index + 1,
          sizeof(*rx_ind->pdu_list) * (rx_ind->number_of_pdus - index - 1));
  rx_ind->number_of_pdus--;
}
/* ======================================== */

/* ======== small cell integration ======== */
static bool crc_sfn_slot_matcher(void *wanted, void *candidate)
{
  nfapi_p7_message_header_t *msg = candidate;
  int sfn_sf = *(int*)wanted;

  switch (msg->message_id)
  {
    case NFAPI_NR_PHY_MSG_TYPE_CRC_INDICATION:
    {
      nfapi_nr_crc_indication_t *ind = candidate;
      return NFAPI_SFNSLOT2SFN(sfn_sf) == ind->sfn && NFAPI_SFNSLOT2SLOT(sfn_sf) == ind->slot;
    }

    default:
      printf("sfn_slot_match bad ID: %d\n", msg->message_id);

  }
  return false;
}
/* ======================================== */

/* ======== small cell integration ======== */
static void match_crc_rx_pdu(nfapi_nr_rx_data_indication_t *rx_ind, nfapi_nr_crc_indication_t *crc_ind) {
  if (crc_ind->number_crcs > rx_ind->number_of_pdus) {
    int num_unmatched_crcs = 0;
    nfapi_nr_crc_indication_t *crc_ind_unmatched = calloc(1, sizeof(*crc_ind_unmatched));
    crc_ind_unmatched->header = crc_ind->header;
    crc_ind_unmatched->sfn = crc_ind->sfn;
    crc_ind_unmatched->slot = crc_ind->slot;
    crc_ind_unmatched->number_crcs = crc_ind->number_crcs - rx_ind->number_of_pdus;
    crc_ind_unmatched->crc_list = calloc(crc_ind_unmatched->number_crcs, sizeof(nfapi_nr_crc_t));
    for (int i = 0; i < crc_ind->number_crcs; i++) {
      if (!rx_ind_has_rnti(rx_ind, crc_ind->crc_list[i].rnti)) {
          printf("crc_ind->crc_list[%d].rnti %x does not match any rx_ind pdu rnti\n",
                i, crc_ind->crc_list[i].rnti);
          crc_ind_unmatched->crc_list[num_unmatched_crcs] = crc_ind->crc_list[i];
          num_unmatched_crcs++;
          remove_crc_pdu(crc_ind, i);
      }
      if (crc_ind->number_crcs == rx_ind->number_of_pdus) {
        break;
      }
    }
    if (!requeue(&gnb_crc_ind_queue, crc_ind_unmatched))
    {
      printf("requeue failed for crc_ind_unmatched.\n");
      free(crc_ind_unmatched->crc_list);
	  crc_ind_unmatched->crc_list = NULL;
      free(crc_ind_unmatched);
	  crc_ind_unmatched	= NULL;
    }
  }
  else if (crc_ind->number_crcs < rx_ind->number_of_pdus) {
    int num_unmatched_rxs = 0;
    nfapi_nr_rx_data_indication_t *rx_ind_unmatched = calloc(1, sizeof(*rx_ind_unmatched));
    rx_ind_unmatched->header = rx_ind->header;
    rx_ind_unmatched->sfn = rx_ind->sfn;
    rx_ind_unmatched->slot = rx_ind->slot;
    rx_ind_unmatched->number_of_pdus = rx_ind->number_of_pdus - crc_ind->number_crcs;
    rx_ind_unmatched->pdu_list = calloc(rx_ind_unmatched->number_of_pdus, sizeof(nfapi_nr_pdu_t));
    for (int i = 0; i < rx_ind->number_of_pdus; i++) {
      if (!crc_ind_has_rnti(crc_ind, rx_ind->pdu_list[i].rnti)) {
        printf("rx_ind->pdu_list[%d].rnti %d does not match any crc_ind pdu rnti\n",
              i, rx_ind->pdu_list[i].rnti);
        rx_ind_unmatched->pdu_list[num_unmatched_rxs] = rx_ind->pdu_list[i];
        num_unmatched_rxs++;
        remove_rx_pdu(rx_ind, i);
      }
      if (rx_ind->number_of_pdus == crc_ind->number_crcs) {
        break;
      }
    }
    if (!requeue(&gnb_rx_ind_queue, rx_ind_unmatched))
    {
      printf("requeue failed for rx_ind_unmatched.\n");
      free(rx_ind_unmatched->pdu_list);
	  rx_ind_unmatched->pdu_list = NULL;
      free(rx_ind_unmatched);
	  rx_ind_unmatched	= NULL;
    }
  }
  else {
    printf("The number of crc pdus %d = the number of rx pdus %d\n",
          crc_ind->number_crcs, rx_ind->number_of_pdus);
  }
}
/* ======================================== */

// monitor the p7 endpoints and the timing loop and
// send indications to mac
int nfapi_nr_vnf_p7_start(nfapi_vnf_p7_config_t* config)
{	
	/* ======== small cell integration ======== */
	// struct PHY_VARS_gNB_s *gNB = RC.gNB[0]; 
	/* ========================================= */
	uint8_t prev_slot = 0;
	if(config == 0)
		return -1;

	NFAPI_TRACE(NFAPI_TRACE_INFO, "%s()\n", __FUNCTION__);

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;

	// Create p7 receive udp port
	// todo : this needs updating for Ipv6
	printf("\n[NFAPI P7] ->  Initialising VNF P7 port:%u\n", config->port);
	//NFAPI_TRACE(NFAPI_TRACE_INFO, "Initialising VNF P7 port:%u\n", config->port);

	// open the UDP socket
	if ((vnf_p7->socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		NFAPI_TRACE(NFAPI_TRACE_ERROR, "After P7 socket errno: %d\n", errno);
		return -1;
	}

	printf("[NFAPI P7] ->  VNF P7 socket created...\n");
	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 socket created...\n");

	// configure the UDP socket options
	int iptos_value = FAPI2_IP_DSCP << 2;
	if (setsockopt(vnf_p7->socket, IPPROTO_IP, IP_TOS, &iptos_value, sizeof(iptos_value)) < 0)
	{
		NFAPI_TRACE(NFAPI_TRACE_ERROR, "After setsockopt (IP_TOS) errno: %d\n", errno);
		return -1;
	}
	printf("[NFAPI P7] ->  VNF P7 setsockopt succeeded...\n");
	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 setsockopt succeeded...\n");

	// Create the address structure
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(config->port);
	addr.sin_addr.s_addr = INADDR_ANY;

	// bind to the configured port
	printf("[NFAPI P7] ->  VNF P7 binding too %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 binding too %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
	if (bind(vnf_p7->socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) < 0)
	//if (sctp_bindx(config->socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in), 0) < 0)
	{
		NFAPI_TRACE(NFAPI_TRACE_ERROR, "After bind errno: %d\n", errno);
		return -1;
	}
	printf("[NFAPI P7] ->  VNF P7 bind succeeded...\n");
	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 bind succeeded...\n");
	printf("[DEBUG]	vnf_p7->terminate %d\n", vnf_p7->terminate);

	//struct timespec original_pselect_timeout;
	struct timespec pselect_timeout;
	pselect_timeout.tv_sec = 100; 
	pselect_timeout.tv_nsec = 0;

    struct timespec ref_time;
	clock_gettime(CLOCK_MONOTONIC, &ref_time);
	
	uint8_t setup_done = 1;
	while(vnf_p7->terminate == 0)
	{	
		fd_set rfds;
		int maxSock = 0;
		FD_ZERO(&rfds);
		int selectRetval = 0;

		// Add the p7 socket
		FD_SET(vnf_p7->socket, &rfds);
		maxSock = vnf_p7->socket;

		
		struct timespec curr_time;
		clock_gettime(CLOCK_MONOTONIC, &curr_time);
		uint8_t setup_time = curr_time.tv_sec - ref_time.tv_sec;
		

		/* ======== small cell integration ======== */
		
		nfapi_nr_slot_indication_scf_t *slot_ind = get_queue(&gnb_slot_ind_queue);
		if (gnb_rach_ind_queue.num_items > 0) {
			nfapi_nr_rach_indication_t *rach_ind = get_queue(&gnb_rach_ind_queue);
			UL_INFO.rach_ind = *rach_ind;
			printf("[DEBUG]	get_queue rach indication: %d\n", rach_ind);
		}
		if (gnb_rx_ind_queue.num_items > 0 && gnb_crc_ind_queue.num_items > 0) {
			nfapi_nr_rx_data_indication_t *rx_ind = get_queue(&gnb_rx_ind_queue);
			printf("[DEBUG]	get_queue rx indication: %d\n", rx_ind);
			int sfn_slot = NFAPI_SFNSLOT2HEX(rx_ind->sfn, rx_ind->slot);
      		nfapi_nr_crc_indication_t *crc_ind = unqueue_matching(&gnb_crc_ind_queue,
                                 MAX_QUEUE_SIZE,
                                 crc_sfn_slot_matcher,
                                 &sfn_slot);
			if (!crc_ind) {
				printf("No crc indication with the same SFN SLOT of rx indication %u %u\n", rx_ind->sfn, rx_ind->slot);
				requeue(&gnb_rx_ind_queue, rx_ind);
      		}
			else {
				printf("[DEBUG]	get_queue crc indication: %d\n", crc_ind);
				if (crc_ind->number_crcs != rx_ind->number_of_pdus)
					match_crc_rx_pdu(rx_ind, crc_ind);  
				UL_INFO.rx_ind = *rx_ind;
				UL_INFO.crc_ind = *crc_ind;
			}
		}
		if (gnb_uci_ind_queue.num_items > 0) {
			nfapi_nr_uci_indication_t *uci_ind = get_queue(&gnb_uci_ind_queue);
			printf("[DEBUG]	get_queue uci indication: %d\n", uci_ind);
			UL_INFO.uci_ind = *uci_ind;
		}
		if (slot_ind) {
			pthread_mutex_lock(&UL_INFO_mutex);
			UL_INFO.frame     = slot_ind->sfn;
			UL_INFO.slot      = slot_ind->slot;

			printf("[NFAPI_TRACE_DEBUG]  UL_INFO.frame = %d and slot %d, prev_slot = %d, setup_time = %d\n",
				    UL_INFO.frame, UL_INFO.slot, prev_slot, setup_time);
			if (setup_time > 3 && prev_slot != UL_INFO.slot) { 
				//Give the VNF sufficient time to setup before starting scheduling  && prev_slot != gNB->UL_INFO.slot

				//Call the scheduler
				UL_INFO.module_id = 0; // OAI default setting
				UL_INFO.CC_id     = 0; // OAI default setting
				SCF_procSlotInd(&UL_INFO);

				//TODO: Add switch case CRC、RACH、UCI、RX
				if (UL_INFO.rach_ind.number_of_pdus > 0){
					SCF_procRachInd(&UL_INFO.rach_ind);
					UL_INFO.rach_ind.number_of_pdus = 0;
				}
				if (UL_INFO.uci_ind.num_ucis > 0){
					SCF_procUciInd(&UL_INFO.uci_ind);
					UL_INFO.uci_ind.num_ucis = 0;
				}
				if (UL_INFO.rx_ind.number_of_pdus > 0 && UL_INFO.crc_ind.number_crcs > 0){
					SCF_procRxDataInd(&UL_INFO.rx_ind);
					SCF_procCrcInd(&UL_INFO.crc_ind);
					UL_INFO.rx_ind.number_of_pdus = 0;
					UL_INFO.crc_ind.number_crcs = 0;
				}
				prev_slot = UL_INFO.slot;
			}
			pthread_mutex_unlock(&UL_INFO_mutex);
			free(slot_ind);
			slot_ind = NULL;
		}

		selectRetval = pselect(maxSock+1, &rfds, NULL, NULL, &pselect_timeout, NULL);

		if(selectRetval == 0)
		{
			// pselect timed out, continue
		}
		else if(selectRetval > 0)
		{
			// have a p7 message
			if(FD_ISSET(vnf_p7->socket, &rfds))
			{	
				vnf_nr_p7_read_dispatch_message(vnf_p7); 				
			}
		}
		else
		{
			// pselect error
			if(selectRetval == -1 && errno == EINTR)
			{
				// a sigal was received.
			}
			else
			{
				//NFAPI_TRACE(NFAPI_TRACE_INFO, "P7 select failed result %d errno %d timeout:%d.%d orginal:%d.%d last_ms:%ld ms:%ld\n", selectRetval, errno, pselect_timeout.tv_sec, pselect_timeout.tv_nsec, pselect_timeout.tv_sec, pselect_timeout.tv_nsec, last_millisecond, millisecond);
				// should we exit now?
                                if (selectRetval == -1 && errno == 22) // invalid argument??? not sure about timeout duration
                                {
                                  usleep(100000);
                                }
			}
		}
	}
	NFAPI_TRACE(NFAPI_TRACE_INFO, "Closing p7 socket\n");
	close(vnf_p7->socket);

	NFAPI_TRACE(NFAPI_TRACE_INFO, "%s() returning\n", __FUNCTION__);

	return 0;
}


int nfapi_vnf_p7_start(nfapi_vnf_p7_config_t* config)
{
	if(config == 0)
		return -1;

	NFAPI_TRACE(NFAPI_TRACE_INFO, "%s()\n", __FUNCTION__);

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;

	// Create p7 receive udp port
	// todo : this needs updating for Ipv6

	NFAPI_TRACE(NFAPI_TRACE_INFO, "Initialising VNF P7 port:%u\n", config->port);

	// open the UDP socket
	if ((vnf_p7->socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		NFAPI_TRACE(NFAPI_TRACE_ERROR, "After P7 socket errno: %d\n", errno);
		return -1;
	}

	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 socket created...\n");

	// configure the UDP socket options
	int iptos_value = FAPI2_IP_DSCP << 2;
	if (setsockopt(vnf_p7->socket, IPPROTO_IP, IP_TOS, &iptos_value, sizeof(iptos_value)) < 0)
	{
		NFAPI_TRACE(NFAPI_TRACE_ERROR, "After setsockopt (IP_TOS) errno: %d\n", errno);
		return -1;
	}

	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 setsockopt succeeded...\n");

	// Create the address structure
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(config->port);
	addr.sin_addr.s_addr = INADDR_ANY;

	// bind to the configured port
	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 binding too %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
	if (bind(vnf_p7->socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) < 0)
	//if (sctp_bindx(config->socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in), 0) < 0)
	{
		NFAPI_TRACE(NFAPI_TRACE_ERROR, "After bind errno: %d\n", errno);
		return -1;
	}

	NFAPI_TRACE(NFAPI_TRACE_INFO, "VNF P7 bind succeeded...\n");


	//struct timespec original_pselect_timeout;
	struct timespec pselect_timeout;
	pselect_timeout.tv_sec = 0;
	pselect_timeout.tv_nsec = 1000000; // ns in a 1 us


	struct timespec pselect_start;
	struct timespec pselect_stop;

	//struct timespec sf_end;

	long last_millisecond = -1;


	struct timespec sf_duration;
	sf_duration.tv_sec = 0;
	sf_duration.tv_nsec = 1e6; // We want 1ms pause

	struct timespec sf_start;
	clock_gettime(CLOCK_MONOTONIC, &sf_start);
	long millisecond = sf_start.tv_nsec / 1e6;
	sf_start = timespec_add(sf_start, sf_duration);
	NFAPI_TRACE(NFAPI_TRACE_INFO, "next subframe will start at %ld.%ld\n", sf_start.tv_sec, sf_start.tv_nsec);

	while(vnf_p7->terminate == 0)
	{
		fd_set rfds;
		int maxSock = 0;
		FD_ZERO(&rfds);
		int selectRetval = 0;

		// Add the p7 socket
		FD_SET(vnf_p7->socket, &rfds);
		maxSock = vnf_p7->socket;

		clock_gettime(CLOCK_MONOTONIC, &pselect_start);
		//long millisecond = pselect_start.tv_nsec / 1e6;

		if((last_millisecond == -1) || (millisecond == last_millisecond) || (millisecond == (last_millisecond + 1) % 1000) )
		{
                  //NFAPI_TRACE(NFAPI_TRACE_INFO, "pselect_start:%d.%d sf_start:%d.%d\n", pselect_start.tv_sec, pselect_start.tv_nsec, sf_start.tv_sec, sf_start.tv_nsec);


			if((pselect_start.tv_sec > sf_start.tv_sec) ||
			   ((pselect_start.tv_sec == sf_start.tv_sec) && (pselect_start.tv_nsec > sf_start.tv_nsec)))
			{
				// overran the end of the subframe we do not want to wait
				pselect_timeout.tv_sec = 0;
				pselect_timeout.tv_nsec = 0;

				//struct timespec overrun = timespec_sub(pselect_start, sf_start);
				//NFAPI_TRACE(NFAPI_TRACE_INFO, "Subframe overrun detected of %d.%d running to catchup\n", overrun.tv_sec, overrun.tv_nsec);
			}
			else
			{
				// still time before the end of the subframe wait
				pselect_timeout = timespec_sub(sf_start, pselect_start);

			}

//original_pselect_timeout = pselect_timeout;

			// detemine how long to sleep in ns before the start of the next 1ms
			//pselect_timeout.tv_nsec = 1e6 - (pselect_start.tv_nsec % 1000000);

			//uint8_t underrun_possible =0;

			// if we are not sleeping until the next milisecond due to the
			// insycn minor adjment flag it so we don't consider it an error
			//uint8_t underrun_possible =0;
			/*
			{
				nfapi_vnf_p7_connection_info_t* phy = vnf_p7->p7_connections;
				if(phy && phy->in_sync && phy->insync_minor_adjustment != 0 && phy->insync_minor_adjustment_duration > 0 && pselect_start.tv_nsec != 0)
				{
					NFAPI_TRACE(NFAPI_TRACE_NOTE, "[VNF] Subframe minor adjustment %d (%d->%d)\n", phy->insync_minor_adjustment,
							pselect_timeout.tv_nsec, pselect_timeout.tv_nsec - (phy->insync_minor_adjustment * 1000))
					if(phy->insync_minor_adjustment > 0)
					{
						// todo check we don't go below 0
						if((phy->insync_minor_adjustment * 1000) > pselect_timeout.tv_nsec)
							pselect_timeout.tv_nsec = 0;
						else
							pselect_timeout.tv_nsec = pselect_timeout.tv_nsec - (phy->insync_minor_adjustment * 1000);


						//underrun_possible = 1;
					}
					else if(phy->insync_minor_adjustment < 0)
					{
						// todo check we don't go below 0
						pselect_timeout.tv_nsec = pselect_timeout.tv_nsec - (phy->insync_minor_adjustment * 1000);
					}

					//phy->insync_minor_adjustment = 0;
					phy->insync_minor_adjustment_duration--;
				}
			}
			*/


//long wraps = pselect_timeout.tv_nsec % 1e9;


			selectRetval = pselect(maxSock+1, &rfds, NULL, NULL, &pselect_timeout, NULL);

			clock_gettime(CLOCK_MONOTONIC, &pselect_stop);

                        nfapi_vnf_p7_connection_info_t* phy = vnf_p7->p7_connections;

if (selectRetval==-1 && errno == 22)
{
  NFAPI_TRACE(NFAPI_TRACE_ERROR, "INVAL: pselect_timeout:%ld.%ld adj[dur:%d adj:%d], sf_dur:%ld.%ld\n",
  pselect_timeout.tv_sec, pselect_timeout.tv_nsec,
  phy->insync_minor_adjustment_duration, phy->insync_minor_adjustment,
  sf_duration.tv_sec, sf_duration.tv_nsec);
}
			if(selectRetval == 0)
			{
				// calculate the start of the next subframe
				sf_start = timespec_add(sf_start, sf_duration);
				//NFAPI_TRACE(NFAPI_TRACE_INFO, "next subframe will start at %d.%d\n", sf_start.tv_sec, sf_start.tv_nsec);

				if(phy && phy->in_sync && phy->insync_minor_adjustment != 0 && phy->insync_minor_adjustment_duration > 0)
				{
                                        long insync_minor_adjustment_ns = (phy->insync_minor_adjustment * 1000);

                                        sf_start.tv_nsec -= insync_minor_adjustment_ns;

#if 1
                                        if (sf_start.tv_nsec > 1e9)
                                        {
                                          sf_start.tv_sec++;
                                          sf_start.tv_nsec-=1e9;
                                        }
                                        else if (sf_start.tv_nsec < 0)
                                        {
                                          sf_start.tv_sec--;
                                          sf_start.tv_nsec+=1e9;
                                        }
#else
                                        //NFAPI_TRACE(NFAPI_TRACE_NOTE, "[VNF] BEFORE adjustment - Subframe minor adjustment %dus sf_start.tv_nsec:%d\n", phy->insync_minor_adjustment, sf_start.tv_nsec);
					if(phy->insync_minor_adjustment > 0)
					{
						// decrease the subframe duration a little
                                                if (sf_start.tv_nsec > insync_minor_adjustment_ns)
                                                  sf_start.tv_nsec -= insync_minor_adjustment_ns;
                                                else
                                                {
                                                  NFAPI_TRACE(NFAPI_TRACE_ERROR, "[VNF] Adjustment would make it negative sf:%d.%ld adjust:%ld\n\n\n", sf_start.tv_sec, sf_start.tv_nsec, insync_minor_adjustment_ns);
                                                  sf_start.tv_sec--;
                                                  sf_start.tv_nsec += 1e9 - insync_minor_adjustment_ns;
                                                }
					}
					else if(phy->insync_minor_adjustment < 0)
					{
						// todo check we don't go below 0
						// increase the subframe duration a little
						sf_start.tv_nsec += insync_minor_adjustment_ns;

                                                if (sf_start.tv_nsec < 0)
                                                {
                                                  NFAPI_TRACE(NFAPI_TRACE_ERROR, "[VNF] OVERFLOW %d.%ld\n\n\n\n", sf_start.tv_sec, sf_start.tv_nsec);
                                                  sf_start.tv_sec++;
                                                  sf_start.tv_nsec += 1e9;
                                                }
					}
#endif

					//phy->insync_minor_adjustment = 0;
                                        phy->insync_minor_adjustment_duration--;

                                        NFAPI_TRACE(NFAPI_TRACE_NOTE, "[VNF] AFTER adjustment - Subframe minor adjustment %dus sf_start.tv_nsec:%ld duration:%u\n",
                                            phy->insync_minor_adjustment, sf_start.tv_nsec, phy->insync_minor_adjustment_duration);

                                        if (phy->insync_minor_adjustment_duration==0)
                                        {
                                          phy->insync_minor_adjustment = 0;
                                        }
				}
				/*
				long pselect_stop_millisecond = pselect_stop.tv_nsec / 1e6;
				if(millisecond == pselect_stop_millisecond)
				{
					// we have woke up in the same subframe
					if(underrun_possible == 0)
						NFAPI_TRACE(NFAPI_TRACE_WARN, "subframe pselect underrun %ld (%d.%d)\n", millisecond, pselect_stop.tv_sec, pselect_stop.tv_nsec);
				}
				else if(((millisecond + 1) % 1000) != pselect_stop_millisecond)
				{
					// we have overrun the subframe
					NFAPI_TRACE(NFAPI_TRACE_WARN, "subframe pselect overrun %ld %ld\n", millisecond, pselect_stop_millisecond);
					NFAPI_TRACE(NFAPI_TRACE_WARN, "subframe underrun %ld\n", millisecond);
				}
				last_millisecond = millisecond;
				*/

				millisecond ++;
			}
		}
		else
		{
			// we have overrun the subframe advance to go and collect $200
			if((millisecond - last_millisecond) > 3)
				NFAPI_TRACE(NFAPI_TRACE_WARN, "subframe overrun %ld %ld (%ld)\n", millisecond, last_millisecond, millisecond - last_millisecond + 1);

			last_millisecond = ( last_millisecond + 1 ) % 1000;
			selectRetval = 0;
		}

		if(selectRetval == 0)
		{
			vnf_p7->sf_start_time_hr = vnf_get_current_time_hr();

			// pselect timed out
			nfapi_vnf_p7_connection_info_t* curr = vnf_p7->p7_connections;

			while(curr != 0)
			{
				curr->sfn_sf = increment_sfn_sf(curr->sfn_sf);
				vnf_sync(vnf_p7, curr);
				curr = curr->next;
			}

			send_mac_subframe_indications(vnf_p7);

		}
		else if(selectRetval > 0)
		{
			// have a p7 message
			if(FD_ISSET(vnf_p7->socket, &rfds))
			{
				vnf_p7_read_dispatch_message(vnf_p7);
			}
		}
		else
		{
			// pselect error
			if(selectRetval == -1 && errno == EINTR)
			{
				// a sigal was received.
			}
			else
			{
				NFAPI_TRACE(NFAPI_TRACE_INFO, "P7 select failed result %d errno %d timeout:%ld.%ld orginal:%ld.%ld last_ms:%ld ms:%ld\n", selectRetval, errno, pselect_timeout.tv_sec, pselect_timeout.tv_nsec, pselect_timeout.tv_sec, pselect_timeout.tv_nsec, last_millisecond, millisecond);
				// should we exit now?
                                if (selectRetval == -1 && errno == 22) // invalid argument??? not sure about timeout duration
                                {
                                  usleep(100000);
                                }
			}
		}

	}


	NFAPI_TRACE(NFAPI_TRACE_INFO, "Closing p7 socket\n");
	close(vnf_p7->socket);

	NFAPI_TRACE(NFAPI_TRACE_INFO, "%s() returning\n", __FUNCTION__);

	return 0;
}


int nfapi_vnf_p7_stop(nfapi_vnf_p7_config_t* config)
{
	if(config == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	vnf_p7->terminate =1;
	return 0;
}

int nfapi_vnf_p7_add_pnf(nfapi_vnf_p7_config_t* config, const char* pnf_p7_addr, int pnf_p7_port, int phy_id)
{
	printf("\n%s(config:%p phy_id:%d pnf_addr:%s pnf_p7_port:%d)\n", __FUNCTION__, config, phy_id,  pnf_p7_addr, pnf_p7_port);

	if(config == 0)
        {
          return -1;
        }

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;

	nfapi_vnf_p7_connection_info_t* node = (nfapi_vnf_p7_connection_info_t*)malloc(sizeof(nfapi_vnf_p7_connection_info_t));

	memset(node, 0, sizeof(nfapi_vnf_p7_connection_info_t));
	node->phy_id = phy_id;
	node->in_sync = 0;
	node->dl_out_sync_offset = 30;//TODO: Values need to be changed for NR,How to set the values
	node->dl_out_sync_period = 10;
	node->dl_in_sync_offset = 30;
	node->dl_in_sync_period = 512;
	//node->sfn_sf = 0;
	node->sfn = 0;
    node->slot = 0;
	node->min_sync_cycle_count = 8;

	// save the remote endpoint information
	node->remote_addr.sin_family = AF_INET;
	node->remote_addr.sin_port =  pnf_p7_port;//htons(pnf_p7_port);
	node->remote_addr.sin_addr.s_addr = inet_addr(pnf_p7_addr);

	vnf_p7_connection_info_list_add(vnf_p7, node);

	return 0;
}

int nfapi_vnf_p7_del_pnf(nfapi_vnf_p7_config_t* config, int phy_id)
{
	NFAPI_TRACE(NFAPI_TRACE_INFO, "%s(phy_id:%d)\n", __FUNCTION__, phy_id);

	if(config == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;

	nfapi_vnf_p7_connection_info_t* to_delete = vnf_p7_connection_info_list_delete(vnf_p7, phy_id);

	if(to_delete)
	{
		NFAPI_TRACE(NFAPI_TRACE_INFO, "%s(phy_id:%d) deleting connection info\n", __FUNCTION__, phy_id);
		free(to_delete);
	}

	return 0;
}
int nfapi_vnf_p7_dl_config_req(nfapi_vnf_p7_config_t* config, nfapi_dl_config_request_t* req)
{
	//NFAPI_TRACE(NFAPI_TRACE_INFO, "%s(config:%p req:%p)\n", __FUNCTION__, config, req);

	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}

int nfapi_vnf_p7_nr_dl_config_req(nfapi_vnf_p7_config_t* config, nfapi_nr_dl_tti_request_t* req)
{
	printf("\n%s(config:%p req:%p)\n", __FUNCTION__, config, req);

	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_nr_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}

int nfapi_vnf_p7_ul_tti_req(nfapi_vnf_p7_config_t* config, nfapi_nr_ul_tti_request_t* req)
{
	if(config == 0 || req == 0)
		return -1;
	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_nr_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}

int nfapi_vnf_p7_ul_config_req(nfapi_vnf_p7_config_t* config, nfapi_ul_config_request_t* req)
{
	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}
int nfapi_vnf_p7_ul_dci_req(nfapi_vnf_p7_config_t* config, nfapi_nr_ul_dci_request_t* req)
{
	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_nr_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}
int nfapi_vnf_p7_hi_dci0_req(nfapi_vnf_p7_config_t* config, nfapi_hi_dci0_request_t* req)
{
	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}
int nfapi_vnf_p7_tx_data_req(nfapi_vnf_p7_config_t* config, nfapi_nr_tx_data_request_t* req)
{
	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_nr_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}
int nfapi_vnf_p7_tx_req(nfapi_vnf_p7_config_t* config, nfapi_tx_request_t* req)
{
	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}
int nfapi_vnf_p7_lbt_dl_config_req(nfapi_vnf_p7_config_t* config, nfapi_lbt_dl_config_request_t* req)
{
	if(config == 0 || req == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}
int nfapi_vnf_p7_vendor_extension(nfapi_vnf_p7_config_t* config, nfapi_p7_message_header_t* header)
{
	if(config == 0 || header == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	return vnf_p7_pack_and_send_p7_msg(vnf_p7, header);
}

int nfapi_vnf_p7_ue_release_req(nfapi_vnf_p7_config_t* config, nfapi_ue_release_request_t* req)
{
    if(config == 0 || req == 0)
        return -1;

    vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
    return vnf_p7_pack_and_send_p7_msg(vnf_p7, &req->header);
}

int nfapi_vnf_p7_release_msg(nfapi_vnf_p7_config_t* config, nfapi_p7_message_header_t* header)
{
	if(config == 0 || header == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	vnf_p7_release_msg(vnf_p7, header);

	return 0;

}

int nfapi_vnf_p7_release_pdu(nfapi_vnf_p7_config_t* config, void* pdu)
{
	if(config == 0 || pdu == 0)
		return -1;

	vnf_p7_t* vnf_p7 = (vnf_p7_t*)config;
	vnf_p7_release_pdu(vnf_p7, pdu);

	return 0;
}
