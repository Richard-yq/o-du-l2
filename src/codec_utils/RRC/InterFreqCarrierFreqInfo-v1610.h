/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_InterFreqCarrierFreqInfo_v1610_H_
#define	_InterFreqCarrierFreqInfo_v1610_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SSB-PositionQCL-Relation-r16.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct InterFreqNeighCellList_v1610;
struct SSB_MTC2_LP_r16;
struct InterFreqAllowedCellList_r16;
struct InterFreqCAG_CellListPerPLMN_r16;

/* InterFreqCarrierFreqInfo-v1610 */
typedef struct InterFreqCarrierFreqInfo_v1610 {
	struct InterFreqNeighCellList_v1610	*interFreqNeighCellList_v1610;	/* OPTIONAL */
	struct SSB_MTC2_LP_r16	*smtc2_LP_r16;	/* OPTIONAL */
	struct InterFreqAllowedCellList_r16	*interFreqAllowedCellList_r16;	/* OPTIONAL */
	SSB_PositionQCL_Relation_r16_t	*ssb_PositionQCL_Common_r16;	/* OPTIONAL */
	struct InterFreqCarrierFreqInfo_v1610__interFreqCAG_CellList_r16 {
		A_SEQUENCE_OF(struct InterFreqCAG_CellListPerPLMN_r16) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *interFreqCAG_CellList_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} InterFreqCarrierFreqInfo_v1610_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_InterFreqCarrierFreqInfo_v1610;
extern asn_SEQUENCE_specifics_t asn_SPC_InterFreqCarrierFreqInfo_v1610_specs_1;
extern asn_TYPE_member_t asn_MBR_InterFreqCarrierFreqInfo_v1610_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "InterFreqNeighCellList-v1610.h"
#include "SSB-MTC2-LP-r16.h"
#include "InterFreqAllowedCellList-r16.h"
#include "InterFreqCAG-CellListPerPLMN-r16.h"

#endif	/* _InterFreqCarrierFreqInfo_v1610_H_ */
#include <asn_internal.h>
