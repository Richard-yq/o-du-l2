/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_NTN_NeighCellConfig_r17_H_
#define	_NTN_NeighCellConfig_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ARFCN-ValueNR.h"
#include "PhysCellId.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct NTN_Config_r17;

/* NTN-NeighCellConfig-r17 */
typedef struct NTN_NeighCellConfig_r17 {
	struct NTN_Config_r17	*ntn_Config_r17;	/* OPTIONAL */
	ARFCN_ValueNR_t	*carrierFreq_r17;	/* OPTIONAL */
	PhysCellId_t	*physCellId_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NTN_NeighCellConfig_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NTN_NeighCellConfig_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_NTN_NeighCellConfig_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_NTN_NeighCellConfig_r17_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "NTN-Config-r17.h"

#endif	/* _NTN_NeighCellConfig_r17_H_ */
#include <asn_internal.h>
