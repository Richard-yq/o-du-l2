/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PUCCH_PowerControlSetInfo_r17_H_
#define	_PUCCH_PowerControlSetInfo_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PUCCH-PowerControlSetInfoId-r17.h"
#include "P0-PUCCH-Id.h"
#include <NativeEnumerated.h>
#include "PUCCH-PathlossReferenceRS-Id-r17.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PUCCH_PowerControlSetInfo_r17__pucch_ClosedLoopIndex_r17 {
	PUCCH_PowerControlSetInfo_r17__pucch_ClosedLoopIndex_r17_i0	= 0,
	PUCCH_PowerControlSetInfo_r17__pucch_ClosedLoopIndex_r17_i1	= 1
} e_PUCCH_PowerControlSetInfo_r17__pucch_ClosedLoopIndex_r17;

/* PUCCH-PowerControlSetInfo-r17 */
typedef struct PUCCH_PowerControlSetInfo_r17 {
	PUCCH_PowerControlSetInfoId_r17_t	 pucch_PowerControlSetInfoId_r17;
	P0_PUCCH_Id_t	 p0_PUCCH_Id_r17;
	long	 pucch_ClosedLoopIndex_r17;
	PUCCH_PathlossReferenceRS_Id_r17_t	 pucch_PathlossReferenceRS_Id_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PUCCH_PowerControlSetInfo_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_pucch_ClosedLoopIndex_r17_4;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PUCCH_PowerControlSetInfo_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_PUCCH_PowerControlSetInfo_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_PUCCH_PowerControlSetInfo_r17_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _PUCCH_PowerControlSetInfo_r17_H_ */
#include <asn_internal.h>
