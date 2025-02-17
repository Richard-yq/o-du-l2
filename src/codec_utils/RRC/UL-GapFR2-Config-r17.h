/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UL_GapFR2_Config_r17_H_
#define	_UL_GapFR2_Config_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include "ServCellIndex.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UL_GapFR2_Config_r17__ugl_r17 {
	UL_GapFR2_Config_r17__ugl_r17_ms0dot125	= 0,
	UL_GapFR2_Config_r17__ugl_r17_ms0dot25	= 1,
	UL_GapFR2_Config_r17__ugl_r17_ms0dot5	= 2,
	UL_GapFR2_Config_r17__ugl_r17_ms1	= 3
} e_UL_GapFR2_Config_r17__ugl_r17;
typedef enum UL_GapFR2_Config_r17__ugrp_r17 {
	UL_GapFR2_Config_r17__ugrp_r17_ms5	= 0,
	UL_GapFR2_Config_r17__ugrp_r17_ms20	= 1,
	UL_GapFR2_Config_r17__ugrp_r17_ms40	= 2,
	UL_GapFR2_Config_r17__ugrp_r17_ms160	= 3
} e_UL_GapFR2_Config_r17__ugrp_r17;

/* UL-GapFR2-Config-r17 */
typedef struct UL_GapFR2_Config_r17 {
	long	 gapOffset_r17;
	long	 ugl_r17;
	long	 ugrp_r17;
	ServCellIndex_t	*refFR2_ServCellAsyncCA_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UL_GapFR2_Config_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ugl_r17_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ugrp_r17_8;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_UL_GapFR2_Config_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_UL_GapFR2_Config_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_UL_GapFR2_Config_r17_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _UL_GapFR2_Config_r17_H_ */
#include <asn_internal.h>
