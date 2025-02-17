/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PosGapConfig_r17_H_
#define	_PosGapConfig_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MeasPosPreConfigGapId-r17.h"
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PosGapConfig_r17__mgl_r17 {
	PosGapConfig_r17__mgl_r17_ms1dot5	= 0,
	PosGapConfig_r17__mgl_r17_ms3	= 1,
	PosGapConfig_r17__mgl_r17_ms3dot5	= 2,
	PosGapConfig_r17__mgl_r17_ms4	= 3,
	PosGapConfig_r17__mgl_r17_ms5dot5	= 4,
	PosGapConfig_r17__mgl_r17_ms6	= 5,
	PosGapConfig_r17__mgl_r17_ms10	= 6,
	PosGapConfig_r17__mgl_r17_ms20	= 7
} e_PosGapConfig_r17__mgl_r17;
typedef enum PosGapConfig_r17__mgrp_r17 {
	PosGapConfig_r17__mgrp_r17_ms20	= 0,
	PosGapConfig_r17__mgrp_r17_ms40	= 1,
	PosGapConfig_r17__mgrp_r17_ms80	= 2,
	PosGapConfig_r17__mgrp_r17_ms160	= 3
} e_PosGapConfig_r17__mgrp_r17;
typedef enum PosGapConfig_r17__mgta_r17 {
	PosGapConfig_r17__mgta_r17_ms0	= 0,
	PosGapConfig_r17__mgta_r17_ms0dot25	= 1,
	PosGapConfig_r17__mgta_r17_ms0dot5	= 2
} e_PosGapConfig_r17__mgta_r17;
typedef enum PosGapConfig_r17__gapType_r17 {
	PosGapConfig_r17__gapType_r17_perUE	= 0,
	PosGapConfig_r17__gapType_r17_perFR1	= 1,
	PosGapConfig_r17__gapType_r17_perFR2	= 2
} e_PosGapConfig_r17__gapType_r17;

/* PosGapConfig-r17 */
typedef struct PosGapConfig_r17 {
	MeasPosPreConfigGapId_r17_t	 measPosPreConfigGapId_r17;
	long	 gapOffset_r17;
	long	 mgl_r17;
	long	 mgrp_r17;
	long	 mgta_r17;
	long	 gapType_r17;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PosGapConfig_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_mgl_r17_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mgrp_r17_13;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mgta_r17_18;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_gapType_r17_22;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PosGapConfig_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_PosGapConfig_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_PosGapConfig_r17_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _PosGapConfig_r17_H_ */
#include <asn_internal.h>
