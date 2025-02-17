/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_GapConfig_H_
#define	_GapConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>
#include "ServCellIndex.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum GapConfig__mgl {
	GapConfig__mgl_ms1dot5	= 0,
	GapConfig__mgl_ms3	= 1,
	GapConfig__mgl_ms3dot5	= 2,
	GapConfig__mgl_ms4	= 3,
	GapConfig__mgl_ms5dot5	= 4,
	GapConfig__mgl_ms6	= 5
} e_GapConfig__mgl;
typedef enum GapConfig__mgrp {
	GapConfig__mgrp_ms20	= 0,
	GapConfig__mgrp_ms40	= 1,
	GapConfig__mgrp_ms80	= 2,
	GapConfig__mgrp_ms160	= 3
} e_GapConfig__mgrp;
typedef enum GapConfig__mgta {
	GapConfig__mgta_ms0	= 0,
	GapConfig__mgta_ms0dot25	= 1,
	GapConfig__mgta_ms0dot5	= 2
} e_GapConfig__mgta;
typedef enum GapConfig__ext1__refServCellIndicator {
	GapConfig__ext1__refServCellIndicator_pCell	= 0,
	GapConfig__ext1__refServCellIndicator_pSCell	= 1,
	GapConfig__ext1__refServCellIndicator_mcg_FR2	= 2
} e_GapConfig__ext1__refServCellIndicator;
typedef enum GapConfig__ext2__mgl_r16 {
	GapConfig__ext2__mgl_r16_ms10	= 0,
	GapConfig__ext2__mgl_r16_ms20	= 1
} e_GapConfig__ext2__mgl_r16;

/* GapConfig */
typedef struct GapConfig {
	long	 gapOffset;
	long	 mgl;
	long	 mgrp;
	long	 mgta;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct GapConfig__ext1 {
		long	*refServCellIndicator;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct GapConfig__ext2 {
		ServCellIndex_t	*refFR2ServCellAsyncCA_r16;	/* OPTIONAL */
		long	*mgl_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} GapConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_mgl_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mgrp_10;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mgta_15;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_refServCellIndicator_21;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mgl_r16_27;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_GapConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_GapConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_GapConfig_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _GapConfig_H_ */
#include <asn_internal.h>
