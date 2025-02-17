/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_PowerControl_r16_H_
#define	_SL_PowerControl_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16 {
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha0	= 0,
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha04	= 1,
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha05	= 2,
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha06	= 3,
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha07	= 4,
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha08	= 5,
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha09	= 6,
	SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16_alpha1	= 7
} e_SL_PowerControl_r16__sl_Alpha_PSSCH_PSCCH_r16;
typedef enum SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16 {
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha0	= 0,
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha04	= 1,
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha05	= 2,
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha06	= 3,
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha07	= 4,
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha08	= 5,
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha09	= 6,
	SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16_alpha1	= 7
} e_SL_PowerControl_r16__dl_Alpha_PSSCH_PSCCH_r16;
typedef enum SL_PowerControl_r16__dl_Alpha_PSFCH_r16 {
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha0	= 0,
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha04	= 1,
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha05	= 2,
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha06	= 3,
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha07	= 4,
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha08	= 5,
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha09	= 6,
	SL_PowerControl_r16__dl_Alpha_PSFCH_r16_alpha1	= 7
} e_SL_PowerControl_r16__dl_Alpha_PSFCH_r16;

/* SL-PowerControl-r16 */
typedef struct SL_PowerControl_r16 {
	long	 sl_MaxTransPower_r16;
	long	*sl_Alpha_PSSCH_PSCCH_r16;	/* OPTIONAL */
	long	*dl_Alpha_PSSCH_PSCCH_r16;	/* OPTIONAL */
	long	*sl_P0_PSSCH_PSCCH_r16;	/* OPTIONAL */
	long	*dl_P0_PSSCH_PSCCH_r16;	/* OPTIONAL */
	long	*dl_Alpha_PSFCH_r16;	/* OPTIONAL */
	long	*dl_P0_PSFCH_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SL_PowerControl_r16__ext1 {
		long	*dl_P0_PSSCH_PSCCH_r17;	/* OPTIONAL */
		long	*sl_P0_PSSCH_PSCCH_r17;	/* OPTIONAL */
		long	*dl_P0_PSFCH_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_PowerControl_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_Alpha_PSSCH_PSCCH_r16_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dl_Alpha_PSSCH_PSCCH_r16_12;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dl_Alpha_PSFCH_r16_23;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_PowerControl_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_PowerControl_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_PowerControl_r16_1[8];

#ifdef __cplusplus
}
#endif

#endif	/* _SL_PowerControl_r16_H_ */
#include <asn_internal.h>
