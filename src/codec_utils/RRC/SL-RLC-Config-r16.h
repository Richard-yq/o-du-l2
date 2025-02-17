/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_RLC_Config_r16_H_
#define	_SL_RLC_Config_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SN-FieldLengthAM.h"
#include "T-PollRetransmit.h"
#include "PollPDU.h"
#include "PollByte.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>
#include "SN-FieldLengthUM.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_RLC_Config_r16_PR {
	SL_RLC_Config_r16_PR_NOTHING,	/* No components present */
	SL_RLC_Config_r16_PR_sl_AM_RLC_r16,
	SL_RLC_Config_r16_PR_sl_UM_RLC_r16
	/* Extensions may appear below */
	
} SL_RLC_Config_r16_PR;
typedef enum SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16 {
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t1	= 0,
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t2	= 1,
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t3	= 2,
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t4	= 3,
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t6	= 4,
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t8	= 5,
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t16	= 6,
	SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16_t32	= 7
} e_SL_RLC_Config_r16__sl_AM_RLC_r16__sl_MaxRetxThreshold_r16;

/* SL-RLC-Config-r16 */
typedef struct SL_RLC_Config_r16 {
	SL_RLC_Config_r16_PR present;
	union SL_RLC_Config_r16_u {
		struct SL_RLC_Config_r16__sl_AM_RLC_r16 {
			SN_FieldLengthAM_t	*sl_SN_FieldLengthAM_r16;	/* OPTIONAL */
			T_PollRetransmit_t	 sl_T_PollRetransmit_r16;
			PollPDU_t	 sl_PollPDU_r16;
			PollByte_t	 sl_PollByte_r16;
			long	 sl_MaxRetxThreshold_r16;
			/*
			 * This type is extensible,
			 * possible extensions are below.
			 */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *sl_AM_RLC_r16;
		struct SL_RLC_Config_r16__sl_UM_RLC_r16 {
			SN_FieldLengthUM_t	*sl_SN_FieldLengthUM_r16;	/* OPTIONAL */
			/*
			 * This type is extensible,
			 * possible extensions are below.
			 */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *sl_UM_RLC_r16;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_RLC_Config_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_MaxRetxThreshold_r16_7;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_RLC_Config_r16;
extern asn_CHOICE_specifics_t asn_SPC_SL_RLC_Config_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_RLC_Config_r16_1[2];
extern asn_per_constraints_t asn_PER_type_SL_RLC_Config_r16_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _SL_RLC_Config_r16_H_ */
#include <asn_internal.h>
