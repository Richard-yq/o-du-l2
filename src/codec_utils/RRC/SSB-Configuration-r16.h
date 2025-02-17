/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SSB_Configuration_r16_H_
#define	_SSB_Configuration_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ARFCN-ValueNR.h"
#include <NativeEnumerated.h>
#include "SubcarrierSpacing.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SSB_Configuration_r16__halfFrameIndex_r16 {
	SSB_Configuration_r16__halfFrameIndex_r16_zero	= 0,
	SSB_Configuration_r16__halfFrameIndex_r16_one	= 1
} e_SSB_Configuration_r16__halfFrameIndex_r16;
typedef enum SSB_Configuration_r16__ssb_Periodicity_r16 {
	SSB_Configuration_r16__ssb_Periodicity_r16_ms5	= 0,
	SSB_Configuration_r16__ssb_Periodicity_r16_ms10	= 1,
	SSB_Configuration_r16__ssb_Periodicity_r16_ms20	= 2,
	SSB_Configuration_r16__ssb_Periodicity_r16_ms40	= 3,
	SSB_Configuration_r16__ssb_Periodicity_r16_ms80	= 4,
	SSB_Configuration_r16__ssb_Periodicity_r16_ms160	= 5,
	SSB_Configuration_r16__ssb_Periodicity_r16_spare2	= 6,
	SSB_Configuration_r16__ssb_Periodicity_r16_spare1	= 7
} e_SSB_Configuration_r16__ssb_Periodicity_r16;

/* SSB-Configuration-r16 */
typedef struct SSB_Configuration_r16 {
	ARFCN_ValueNR_t	 ssb_Freq_r16;
	long	 halfFrameIndex_r16;
	SubcarrierSpacing_t	 ssbSubcarrierSpacing_r16;
	long	*ssb_Periodicity_r16;	/* OPTIONAL */
	struct SSB_Configuration_r16__sfn0_Offset_r16 {
		long	 sfn_Offset_r16;
		long	*integerSubframeOffset_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *sfn0_Offset_r16;
	long	 sfn_SSB_Offset_r16;
	long	*ss_PBCH_BlockPower_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SSB_Configuration_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_halfFrameIndex_r16_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ssb_Periodicity_r16_7;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SSB_Configuration_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SSB_Configuration_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SSB_Configuration_r16_1[7];

#ifdef __cplusplus
}
#endif

#endif	/* _SSB_Configuration_r16_H_ */
#include <asn_internal.h>
