/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_ULTxSwitchingBandPair_r16_H_
#define	_ULTxSwitchingBandPair_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <BIT_STRING.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ULTxSwitchingBandPair_r16__uplinkTxSwitchingPeriod_r16 {
	ULTxSwitchingBandPair_r16__uplinkTxSwitchingPeriod_r16_n35us	= 0,
	ULTxSwitchingBandPair_r16__uplinkTxSwitchingPeriod_r16_n140us	= 1,
	ULTxSwitchingBandPair_r16__uplinkTxSwitchingPeriod_r16_n210us	= 2
} e_ULTxSwitchingBandPair_r16__uplinkTxSwitchingPeriod_r16;

/* ULTxSwitchingBandPair-r16 */
typedef struct ULTxSwitchingBandPair_r16 {
	long	 bandIndexUL1_r16;
	long	 bandIndexUL2_r16;
	long	 uplinkTxSwitchingPeriod_r16;
	BIT_STRING_t	*uplinkTxSwitching_DL_Interruption_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ULTxSwitchingBandPair_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_uplinkTxSwitchingPeriod_r16_4;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_ULTxSwitchingBandPair_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_ULTxSwitchingBandPair_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_ULTxSwitchingBandPair_r16_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _ULTxSwitchingBandPair_r16_H_ */
#include <asn_internal.h>
