/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_TxPercentageConfig_r16_H_
#define	_SL_TxPercentageConfig_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_TxPercentageConfig_r16__sl_TxPercentage_r16 {
	SL_TxPercentageConfig_r16__sl_TxPercentage_r16_p20	= 0,
	SL_TxPercentageConfig_r16__sl_TxPercentage_r16_p35	= 1,
	SL_TxPercentageConfig_r16__sl_TxPercentage_r16_p50	= 2
} e_SL_TxPercentageConfig_r16__sl_TxPercentage_r16;

/* SL-TxPercentageConfig-r16 */
typedef struct SL_TxPercentageConfig_r16 {
	long	 sl_Priority_r16;
	long	 sl_TxPercentage_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_TxPercentageConfig_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_TxPercentage_r16_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_TxPercentageConfig_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_TxPercentageConfig_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_TxPercentageConfig_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SL_TxPercentageConfig_r16_H_ */
#include <asn_internal.h>
