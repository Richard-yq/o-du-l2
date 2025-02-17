/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UplinkTxDirectCurrentTwoCarrierInfo_r16_H_
#define	_UplinkTxDirectCurrentTwoCarrierInfo_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ServCellIndex.h"
#include <BOOLEAN.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UplinkTxDirectCurrentTwoCarrierInfo-r16 */
typedef struct UplinkTxDirectCurrentTwoCarrierInfo_r16 {
	ServCellIndex_t	 referenceCarrierIndex_r16;
	BOOLEAN_t	 shift7dot5kHz_r16;
	long	 txDirectCurrentLocation_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UplinkTxDirectCurrentTwoCarrierInfo_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UplinkTxDirectCurrentTwoCarrierInfo_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_UplinkTxDirectCurrentTwoCarrierInfo_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_UplinkTxDirectCurrentTwoCarrierInfo_r16_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _UplinkTxDirectCurrentTwoCarrierInfo_r16_H_ */
#include <asn_internal.h>
