/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_P0_PUCCH_H_
#define	_P0_PUCCH_H_


#include <asn_application.h>

/* Including external dependencies */
#include "P0-PUCCH-Id.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* P0-PUCCH */
typedef struct P0_PUCCH {
	P0_PUCCH_Id_t	 p0_PUCCH_Id;
	long	 p0_PUCCH_Value;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} P0_PUCCH_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_P0_PUCCH;
extern asn_SEQUENCE_specifics_t asn_SPC_P0_PUCCH_specs_1;
extern asn_TYPE_member_t asn_MBR_P0_PUCCH_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _P0_PUCCH_H_ */
#include <asn_internal.h>
