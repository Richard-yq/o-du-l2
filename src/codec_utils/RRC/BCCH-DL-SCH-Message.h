/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BCCH_DL_SCH_Message_H_
#define	_BCCH_DL_SCH_Message_H_


#include <asn_application.h>

/* Including external dependencies */
#include "BCCH-DL-SCH-MessageType.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BCCH-DL-SCH-Message */
typedef struct BCCH_DL_SCH_Message {
	BCCH_DL_SCH_MessageType_t	 message;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BCCH_DL_SCH_Message_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BCCH_DL_SCH_Message;

#ifdef __cplusplus
}
#endif

#endif	/* _BCCH_DL_SCH_Message_H_ */
#include <asn_internal.h>
