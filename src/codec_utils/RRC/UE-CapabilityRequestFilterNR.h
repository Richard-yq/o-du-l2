/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UE_CapabilityRequestFilterNR_H_
#define	_UE_CapabilityRequestFilterNR_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct FreqBandList;
struct UE_CapabilityRequestFilterNR_v1540;

/* UE-CapabilityRequestFilterNR */
typedef struct UE_CapabilityRequestFilterNR {
	struct FreqBandList	*frequencyBandListFilter;	/* OPTIONAL */
	struct UE_CapabilityRequestFilterNR_v1540	*nonCriticalExtension;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UE_CapabilityRequestFilterNR_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UE_CapabilityRequestFilterNR;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "FreqBandList.h"
#include "UE-CapabilityRequestFilterNR-v1540.h"

#endif	/* _UE_CapabilityRequestFilterNR_H_ */
#include <asn_internal.h>
