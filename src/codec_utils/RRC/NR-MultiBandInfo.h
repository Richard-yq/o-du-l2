/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_NR_MultiBandInfo_H_
#define	_NR_MultiBandInfo_H_


#include <asn_application.h>

/* Including external dependencies */
#include "FreqBandIndicatorNR.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct NR_NS_PmaxList;

/* NR-MultiBandInfo */
typedef struct NR_MultiBandInfo {
	FreqBandIndicatorNR_t	*freqBandIndicatorNR;	/* OPTIONAL */
	struct NR_NS_PmaxList	*nr_NS_PmaxList;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NR_MultiBandInfo_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NR_MultiBandInfo;
extern asn_SEQUENCE_specifics_t asn_SPC_NR_MultiBandInfo_specs_1;
extern asn_TYPE_member_t asn_MBR_NR_MultiBandInfo_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "NR-NS-PmaxList.h"

#endif	/* _NR_MultiBandInfo_H_ */
#include <asn_internal.h>
