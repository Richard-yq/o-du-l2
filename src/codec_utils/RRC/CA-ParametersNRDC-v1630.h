/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CA_ParametersNRDC_v1630_H_
#define	_CA_ParametersNRDC_v1630_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct CA_ParametersNR_v1610;
struct CA_ParametersNR_v1630;

/* CA-ParametersNRDC-v1630 */
typedef struct CA_ParametersNRDC_v1630 {
	struct CA_ParametersNR_v1610	*ca_ParametersNR_ForDC_v1610;	/* OPTIONAL */
	struct CA_ParametersNR_v1630	*ca_ParametersNR_ForDC_v1630;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CA_ParametersNRDC_v1630_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CA_ParametersNRDC_v1630;
extern asn_SEQUENCE_specifics_t asn_SPC_CA_ParametersNRDC_v1630_specs_1;
extern asn_TYPE_member_t asn_MBR_CA_ParametersNRDC_v1630_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CA-ParametersNR-v1610.h"
#include "CA-ParametersNR-v1630.h"

#endif	/* _CA_ParametersNRDC_v1630_H_ */
#include <asn_internal.h>
