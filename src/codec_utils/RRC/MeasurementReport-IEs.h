/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasurementReport_IEs_H_
#define	_MeasurementReport_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MeasResults.h"
#include <OCTET_STRING.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MeasurementReport-IEs */
typedef struct MeasurementReport_IEs {
	MeasResults_t	 measResults;
	OCTET_STRING_t	*lateNonCriticalExtension;	/* OPTIONAL */
	struct MeasurementReport_IEs__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasurementReport_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasurementReport_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasurementReport_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasurementReport_IEs_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _MeasurementReport_IEs_H_ */
#include <asn_internal.h>
