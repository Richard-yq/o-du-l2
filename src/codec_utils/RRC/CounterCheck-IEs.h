/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CounterCheck_IEs_H_
#define	_CounterCheck_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include "DRB-CountMSB-InfoList.h"
#include <OCTET_STRING.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CounterCheck-IEs */
typedef struct CounterCheck_IEs {
	DRB_CountMSB_InfoList_t	 drb_CountMSB_InfoList;
	OCTET_STRING_t	*lateNonCriticalExtension;	/* OPTIONAL */
	struct CounterCheck_IEs__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CounterCheck_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CounterCheck_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_CounterCheck_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_CounterCheck_IEs_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _CounterCheck_IEs_H_ */
#include <asn_internal.h>
