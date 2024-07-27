/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BCCH_BCH_MessageType_H_
#define	_BCCH_BCH_MessageType_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BCCH_BCH_MessageType_PR {
	BCCH_BCH_MessageType_PR_NOTHING,	/* No components present */
	BCCH_BCH_MessageType_PR_mib,
	BCCH_BCH_MessageType_PR_messageClassExtension
} BCCH_BCH_MessageType_PR;

/* Forward declarations */
struct MIB;

/* BCCH-BCH-MessageType */
typedef struct BCCH_BCH_MessageType {
	BCCH_BCH_MessageType_PR present;
	union BCCH_BCH_MessageType_u {
		struct MIB	*mib;
		struct BCCH_BCH_MessageType__messageClassExtension {
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *messageClassExtension;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BCCH_BCH_MessageType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BCCH_BCH_MessageType;
extern asn_CHOICE_specifics_t asn_SPC_BCCH_BCH_MessageType_specs_1;
extern asn_TYPE_member_t asn_MBR_BCCH_BCH_MessageType_1[2];
extern asn_per_constraints_t asn_PER_type_BCCH_BCH_MessageType_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MIB.h"

#endif	/* _BCCH_BCH_MessageType_H_ */
#include <asn_internal.h>
