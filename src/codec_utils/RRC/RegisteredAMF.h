/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RegisteredAMF_H_
#define	_RegisteredAMF_H_


#include <asn_application.h>

/* Including external dependencies */
#include "AMF-Identifier.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PLMN_IdentitY;

/* RegisteredAMF */
typedef struct RegisteredAMF {
	struct PLMN_IdentitY	*plmn_Identity;	/* OPTIONAL */
	AMF_Identifier_t	 amf_Identifier;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RegisteredAMF_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RegisteredAMF;
extern asn_SEQUENCE_specifics_t asn_SPC_RegisteredAMF_specs_1;
extern asn_TYPE_member_t asn_MBR_RegisteredAMF_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PLMN-IdentitY.h"

#endif	/* _RegisteredAMF_H_ */
#include <asn_internal.h>
