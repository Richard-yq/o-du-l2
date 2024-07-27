/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UAC_BarringInfoSetList_H_
#define	_UAC_BarringInfoSetList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct UAC_BarringInfoSet;

/* UAC-BarringInfoSetList */
typedef struct UAC_BarringInfoSetList {
	A_SEQUENCE_OF(struct UAC_BarringInfoSet) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UAC_BarringInfoSetList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UAC_BarringInfoSetList;
extern asn_SET_OF_specifics_t asn_SPC_UAC_BarringInfoSetList_specs_1;
extern asn_TYPE_member_t asn_MBR_UAC_BarringInfoSetList_1[1];
extern asn_per_constraints_t asn_PER_type_UAC_BarringInfoSetList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "UAC-BarringInfoSet.h"

#endif	/* _UAC_BarringInfoSetList_H_ */
#include <asn_internal.h>
