/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_InterFreqBlackCellList_H_
#define	_InterFreqBlackCellList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PCI_Range;

/* InterFreqBlackCellList */
typedef struct InterFreqBlackCellList {
	A_SEQUENCE_OF(struct PCI_Range) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} InterFreqBlackCellList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_InterFreqBlackCellList;
extern asn_SET_OF_specifics_t asn_SPC_InterFreqBlackCellList_specs_1;
extern asn_TYPE_member_t asn_MBR_InterFreqBlackCellList_1[1];
extern asn_per_constraints_t asn_PER_type_InterFreqBlackCellList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PCI-Range.h"

#endif	/* _InterFreqBlackCellList_H_ */
#include <asn_internal.h>
