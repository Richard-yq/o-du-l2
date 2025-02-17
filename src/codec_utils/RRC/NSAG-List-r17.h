/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_NSAG_List_r17_H_
#define	_NSAG_List_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "NSAG-ID-r17.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NSAG-List-r17 */
typedef struct NSAG_List_r17 {
	A_SEQUENCE_OF(NSAG_ID_r17_t) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NSAG_List_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NSAG_List_r17;
extern asn_SET_OF_specifics_t asn_SPC_NSAG_List_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_NSAG_List_r17_1[1];
extern asn_per_constraints_t asn_PER_type_NSAG_List_r17_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _NSAG_List_r17_H_ */
#include <asn_internal.h>
