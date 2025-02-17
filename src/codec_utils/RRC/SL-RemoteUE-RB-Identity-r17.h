/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_RemoteUE_RB_Identity_r17_H_
#define	_SL_RemoteUE_RB_Identity_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include "DRB-Identity.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_RemoteUE_RB_Identity_r17_PR {
	SL_RemoteUE_RB_Identity_r17_PR_NOTHING,	/* No components present */
	SL_RemoteUE_RB_Identity_r17_PR_srb_Identity_r17,
	SL_RemoteUE_RB_Identity_r17_PR_drb_Identity_r17
	/* Extensions may appear below */
	
} SL_RemoteUE_RB_Identity_r17_PR;

/* SL-RemoteUE-RB-Identity-r17 */
typedef struct SL_RemoteUE_RB_Identity_r17 {
	SL_RemoteUE_RB_Identity_r17_PR present;
	union SL_RemoteUE_RB_Identity_r17_u {
		long	 srb_Identity_r17;
		DRB_Identity_t	 drb_Identity_r17;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_RemoteUE_RB_Identity_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SL_RemoteUE_RB_Identity_r17;
extern asn_CHOICE_specifics_t asn_SPC_SL_RemoteUE_RB_Identity_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_RemoteUE_RB_Identity_r17_1[2];
extern asn_per_constraints_t asn_PER_type_SL_RemoteUE_RB_Identity_r17_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _SL_RemoteUE_RB_Identity_r17_H_ */
#include <asn_internal.h>
