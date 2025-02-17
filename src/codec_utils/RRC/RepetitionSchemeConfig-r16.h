/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RepetitionSchemeConfig_r16_H_
#define	_RepetitionSchemeConfig_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RepetitionSchemeConfig_r16_PR {
	RepetitionSchemeConfig_r16_PR_NOTHING,	/* No components present */
	RepetitionSchemeConfig_r16_PR_fdm_TDM_r16,
	RepetitionSchemeConfig_r16_PR_slotBased_r16
} RepetitionSchemeConfig_r16_PR;

/* Forward declarations */
struct SetupRelease_FDM_TDM_r16;
struct SetupRelease_SlotBased_r16;

/* RepetitionSchemeConfig-r16 */
typedef struct RepetitionSchemeConfig_r16 {
	RepetitionSchemeConfig_r16_PR present;
	union RepetitionSchemeConfig_r16_u {
		struct SetupRelease_FDM_TDM_r16	*fdm_TDM_r16;
		struct SetupRelease_SlotBased_r16	*slotBased_r16;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RepetitionSchemeConfig_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RepetitionSchemeConfig_r16;
extern asn_CHOICE_specifics_t asn_SPC_RepetitionSchemeConfig_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_RepetitionSchemeConfig_r16_1[2];
extern asn_per_constraints_t asn_PER_type_RepetitionSchemeConfig_r16_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SetupRelease.h"

#endif	/* _RepetitionSchemeConfig_r16_H_ */
#include <asn_internal.h>
