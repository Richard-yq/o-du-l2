/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./new_out -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#ifndef	_UAC_BarringPerPLMN_H_
#define	_UAC_BarringPerPLMN_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include "UAC-BarringInfoSetIndex.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UAC_BarringPerPLMN__uac_ACBarringListType_PR {
	UAC_BarringPerPLMN__uac_ACBarringListType_PR_NOTHING,	/* No components present */
	UAC_BarringPerPLMN__uac_ACBarringListType_PR_uac_ImplicitACBarringList,
	UAC_BarringPerPLMN__uac_ACBarringListType_PR_uac_ExplicitACBarringList
} UAC_BarringPerPLMN__uac_ACBarringListType_PR;

/* Forward declarations */
struct UAC_BarringPerCatList;

/* UAC-BarringPerPLMN */
typedef struct UAC_BarringPerPLMN {
	long	 plmn_IdentityIndex;
	struct UAC_BarringPerPLMN__uac_ACBarringListType {
		UAC_BarringPerPLMN__uac_ACBarringListType_PR present;
		union UAC_BarringPerPLMN__uac_ACBarringListType_u {
			struct UAC_BarringPerPLMN__uac_ACBarringListType__uac_ImplicitACBarringList {
				A_SEQUENCE_OF(UAC_BarringInfoSetIndex_t) list;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *uac_ImplicitACBarringList;
			struct UAC_BarringPerCatList	*uac_ExplicitACBarringList;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *uac_ACBarringListType;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UAC_BarringPerPLMN_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UAC_BarringPerPLMN;
extern asn_SEQUENCE_specifics_t asn_SPC_UAC_BarringPerPLMN_specs_1;
extern asn_TYPE_member_t asn_MBR_UAC_BarringPerPLMN_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _UAC_BarringPerPLMN_H_ */
#include <asn_internal.h>
