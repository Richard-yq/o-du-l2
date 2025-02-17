/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PH_InfoMCG_H_
#define	_PH_InfoMCG_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ServCellIndex.h"
#include "PH-UplinkCarrierMCG.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PH_InfoMCG__ext1__twoSRS_PUSCH_Repetition_r17 {
	PH_InfoMCG__ext1__twoSRS_PUSCH_Repetition_r17_enabled	= 0
} e_PH_InfoMCG__ext1__twoSRS_PUSCH_Repetition_r17;

/* Forward declarations */
struct PH_UplinkCarrierMCG;

/* PH-InfoMCG */
typedef struct PH_InfoMCG {
	ServCellIndex_t	 servCellIndex;
	PH_UplinkCarrierMCG_t	 ph_Uplink;
	struct PH_UplinkCarrierMCG	*ph_SupplementaryUplink;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct PH_InfoMCG__ext1 {
		long	*twoSRS_PUSCH_Repetition_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PH_InfoMCG_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_twoSRS_PUSCH_Repetition_r17_7;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PH_InfoMCG;
extern asn_SEQUENCE_specifics_t asn_SPC_PH_InfoMCG_specs_1;
extern asn_TYPE_member_t asn_MBR_PH_InfoMCG_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PH-UplinkCarrierMCG.h"

#endif	/* _PH_InfoMCG_H_ */
#include <asn_internal.h>
