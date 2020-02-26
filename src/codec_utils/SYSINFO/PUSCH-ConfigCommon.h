/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./new_out -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#ifndef	_PUSCH_ConfigCommon_H_
#define	_PUSCH_ConfigCommon_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PUSCH_ConfigCommon__groupHoppingEnabledTransformPrecoding {
	PUSCH_ConfigCommon__groupHoppingEnabledTransformPrecoding_enabled	= 0
} e_PUSCH_ConfigCommon__groupHoppingEnabledTransformPrecoding;

/* Forward declarations */
struct PUSCH_TimeDomainResourceAllocationList;

/* PUSCH-ConfigCommon */
typedef struct PUSCH_ConfigCommon {
	long	*groupHoppingEnabledTransformPrecoding;	/* OPTIONAL */
	struct PUSCH_TimeDomainResourceAllocationList	*pusch_TimeDomainAllocationList;	/* OPTIONAL */
	long	*msg3_DeltaPreamble;	/* OPTIONAL */
	long	*p0_NominalWithGrant;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PUSCH_ConfigCommon_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_groupHoppingEnabledTransformPrecoding_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PUSCH_ConfigCommon;
extern asn_SEQUENCE_specifics_t asn_SPC_PUSCH_ConfigCommon_specs_1;
extern asn_TYPE_member_t asn_MBR_PUSCH_ConfigCommon_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _PUSCH_ConfigCommon_H_ */
#include <asn_internal.h>
