/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SIB17_IEs_r17_H_
#define	_SIB17_IEs_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <OCTET_STRING.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SIB17_IEs_r17__validityDuration_r17 {
	SIB17_IEs_r17__validityDuration_r17_t1	= 0,
	SIB17_IEs_r17__validityDuration_r17_t2	= 1,
	SIB17_IEs_r17__validityDuration_r17_t4	= 2,
	SIB17_IEs_r17__validityDuration_r17_t8	= 3,
	SIB17_IEs_r17__validityDuration_r17_t16	= 4,
	SIB17_IEs_r17__validityDuration_r17_t32	= 5,
	SIB17_IEs_r17__validityDuration_r17_t64	= 6,
	SIB17_IEs_r17__validityDuration_r17_t128	= 7,
	SIB17_IEs_r17__validityDuration_r17_t256	= 8,
	SIB17_IEs_r17__validityDuration_r17_t512	= 9,
	SIB17_IEs_r17__validityDuration_r17_infinity	= 10,
	SIB17_IEs_r17__validityDuration_r17_spare5	= 11,
	SIB17_IEs_r17__validityDuration_r17_spare4	= 12,
	SIB17_IEs_r17__validityDuration_r17_spare3	= 13,
	SIB17_IEs_r17__validityDuration_r17_spare2	= 14,
	SIB17_IEs_r17__validityDuration_r17_spare1	= 15
} e_SIB17_IEs_r17__validityDuration_r17;

/* Forward declarations */
struct TRS_ResourceSet_r17;

/* SIB17-IEs-r17 */
typedef struct SIB17_IEs_r17 {
	struct SIB17_IEs_r17__trs_ResourceSetConfig_r17 {
		A_SEQUENCE_OF(struct TRS_ResourceSet_r17) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} trs_ResourceSetConfig_r17;
	long	*validityDuration_r17;	/* OPTIONAL */
	OCTET_STRING_t	*lateNonCriticalExtension;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SIB17_IEs_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_validityDuration_r17_4;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SIB17_IEs_r17;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "TRS-ResourceSet-r17.h"

#endif	/* _SIB17_IEs_r17_H_ */
#include <asn_internal.h>
