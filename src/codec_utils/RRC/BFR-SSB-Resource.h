/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BFR_SSB_Resource_H_
#define	_BFR_SSB_Resource_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SSB-Index.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BFR-SSB-Resource */
typedef struct BFR_SSB_Resource {
	SSB_Index_t	 ssb;
	long	 ra_PreambleIndex;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BFR_SSB_Resource_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BFR_SSB_Resource;
extern asn_SEQUENCE_specifics_t asn_SPC_BFR_SSB_Resource_specs_1;
extern asn_TYPE_member_t asn_MBR_BFR_SSB_Resource_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _BFR_SSB_Resource_H_ */
#include <asn_internal.h>
