/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SRS_PosResourceAP_r16_H_
#define	_SRS_PosResourceAP_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16 {
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16_n1	= 0,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16_n2	= 1,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16_n4	= 2,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16_n8	= 3,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16_n16	= 4,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16_n32	= 5,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16_n64	= 6
} e_SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_r16;
typedef enum SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16 {
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n1	= 0,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n2	= 1,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n3	= 2,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n4	= 3,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n5	= 4,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n6	= 5,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n8	= 6,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n10	= 7,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n12	= 8,
	SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_n14	= 9
} e_SRS_PosResourceAP_r16__maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16;

/* SRS-PosResourceAP-r16 */
typedef struct SRS_PosResourceAP_r16 {
	long	 maxNumberAP_SRS_PosResourcesPerBWP_r16;
	long	 maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SRS_PosResourceAP_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberAP_SRS_PosResourcesPerBWP_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberAP_SRS_PosResourcesPerBWP_PerSlot_r16_10;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SRS_PosResourceAP_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SRS_PosResourceAP_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SRS_PosResourceAP_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SRS_PosResourceAP_r16_H_ */
#include <asn_internal.h>
