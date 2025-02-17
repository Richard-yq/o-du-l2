/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PC5-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RRCReconfigurationCompleteSidelink_v1720_IEs_H_
#define	_RRCReconfigurationCompleteSidelink_v1720_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RRCReconfigurationCompleteSidelink_v1720_IEs__sl_DRX_ConfigReject_v1720 {
	RRCReconfigurationCompleteSidelink_v1720_IEs__sl_DRX_ConfigReject_v1720_true	= 0
} e_RRCReconfigurationCompleteSidelink_v1720_IEs__sl_DRX_ConfigReject_v1720;

/* RRCReconfigurationCompleteSidelink-v1720-IEs */
typedef struct RRCReconfigurationCompleteSidelink_v1720_IEs {
	long	*sl_DRX_ConfigReject_v1720;	/* OPTIONAL */
	struct RRCReconfigurationCompleteSidelink_v1720_IEs__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RRCReconfigurationCompleteSidelink_v1720_IEs_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_DRX_ConfigReject_v1720_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_RRCReconfigurationCompleteSidelink_v1720_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_RRCReconfigurationCompleteSidelink_v1720_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_RRCReconfigurationCompleteSidelink_v1720_IEs_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _RRCReconfigurationCompleteSidelink_v1720_IEs_H_ */
#include <asn_internal.h>
