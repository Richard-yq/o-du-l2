/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PC5-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UECapabilityInformationSidelink_H_
#define	_UECapabilityInformationSidelink_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RRC-TransactionIdentifier.h"
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UECapabilityInformationSidelink__criticalExtensions_PR {
	UECapabilityInformationSidelink__criticalExtensions_PR_NOTHING,	/* No components present */
	UECapabilityInformationSidelink__criticalExtensions_PR_ueCapabilityInformationSidelink_r16,
	UECapabilityInformationSidelink__criticalExtensions_PR_criticalExtensionsFuture
} UECapabilityInformationSidelink__criticalExtensions_PR;

/* Forward declarations */
struct UECapabilityInformationSidelink_r16_IEs;

/* UECapabilityInformationSidelink */
typedef struct UECapabilityInformationSidelink {
	RRC_TransactionIdentifier_t	 rrc_TransactionIdentifier_r16;
	struct UECapabilityInformationSidelink__criticalExtensions {
		UECapabilityInformationSidelink__criticalExtensions_PR present;
		union UECapabilityInformationSidelink__criticalExtensions_u {
			struct UECapabilityInformationSidelink_r16_IEs	*ueCapabilityInformationSidelink_r16;
			struct UECapabilityInformationSidelink__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UECapabilityInformationSidelink_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UECapabilityInformationSidelink;
extern asn_SEQUENCE_specifics_t asn_SPC_UECapabilityInformationSidelink_specs_1;
extern asn_TYPE_member_t asn_MBR_UECapabilityInformationSidelink_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "UECapabilityInformationSidelink-r16-IEs.h"

#endif	/* _UECapabilityInformationSidelink_H_ */
#include <asn_internal.h>
