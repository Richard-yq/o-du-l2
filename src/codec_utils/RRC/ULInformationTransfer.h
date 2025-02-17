/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_ULInformationTransfer_H_
#define	_ULInformationTransfer_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ULInformationTransfer__criticalExtensions_PR {
	ULInformationTransfer__criticalExtensions_PR_NOTHING,	/* No components present */
	ULInformationTransfer__criticalExtensions_PR_ulInformationTransfer,
	ULInformationTransfer__criticalExtensions_PR_criticalExtensionsFuture
} ULInformationTransfer__criticalExtensions_PR;

/* Forward declarations */
struct ULInformationTransfer_IEs;

/* ULInformationTransfer */
typedef struct ULInformationTransfer {
	struct ULInformationTransfer__criticalExtensions {
		ULInformationTransfer__criticalExtensions_PR present;
		union ULInformationTransfer__criticalExtensions_u {
			struct ULInformationTransfer_IEs	*ulInformationTransfer;
			struct ULInformationTransfer__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ULInformationTransfer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ULInformationTransfer;
extern asn_SEQUENCE_specifics_t asn_SPC_ULInformationTransfer_specs_1;
extern asn_TYPE_member_t asn_MBR_ULInformationTransfer_1[1];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ULInformationTransfer-IEs.h"

#endif	/* _ULInformationTransfer_H_ */
#include <asn_internal.h>
