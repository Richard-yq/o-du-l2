/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_DLInformationTransfer_H_
#define	_DLInformationTransfer_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RRC-TransactionIdentifier.h"
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DLInformationTransfer__criticalExtensions_PR {
	DLInformationTransfer__criticalExtensions_PR_NOTHING,	/* No components present */
	DLInformationTransfer__criticalExtensions_PR_dlInformationTransfer,
	DLInformationTransfer__criticalExtensions_PR_criticalExtensionsFuture
} DLInformationTransfer__criticalExtensions_PR;

/* Forward declarations */
struct DLInformationTransfer_IEs;

/* DLInformationTransfer */
typedef struct DLInformationTransfer {
	RRC_TransactionIdentifier_t	 rrc_TransactionIdentifier;
	struct DLInformationTransfer__criticalExtensions {
		DLInformationTransfer__criticalExtensions_PR present;
		union DLInformationTransfer__criticalExtensions_u {
			struct DLInformationTransfer_IEs	*dlInformationTransfer;
			struct DLInformationTransfer__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DLInformationTransfer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DLInformationTransfer;
extern asn_SEQUENCE_specifics_t asn_SPC_DLInformationTransfer_specs_1;
extern asn_TYPE_member_t asn_MBR_DLInformationTransfer_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "DLInformationTransfer-IEs.h"

#endif	/* _DLInformationTransfer_H_ */
#include <asn_internal.h>
