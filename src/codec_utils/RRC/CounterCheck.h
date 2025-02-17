/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CounterCheck_H_
#define	_CounterCheck_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RRC-TransactionIdentifier.h"
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CounterCheck__criticalExtensions_PR {
	CounterCheck__criticalExtensions_PR_NOTHING,	/* No components present */
	CounterCheck__criticalExtensions_PR_counterCheck,
	CounterCheck__criticalExtensions_PR_criticalExtensionsFuture
} CounterCheck__criticalExtensions_PR;

/* Forward declarations */
struct CounterCheck_IEs;

/* CounterCheck */
typedef struct CounterCheck {
	RRC_TransactionIdentifier_t	 rrc_TransactionIdentifier;
	struct CounterCheck__criticalExtensions {
		CounterCheck__criticalExtensions_PR present;
		union CounterCheck__criticalExtensions_u {
			struct CounterCheck_IEs	*counterCheck;
			struct CounterCheck__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CounterCheck_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CounterCheck;
extern asn_SEQUENCE_specifics_t asn_SPC_CounterCheck_specs_1;
extern asn_TYPE_member_t asn_MBR_CounterCheck_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CounterCheck-IEs.h"

#endif	/* _CounterCheck_H_ */
#include <asn_internal.h>
