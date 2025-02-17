/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UERadioAccessCapabilityInformation_H_
#define	_UERadioAccessCapabilityInformation_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NULL.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UERadioAccessCapabilityInformation__criticalExtensions_PR {
	UERadioAccessCapabilityInformation__criticalExtensions_PR_NOTHING,	/* No components present */
	UERadioAccessCapabilityInformation__criticalExtensions_PR_c1,
	UERadioAccessCapabilityInformation__criticalExtensions_PR_criticalExtensionsFuture
} UERadioAccessCapabilityInformation__criticalExtensions_PR;
typedef enum UERadioAccessCapabilityInformation__criticalExtensions__c1_PR {
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_NOTHING,	/* No components present */
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_ueRadioAccessCapabilityInformation,
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_spare7,
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_spare6,
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_spare5,
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_spare4,
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_spare3,
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_spare2,
	UERadioAccessCapabilityInformation__criticalExtensions__c1_PR_spare1
} UERadioAccessCapabilityInformation__criticalExtensions__c1_PR;

/* Forward declarations */
struct UERadioAccessCapabilityInformation_IEs;

/* UERadioAccessCapabilityInformation */
typedef struct UERadioAccessCapabilityInformation {
	struct UERadioAccessCapabilityInformation__criticalExtensions {
		UERadioAccessCapabilityInformation__criticalExtensions_PR present;
		union UERadioAccessCapabilityInformation__criticalExtensions_u {
			struct UERadioAccessCapabilityInformation__criticalExtensions__c1 {
				UERadioAccessCapabilityInformation__criticalExtensions__c1_PR present;
				union UERadioAccessCapabilityInformation__criticalExtensions__c1_u {
					struct UERadioAccessCapabilityInformation_IEs	*ueRadioAccessCapabilityInformation;
					NULL_t	 spare7;
					NULL_t	 spare6;
					NULL_t	 spare5;
					NULL_t	 spare4;
					NULL_t	 spare3;
					NULL_t	 spare2;
					NULL_t	 spare1;
				} choice;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *c1;
			struct UERadioAccessCapabilityInformation__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UERadioAccessCapabilityInformation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UERadioAccessCapabilityInformation;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "UERadioAccessCapabilityInformation-IEs.h"

#endif	/* _UERadioAccessCapabilityInformation_H_ */
#include <asn_internal.h>
