/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasurementTimingConfigurationRrc_IEs_H_
#define	_MeasurementTimingConfigurationRrc_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct MeasTimingList;

/* MeasurementTimingConfigurationRrc-IEs */
typedef struct MeasurementTimingConfigurationRrc_IEs {
	struct MeasTimingList	*measTiming;	/* OPTIONAL */
	struct MeasurementTimingConfigurationRrc_IEs__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasurementTimingConfigurationRrc_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasurementTimingConfigurationRrc_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasurementTimingConfigurationRrc_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasurementTimingConfigurationRrc_IEs_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasTimingList.h"

#endif	/* _MeasurementTimingConfigurationRrc_IEs_H_ */
#include <asn_internal.h>
