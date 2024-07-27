/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SSB_ConfigMobility_H_
#define	_SSB_ConfigMobility_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct SetupRelease_SSB_ToMeasure;
struct SS_RSSI_Measurement;

/* SSB-ConfigMobility */
typedef struct SSB_ConfigMobility {
	struct SetupRelease_SSB_ToMeasure	*ssb_ToMeasure;	/* OPTIONAL */
	BOOLEAN_t	 deriveSSB_IndexFromCell;
	struct SS_RSSI_Measurement	*ss_RSSI_Measurement;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SSB_ConfigMobility_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SSB_ConfigMobility;
extern asn_SEQUENCE_specifics_t asn_SPC_SSB_ConfigMobility_specs_1;
extern asn_TYPE_member_t asn_MBR_SSB_ConfigMobility_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SetupRelease.h"
#include "SS-RSSI-Measurement.h"

#endif	/* _SSB_ConfigMobility_H_ */
#include <asn_internal.h>
