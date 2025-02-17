/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_RemoteUE_Config_r17_H_
#define	_SL_RemoteUE_Config_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RSRP-Range.h"
#include "Hysteresis.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct SL_ReselectionConfig_r17;

/* SL-RemoteUE-Config-r17 */
typedef struct SL_RemoteUE_Config_r17 {
	RSRP_Range_t	*threshHighRemote_r17;	/* OPTIONAL */
	Hysteresis_t	*hystMaxRemote_r17;	/* OPTIONAL */
	struct SL_ReselectionConfig_r17	*sl_ReselectionConfig_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_RemoteUE_Config_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SL_RemoteUE_Config_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_RemoteUE_Config_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_RemoteUE_Config_r17_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SL-ReselectionConfig-r17.h"

#endif	/* _SL_RemoteUE_Config_r17_H_ */
#include <asn_internal.h>
