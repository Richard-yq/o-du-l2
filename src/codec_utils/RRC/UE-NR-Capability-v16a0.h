/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UE_NR_Capability_v16a0_H_
#define	_UE_NR_Capability_v16a0_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Phy_Parameters_v16a0;
struct RF_Parameters_v16a0;

/* UE-NR-Capability-v16a0 */
typedef struct UE_NR_Capability_v16a0 {
	struct Phy_Parameters_v16a0	*phy_Parameters_v16a0;	/* OPTIONAL */
	struct RF_Parameters_v16a0	*rf_Parameters_v16a0;	/* OPTIONAL */
	struct UE_NR_Capability_v16a0__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UE_NR_Capability_v16a0_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UE_NR_Capability_v16a0;
extern asn_SEQUENCE_specifics_t asn_SPC_UE_NR_Capability_v16a0_specs_1;
extern asn_TYPE_member_t asn_MBR_UE_NR_Capability_v16a0_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "Phy-Parameters-v16a0.h"
#include "RF-Parameters-v16a0.h"

#endif	/* _UE_NR_Capability_v16a0_H_ */
#include <asn_internal.h>
