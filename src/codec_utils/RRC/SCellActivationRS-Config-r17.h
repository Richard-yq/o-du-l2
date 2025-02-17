/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SCellActivationRS_Config_r17_H_
#define	_SCellActivationRS_Config_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SCellActivationRS-ConfigId-r17.h"
#include "NZP-CSI-RS-ResourceSetId.h"
#include <NativeInteger.h>
#include "TCI-StateId.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SCellActivationRS-Config-r17 */
typedef struct SCellActivationRS_Config_r17 {
	SCellActivationRS_ConfigId_r17_t	 scellActivationRS_Id_r17;
	NZP_CSI_RS_ResourceSetId_t	 resourceSet_r17;
	long	*gapBetweenBursts_r17;	/* OPTIONAL */
	TCI_StateId_t	 qcl_Info_r17;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SCellActivationRS_Config_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SCellActivationRS_Config_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_SCellActivationRS_Config_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SCellActivationRS_Config_r17_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _SCellActivationRS_Config_r17_H_ */
#include <asn_internal.h>
