/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CSI_AperiodicTriggerState_H_
#define	_CSI_AperiodicTriggerState_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CSI_AperiodicTriggerState__ext1__ap_CSI_MultiplexingMode_r17 {
	CSI_AperiodicTriggerState__ext1__ap_CSI_MultiplexingMode_r17_enabled	= 0
} e_CSI_AperiodicTriggerState__ext1__ap_CSI_MultiplexingMode_r17;

/* Forward declarations */
struct CSI_AssociatedReportConfigInfo;

/* CSI-AperiodicTriggerState */
typedef struct CSI_AperiodicTriggerState {
	struct CSI_AperiodicTriggerState__associatedReportConfigInfoList {
		A_SEQUENCE_OF(struct CSI_AssociatedReportConfigInfo) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} associatedReportConfigInfoList;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct CSI_AperiodicTriggerState__ext1 {
		long	*ap_CSI_MultiplexingMode_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CSI_AperiodicTriggerState_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ap_CSI_MultiplexingMode_r17_6;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CSI_AperiodicTriggerState;
extern asn_SEQUENCE_specifics_t asn_SPC_CSI_AperiodicTriggerState_specs_1;
extern asn_TYPE_member_t asn_MBR_CSI_AperiodicTriggerState_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CSI-AssociatedReportConfigInfo.h"

#endif	/* _CSI_AperiodicTriggerState_H_ */
#include <asn_internal.h>
