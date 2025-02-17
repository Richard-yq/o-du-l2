/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SchedulingInfo2_r17_H_
#define	_SchedulingInfo2_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include "SIB-Mapping-v1700.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SchedulingInfo2_r17__si_BroadcastStatus_r17 {
	SchedulingInfo2_r17__si_BroadcastStatus_r17_broadcasting	= 0,
	SchedulingInfo2_r17__si_BroadcastStatus_r17_notBroadcasting	= 1
} e_SchedulingInfo2_r17__si_BroadcastStatus_r17;
typedef enum SchedulingInfo2_r17__si_Periodicity_r17 {
	SchedulingInfo2_r17__si_Periodicity_r17_rf8	= 0,
	SchedulingInfo2_r17__si_Periodicity_r17_rf16	= 1,
	SchedulingInfo2_r17__si_Periodicity_r17_rf32	= 2,
	SchedulingInfo2_r17__si_Periodicity_r17_rf64	= 3,
	SchedulingInfo2_r17__si_Periodicity_r17_rf128	= 4,
	SchedulingInfo2_r17__si_Periodicity_r17_rf256	= 5,
	SchedulingInfo2_r17__si_Periodicity_r17_rf512	= 6
} e_SchedulingInfo2_r17__si_Periodicity_r17;

/* SchedulingInfo2-r17 */
typedef struct SchedulingInfo2_r17 {
	long	 si_BroadcastStatus_r17;
	long	 si_WindowPosition_r17;
	long	 si_Periodicity_r17;
	SIB_Mapping_v1700_t	 sib_MappingInfo_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SchedulingInfo2_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_si_BroadcastStatus_r17_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_si_Periodicity_r17_6;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SchedulingInfo2_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_SchedulingInfo2_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SchedulingInfo2_r17_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _SchedulingInfo2_r17_H_ */
#include <asn_internal.h>
