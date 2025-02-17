/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CI_ConfigurationPerServingCell_r16_H_
#define	_CI_ConfigurationPerServingCell_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ServCellIndex.h"
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16 {
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n1	= 0,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n2	= 1,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n4	= 2,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n5	= 3,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n7	= 4,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n8	= 5,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n10	= 6,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n14	= 7,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n16	= 8,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n20	= 9,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n28	= 10,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n32	= 11,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n35	= 12,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n42	= 13,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n56	= 14,
	CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16_n112	= 15
} e_CI_ConfigurationPerServingCell_r16__ci_PayloadSize_r16;
typedef enum CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeDurationForCI_r16 {
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeDurationForCI_r16_n2	= 0,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeDurationForCI_r16_n4	= 1,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeDurationForCI_r16_n7	= 2,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeDurationForCI_r16_n14	= 3
} e_CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeDurationForCI_r16;
typedef enum CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16 {
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16_n1	= 0,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16_n2	= 1,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16_n4	= 2,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16_n7	= 3,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16_n14	= 4,
	CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16_n28	= 5
} e_CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16__timeGranularityForCI_r16;
typedef enum CI_ConfigurationPerServingCell_r16__uplinkCancellationPriority_v1610 {
	CI_ConfigurationPerServingCell_r16__uplinkCancellationPriority_v1610_enabled	= 0
} e_CI_ConfigurationPerServingCell_r16__uplinkCancellationPriority_v1610;

/* CI-ConfigurationPerServingCell-r16 */
typedef struct CI_ConfigurationPerServingCell_r16 {
	ServCellIndex_t	 servingCellId;
	long	 positionInDCI_r16;
	long	*positionInDCI_ForSUL_r16;	/* OPTIONAL */
	long	 ci_PayloadSize_r16;
	struct CI_ConfigurationPerServingCell_r16__timeFrequencyRegion_r16 {
		long	*timeDurationForCI_r16;	/* OPTIONAL */
		long	 timeGranularityForCI_r16;
		long	 frequencyRegionForCI_r16;
		long	 deltaOffset_r16;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} timeFrequencyRegion_r16;
	long	*uplinkCancellationPriority_v1610;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CI_ConfigurationPerServingCell_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ci_PayloadSize_r16_5;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_timeDurationForCI_r16_23;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_timeGranularityForCI_r16_28;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_uplinkCancellationPriority_v1610_38;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CI_ConfigurationPerServingCell_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_CI_ConfigurationPerServingCell_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_CI_ConfigurationPerServingCell_r16_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _CI_ConfigurationPerServingCell_r16_H_ */
#include <asn_internal.h>
