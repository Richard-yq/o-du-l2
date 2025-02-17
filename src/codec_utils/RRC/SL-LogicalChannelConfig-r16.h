/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_LogicalChannelConfig_r16_H_
#define	_SL_LogicalChannelConfig_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include "SchedulingRequestId.h"
#include <BOOLEAN.h>
#include "SL-ConfigIndexCG-r16.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "SubcarrierSpacing.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16 {
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps0	= 0,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps8	= 1,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps16	= 2,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps32	= 3,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps64	= 4,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps128	= 5,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps256	= 6,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps512	= 7,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps1024	= 8,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps2048	= 9,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps4096	= 10,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps8192	= 11,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps16384	= 12,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps32768	= 13,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_kBps65536	= 14,
	SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16_infinity	= 15
} e_SL_LogicalChannelConfig_r16__sl_PrioritisedBitRate_r16;
typedef enum SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16 {
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms5	= 0,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms10	= 1,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms20	= 2,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms50	= 3,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms100	= 4,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms150	= 5,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms300	= 6,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms500	= 7,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_ms1000	= 8,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_spare7	= 9,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_spare6	= 10,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_spare5	= 11,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_spare4	= 12,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_spare3	= 13,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_spare2	= 14,
	SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16_spare1	= 15
} e_SL_LogicalChannelConfig_r16__sl_BucketSizeDuration_r16;
typedef enum SL_LogicalChannelConfig_r16__sl_ConfiguredGrantType1Allowed_r16 {
	SL_LogicalChannelConfig_r16__sl_ConfiguredGrantType1Allowed_r16_true	= 0
} e_SL_LogicalChannelConfig_r16__sl_ConfiguredGrantType1Allowed_r16;
typedef enum SL_LogicalChannelConfig_r16__sl_HARQ_FeedbackEnabled_r16 {
	SL_LogicalChannelConfig_r16__sl_HARQ_FeedbackEnabled_r16_enabled	= 0,
	SL_LogicalChannelConfig_r16__sl_HARQ_FeedbackEnabled_r16_disabled	= 1
} e_SL_LogicalChannelConfig_r16__sl_HARQ_FeedbackEnabled_r16;
typedef enum SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16 {
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_ms0p02	= 0,
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_ms0p04	= 1,
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_ms0p0625	= 2,
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_ms0p125	= 3,
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_ms0p25	= 4,
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_ms0p5	= 5,
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_spare2	= 6,
	SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16_spare1	= 7
} e_SL_LogicalChannelConfig_r16__sl_MaxPUSCH_Duration_r16;

/* SL-LogicalChannelConfig-r16 */
typedef struct SL_LogicalChannelConfig_r16 {
	long	 sl_Priority_r16;
	long	 sl_PrioritisedBitRate_r16;
	long	 sl_BucketSizeDuration_r16;
	long	*sl_ConfiguredGrantType1Allowed_r16;	/* OPTIONAL */
	long	*sl_HARQ_FeedbackEnabled_r16;	/* OPTIONAL */
	struct SL_LogicalChannelConfig_r16__sl_AllowedCG_List_r16 {
		A_SEQUENCE_OF(SL_ConfigIndexCG_r16_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *sl_AllowedCG_List_r16;
	struct SL_LogicalChannelConfig_r16__sl_AllowedSCS_List_r16 {
		A_SEQUENCE_OF(SubcarrierSpacing_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *sl_AllowedSCS_List_r16;
	long	*sl_MaxPUSCH_Duration_r16;	/* OPTIONAL */
	long	*sl_LogicalChannelGroup_r16;	/* OPTIONAL */
	SchedulingRequestId_t	*sl_SchedulingRequestId_r16;	/* OPTIONAL */
	BOOLEAN_t	*sl_LogicalChannelSR_DelayTimerApplied_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_LogicalChannelConfig_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_PrioritisedBitRate_r16_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_BucketSizeDuration_r16_20;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_ConfiguredGrantType1Allowed_r16_37;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_HARQ_FeedbackEnabled_r16_39;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_MaxPUSCH_Duration_r16_46;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_LogicalChannelConfig_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_LogicalChannelConfig_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_LogicalChannelConfig_r16_1[11];

#ifdef __cplusplus
}
#endif

#endif	/* _SL_LogicalChannelConfig_r16_H_ */
#include <asn_internal.h>
