/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PUCCH_Config_H_
#define	_PUCCH_Config_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "PUCCH-ResourceSetId.h"
#include "PUCCH-ResourceId.h"
#include "SchedulingRequestResourceId.h"
#include <NativeInteger.h>
#include "PUCCH-SpatialRelationInfoId.h"
#include <NativeEnumerated.h>
#include <constr_CHOICE.h>
#include "PUCCH-SpatialRelationInfoId-r16.h"
#include "PUCCH-ResourceGroupId-r16.h"
#include <constr_SEQUENCE.h>
#include "PUCCH-PowerControlSetInfoId-r17.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PUCCH_Config__ext1__subslotLengthForPUCCH_r16_PR {
	PUCCH_Config__ext1__subslotLengthForPUCCH_r16_PR_NOTHING,	/* No components present */
	PUCCH_Config__ext1__subslotLengthForPUCCH_r16_PR_normalCP_r16,
	PUCCH_Config__ext1__subslotLengthForPUCCH_r16_PR_extendedCP_r16
} PUCCH_Config__ext1__subslotLengthForPUCCH_r16_PR;
typedef enum PUCCH_Config__ext1__subslotLengthForPUCCH_r16__normalCP_r16 {
	PUCCH_Config__ext1__subslotLengthForPUCCH_r16__normalCP_r16_n2	= 0,
	PUCCH_Config__ext1__subslotLengthForPUCCH_r16__normalCP_r16_n7	= 1
} e_PUCCH_Config__ext1__subslotLengthForPUCCH_r16__normalCP_r16;
typedef enum PUCCH_Config__ext1__subslotLengthForPUCCH_r16__extendedCP_r16 {
	PUCCH_Config__ext1__subslotLengthForPUCCH_r16__extendedCP_r16_n2	= 0,
	PUCCH_Config__ext1__subslotLengthForPUCCH_r16__extendedCP_r16_n6	= 1
} e_PUCCH_Config__ext1__subslotLengthForPUCCH_r16__extendedCP_r16;
typedef enum PUCCH_Config__ext1__dmrs_UplinkTransformPrecodingPUCCH_r16 {
	PUCCH_Config__ext1__dmrs_UplinkTransformPrecodingPUCCH_r16_enabled	= 0
} e_PUCCH_Config__ext1__dmrs_UplinkTransformPrecodingPUCCH_r16;
typedef enum PUCCH_Config__ext2__mappingPattern_r17 {
	PUCCH_Config__ext2__mappingPattern_r17_cyclicMapping	= 0,
	PUCCH_Config__ext2__mappingPattern_r17_sequentialMapping	= 1
} e_PUCCH_Config__ext2__mappingPattern_r17;
typedef enum PUCCH_Config__ext2__secondTPCFieldDCI_1_1_r17 {
	PUCCH_Config__ext2__secondTPCFieldDCI_1_1_r17_enabled	= 0
} e_PUCCH_Config__ext2__secondTPCFieldDCI_1_1_r17;
typedef enum PUCCH_Config__ext2__secondTPCFieldDCI_1_2_r17 {
	PUCCH_Config__ext2__secondTPCFieldDCI_1_2_r17_enabled	= 0
} e_PUCCH_Config__ext2__secondTPCFieldDCI_1_2_r17;

/* Forward declarations */
struct SetupRelease_PUCCH_FormatConfig;
struct PUCCH_PowerControl;
struct PUCCH_ResourceSet;
struct PUCCH_Resource;
struct SchedulingRequestResourceConfig;
struct PUCCH_SpatialRelationInfo;
struct SetupRelease_DL_DataToUL_ACK_r16;
struct SetupRelease_UL_AccessConfigListDCI_1_1_r16;
struct SetupRelease_DL_DataToUL_ACK_DCI_1_2_r16;
struct SetupRelease_SPS_PUCCH_AN_List_r16;
struct PUCCH_ResourceExt_v1610;
struct PUCCH_SpatialRelationInfoExt_r16;
struct PUCCH_ResourceGroup_r16;
struct SchedulingRequestResourceConfigExt_v1610;
struct SetupRelease_PUCCH_FormatConfigExt_r17;
struct SetupRelease_UL_AccessConfigListDCI_1_2_r17;
struct SetupRelease_DL_DataToUL_ACK_r17;
struct SetupRelease_DL_DataToUL_ACK_DCI_1_2_r17;
struct SetupRelease_UL_AccessConfigListDCI_1_1_r17;
struct SetupRelease_DMRS_BundlingPUCCH_Config_r17;
struct SetupRelease_DL_DataToUL_ACK_v1700;
struct SetupRelease_DL_DataToUL_ACK_MulticastDCI_Format4_1_r17;
struct PUCCH_PowerControlSetInfo_r17;
struct SchedulingRequestResourceConfigExt_v1700;

/* PUCCH-Config */
typedef struct PUCCH_Config {
	struct PUCCH_Config__resourceSetToAddModList {
		A_SEQUENCE_OF(struct PUCCH_ResourceSet) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *resourceSetToAddModList;
	struct PUCCH_Config__resourceSetToReleaseList {
		A_SEQUENCE_OF(PUCCH_ResourceSetId_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *resourceSetToReleaseList;
	struct PUCCH_Config__resourceToAddModList {
		A_SEQUENCE_OF(struct PUCCH_Resource) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *resourceToAddModList;
	struct PUCCH_Config__resourceToReleaseList {
		A_SEQUENCE_OF(PUCCH_ResourceId_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *resourceToReleaseList;
	struct SetupRelease_PUCCH_FormatConfig	*format1;	/* OPTIONAL */
	struct SetupRelease_PUCCH_FormatConfig	*format2;	/* OPTIONAL */
	struct SetupRelease_PUCCH_FormatConfig	*format3;	/* OPTIONAL */
	struct SetupRelease_PUCCH_FormatConfig	*format4;	/* OPTIONAL */
	struct PUCCH_Config__schedulingRequestResourceToAddModList {
		A_SEQUENCE_OF(struct SchedulingRequestResourceConfig) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *schedulingRequestResourceToAddModList;
	struct PUCCH_Config__schedulingRequestResourceToReleaseList {
		A_SEQUENCE_OF(SchedulingRequestResourceId_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *schedulingRequestResourceToReleaseList;
	struct PUCCH_Config__multi_CSI_PUCCH_ResourceList {
		A_SEQUENCE_OF(PUCCH_ResourceId_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *multi_CSI_PUCCH_ResourceList;
	struct PUCCH_Config__dl_DataToUL_ACK {
		A_SEQUENCE_OF(long) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *dl_DataToUL_ACK;
	struct PUCCH_Config__spatialRelationInfoToAddModList {
		A_SEQUENCE_OF(struct PUCCH_SpatialRelationInfo) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *spatialRelationInfoToAddModList;
	struct PUCCH_Config__spatialRelationInfoToReleaseList {
		A_SEQUENCE_OF(PUCCH_SpatialRelationInfoId_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *spatialRelationInfoToReleaseList;
	struct PUCCH_PowerControl	*pucch_PowerControl;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct PUCCH_Config__ext1 {
		struct PUCCH_Config__ext1__resourceToAddModListExt_v1610 {
			A_SEQUENCE_OF(struct PUCCH_ResourceExt_v1610) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *resourceToAddModListExt_v1610;
		struct SetupRelease_DL_DataToUL_ACK_r16	*dl_DataToUL_ACK_r16;	/* OPTIONAL */
		struct SetupRelease_UL_AccessConfigListDCI_1_1_r16	*ul_AccessConfigListDCI_1_1_r16;	/* OPTIONAL */
		struct PUCCH_Config__ext1__subslotLengthForPUCCH_r16 {
			PUCCH_Config__ext1__subslotLengthForPUCCH_r16_PR present;
			union PUCCH_Config__ext1__subslotLengthForPUCCH_r16_u {
				long	 normalCP_r16;
				long	 extendedCP_r16;
			} choice;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *subslotLengthForPUCCH_r16;
		struct SetupRelease_DL_DataToUL_ACK_DCI_1_2_r16	*dl_DataToUL_ACK_DCI_1_2_r16;	/* OPTIONAL */
		long	*numberOfBitsForPUCCH_ResourceIndicatorDCI_1_2_r16;	/* OPTIONAL */
		long	*dmrs_UplinkTransformPrecodingPUCCH_r16;	/* OPTIONAL */
		struct PUCCH_Config__ext1__spatialRelationInfoToAddModListSizeExt_v1610 {
			A_SEQUENCE_OF(struct PUCCH_SpatialRelationInfo) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *spatialRelationInfoToAddModListSizeExt_v1610;
		struct PUCCH_Config__ext1__spatialRelationInfoToReleaseListSizeExt_v1610 {
			A_SEQUENCE_OF(PUCCH_SpatialRelationInfoId_t) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *spatialRelationInfoToReleaseListSizeExt_v1610;
		struct PUCCH_Config__ext1__spatialRelationInfoToAddModListExt_v1610 {
			A_SEQUENCE_OF(struct PUCCH_SpatialRelationInfoExt_r16) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *spatialRelationInfoToAddModListExt_v1610;
		struct PUCCH_Config__ext1__spatialRelationInfoToReleaseListExt_v1610 {
			A_SEQUENCE_OF(PUCCH_SpatialRelationInfoId_r16_t) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *spatialRelationInfoToReleaseListExt_v1610;
		struct PUCCH_Config__ext1__resourceGroupToAddModList_r16 {
			A_SEQUENCE_OF(struct PUCCH_ResourceGroup_r16) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *resourceGroupToAddModList_r16;
		struct PUCCH_Config__ext1__resourceGroupToReleaseList_r16 {
			A_SEQUENCE_OF(PUCCH_ResourceGroupId_r16_t) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *resourceGroupToReleaseList_r16;
		struct SetupRelease_SPS_PUCCH_AN_List_r16	*sps_PUCCH_AN_List_r16;	/* OPTIONAL */
		struct PUCCH_Config__ext1__schedulingRequestResourceToAddModListExt_v1610 {
			A_SEQUENCE_OF(struct SchedulingRequestResourceConfigExt_v1610) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *schedulingRequestResourceToAddModListExt_v1610;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct PUCCH_Config__ext2 {
		struct SetupRelease_PUCCH_FormatConfig	*format0_r17;	/* OPTIONAL */
		struct SetupRelease_PUCCH_FormatConfigExt_r17	*format2Ext_r17;	/* OPTIONAL */
		struct SetupRelease_PUCCH_FormatConfigExt_r17	*format3Ext_r17;	/* OPTIONAL */
		struct SetupRelease_PUCCH_FormatConfigExt_r17	*format4Ext_r17;	/* OPTIONAL */
		struct SetupRelease_UL_AccessConfigListDCI_1_2_r17	*ul_AccessConfigListDCI_1_2_r17;	/* OPTIONAL */
		long	*mappingPattern_r17;	/* OPTIONAL */
		struct PUCCH_Config__ext2__powerControlSetInfoToAddModList_r17 {
			A_SEQUENCE_OF(struct PUCCH_PowerControlSetInfo_r17) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *powerControlSetInfoToAddModList_r17;
		struct PUCCH_Config__ext2__powerControlSetInfoToReleaseList_r17 {
			A_SEQUENCE_OF(PUCCH_PowerControlSetInfoId_r17_t) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *powerControlSetInfoToReleaseList_r17;
		long	*secondTPCFieldDCI_1_1_r17;	/* OPTIONAL */
		long	*secondTPCFieldDCI_1_2_r17;	/* OPTIONAL */
		struct SetupRelease_DL_DataToUL_ACK_r17	*dl_DataToUL_ACK_r17;	/* OPTIONAL */
		struct SetupRelease_DL_DataToUL_ACK_DCI_1_2_r17	*dl_DataToUL_ACK_DCI_1_2_r17;	/* OPTIONAL */
		struct SetupRelease_UL_AccessConfigListDCI_1_1_r17	*ul_AccessConfigListDCI_1_1_r17;	/* OPTIONAL */
		struct PUCCH_Config__ext2__schedulingRequestResourceToAddModListExt_v1700 {
			A_SEQUENCE_OF(struct SchedulingRequestResourceConfigExt_v1700) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *schedulingRequestResourceToAddModListExt_v1700;
		struct SetupRelease_DMRS_BundlingPUCCH_Config_r17	*dmrs_BundlingPUCCH_Config_r17;	/* OPTIONAL */
		struct SetupRelease_DL_DataToUL_ACK_v1700	*dl_DataToUL_ACK_v1700;	/* OPTIONAL */
		struct SetupRelease_DL_DataToUL_ACK_MulticastDCI_Format4_1_r17	*dl_DataToUL_ACK_MulticastDCI_Format4_1_r17;	/* OPTIONAL */
		struct SetupRelease_SPS_PUCCH_AN_List_r16	*sps_PUCCH_AN_ListMulticast_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PUCCH_Config_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_normalCP_r16_34;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_extendedCP_r16_37;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_UplinkTransformPrecodingPUCCH_r16_42;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mappingPattern_r17_65;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_secondTPCFieldDCI_1_1_r17_72;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_secondTPCFieldDCI_1_2_r17_74;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PUCCH_Config;
extern asn_SEQUENCE_specifics_t asn_SPC_PUCCH_Config_specs_1;
extern asn_TYPE_member_t asn_MBR_PUCCH_Config_1[17];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SetupRelease.h"
#include "PUCCH-PowerControl.h"
#include "PUCCH-ResourceSet.h"
#include "PUCCH-Resource.h"
#include "SchedulingRequestResourceConfig.h"
#include "PUCCH-SpatialRelationInfo.h"
#include "PUCCH-ResourceExt-v1610.h"
#include "PUCCH-SpatialRelationInfoExt-r16.h"
#include "PUCCH-ResourceGroup-r16.h"
#include "SchedulingRequestResourceConfigExt-v1610.h"
#include "PUCCH-PowerControlSetInfo-r17.h"
#include "SchedulingRequestResourceConfigExt-v1700.h"

#endif	/* _PUCCH_Config_H_ */
#include <asn_internal.h>
