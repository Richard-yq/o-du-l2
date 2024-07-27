/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_ServingCellConfig_H_
#define	_ServingCellConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include "BWP-Id.h"
#include <NativeEnumerated.h>
#include "TAG-Id.h"
#include "MeasObjectId.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ServingCellConfig__bwp_InactivityTimer {
	ServingCellConfig__bwp_InactivityTimer_ms2	= 0,
	ServingCellConfig__bwp_InactivityTimer_ms3	= 1,
	ServingCellConfig__bwp_InactivityTimer_ms4	= 2,
	ServingCellConfig__bwp_InactivityTimer_ms5	= 3,
	ServingCellConfig__bwp_InactivityTimer_ms6	= 4,
	ServingCellConfig__bwp_InactivityTimer_ms8	= 5,
	ServingCellConfig__bwp_InactivityTimer_ms10	= 6,
	ServingCellConfig__bwp_InactivityTimer_ms20	= 7,
	ServingCellConfig__bwp_InactivityTimer_ms30	= 8,
	ServingCellConfig__bwp_InactivityTimer_ms40	= 9,
	ServingCellConfig__bwp_InactivityTimer_ms50	= 10,
	ServingCellConfig__bwp_InactivityTimer_ms60	= 11,
	ServingCellConfig__bwp_InactivityTimer_ms80	= 12,
	ServingCellConfig__bwp_InactivityTimer_ms100	= 13,
	ServingCellConfig__bwp_InactivityTimer_ms200	= 14,
	ServingCellConfig__bwp_InactivityTimer_ms300	= 15,
	ServingCellConfig__bwp_InactivityTimer_ms500	= 16,
	ServingCellConfig__bwp_InactivityTimer_ms750	= 17,
	ServingCellConfig__bwp_InactivityTimer_ms1280	= 18,
	ServingCellConfig__bwp_InactivityTimer_ms1920	= 19,
	ServingCellConfig__bwp_InactivityTimer_ms2560	= 20,
	ServingCellConfig__bwp_InactivityTimer_spare10	= 21,
	ServingCellConfig__bwp_InactivityTimer_spare9	= 22,
	ServingCellConfig__bwp_InactivityTimer_spare8	= 23,
	ServingCellConfig__bwp_InactivityTimer_spare7	= 24,
	ServingCellConfig__bwp_InactivityTimer_spare6	= 25,
	ServingCellConfig__bwp_InactivityTimer_spare5	= 26,
	ServingCellConfig__bwp_InactivityTimer_spare4	= 27,
	ServingCellConfig__bwp_InactivityTimer_spare3	= 28,
	ServingCellConfig__bwp_InactivityTimer_spare2	= 29,
	ServingCellConfig__bwp_InactivityTimer_spare1	= 30
} e_ServingCellConfig__bwp_InactivityTimer;
typedef enum ServingCellConfig__sCellDeactivationTimer {
	ServingCellConfig__sCellDeactivationTimer_ms20	= 0,
	ServingCellConfig__sCellDeactivationTimer_ms40	= 1,
	ServingCellConfig__sCellDeactivationTimer_ms80	= 2,
	ServingCellConfig__sCellDeactivationTimer_ms160	= 3,
	ServingCellConfig__sCellDeactivationTimer_ms200	= 4,
	ServingCellConfig__sCellDeactivationTimer_ms240	= 5,
	ServingCellConfig__sCellDeactivationTimer_ms320	= 6,
	ServingCellConfig__sCellDeactivationTimer_ms400	= 7,
	ServingCellConfig__sCellDeactivationTimer_ms480	= 8,
	ServingCellConfig__sCellDeactivationTimer_ms520	= 9,
	ServingCellConfig__sCellDeactivationTimer_ms640	= 10,
	ServingCellConfig__sCellDeactivationTimer_ms720	= 11,
	ServingCellConfig__sCellDeactivationTimer_ms840	= 12,
	ServingCellConfig__sCellDeactivationTimer_ms1280	= 13,
	ServingCellConfig__sCellDeactivationTimer_spare2	= 14,
	ServingCellConfig__sCellDeactivationTimer_spare1	= 15
} e_ServingCellConfig__sCellDeactivationTimer;
typedef enum ServingCellConfig__ue_BeamLockFunction {
	ServingCellConfig__ue_BeamLockFunction_enabled	= 0
} e_ServingCellConfig__ue_BeamLockFunction;
typedef enum ServingCellConfig__pathlossReferenceLinking {
	ServingCellConfig__pathlossReferenceLinking_pCell	= 0,
	ServingCellConfig__pathlossReferenceLinking_sCell	= 1
} e_ServingCellConfig__pathlossReferenceLinking;

/* Forward declarations */
struct TDD_UL_DL_ConfigDedicated;
struct BWP_DownlinkDedicated;
struct UplinkConfig;
struct SetupRelease_PDCCH_ServingCellConfig;
struct SetupRelease_PDSCH_ServingCellConfig;
struct SetupRelease_CSI_MeasConfig;
struct CrossCarrierSchedulingConfig;
struct BWP_Downlink;

/* ServingCellConfig */
typedef struct ServingCellConfig {
	struct TDD_UL_DL_ConfigDedicated	*tdd_UL_DL_ConfigurationDedicated;	/* OPTIONAL */
	struct BWP_DownlinkDedicated	*initialDownlinkBWP;	/* OPTIONAL */
	struct ServingCellConfig__downlinkBWP_ToReleaseList {
		A_SEQUENCE_OF(BWP_Id_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *downlinkBWP_ToReleaseList;
	struct ServingCellConfig__downlinkBWP_ToAddModList {
		A_SEQUENCE_OF(struct BWP_Downlink) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *downlinkBWP_ToAddModList;
	BWP_Id_t	*firstActiveDownlinkBWP_Id;	/* OPTIONAL */
	long	*bwp_InactivityTimer;	/* OPTIONAL */
	BWP_Id_t	*defaultDownlinkBWP_Id;	/* OPTIONAL */
	struct UplinkConfig	*uplinkConfig;	/* OPTIONAL */
	struct UplinkConfig	*supplementaryUplink;	/* OPTIONAL */
	struct SetupRelease_PDCCH_ServingCellConfig	*pdcch_ServingCellConfig;	/* OPTIONAL */
	struct SetupRelease_PDSCH_ServingCellConfig	*pdsch_ServingCellConfig;	/* OPTIONAL */
	struct SetupRelease_CSI_MeasConfig	*csi_MeasConfig;	/* OPTIONAL */
	long	*sCellDeactivationTimer;	/* OPTIONAL */
	struct CrossCarrierSchedulingConfig	*crossCarrierSchedulingConfig;	/* OPTIONAL */
	TAG_Id_t	 tag_Id;
	long	*ue_BeamLockFunction;	/* OPTIONAL */
	long	*pathlossReferenceLinking;	/* OPTIONAL */
	MeasObjectId_t	*servingCellMO;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ServingCellConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_bwp_InactivityTimer_9;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sCellDeactivationTimer_47;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ue_BeamLockFunction_66;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pathlossReferenceLinking_68;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_ServingCellConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_ServingCellConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_ServingCellConfig_1[18];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "TDD-UL-DL-ConfigDedicated.h"
#include "BWP-DownlinkDedicated.h"
#include "UplinkConfig.h"
#include "SetupRelease.h"
#include "CrossCarrierSchedulingConfig.h"
#include "BWP-Downlink.h"

#endif	/* _ServingCellConfig_H_ */
#include <asn_internal.h>
