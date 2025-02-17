/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PeriodicalReportConfig_H_
#define	_PeriodicalReportConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include "NR-RS-Type.h"
#include "ReportInterval.h"
#include <NativeEnumerated.h>
#include "MeasReportQuantity.h"
#include <NativeInteger.h>
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PeriodicalReportConfig__reportAmount {
	PeriodicalReportConfig__reportAmount_r1	= 0,
	PeriodicalReportConfig__reportAmount_r2	= 1,
	PeriodicalReportConfig__reportAmount_r4	= 2,
	PeriodicalReportConfig__reportAmount_r8	= 3,
	PeriodicalReportConfig__reportAmount_r16	= 4,
	PeriodicalReportConfig__reportAmount_r32	= 5,
	PeriodicalReportConfig__reportAmount_r64	= 6,
	PeriodicalReportConfig__reportAmount_infinity	= 7
} e_PeriodicalReportConfig__reportAmount;
typedef enum PeriodicalReportConfig__ext1__includeCommonLocationInfo_r16 {
	PeriodicalReportConfig__ext1__includeCommonLocationInfo_r16_true	= 0
} e_PeriodicalReportConfig__ext1__includeCommonLocationInfo_r16;
typedef enum PeriodicalReportConfig__ext1__reportAddNeighMeas_r16 {
	PeriodicalReportConfig__ext1__reportAddNeighMeas_r16_setup	= 0
} e_PeriodicalReportConfig__ext1__reportAddNeighMeas_r16;
typedef enum PeriodicalReportConfig__ext2__coarseLocationRequest_r17 {
	PeriodicalReportConfig__ext2__coarseLocationRequest_r17_true	= 0
} e_PeriodicalReportConfig__ext2__coarseLocationRequest_r17;

/* Forward declarations */
struct MeasReportQuantity;
struct MeasRSSI_ReportConfig_r16;
struct SetupRelease_BT_NameList_r16;
struct SetupRelease_WLAN_NameList_r16;
struct SetupRelease_Sensor_NameList_r16;
struct SetupRelease_UL_DelayValueConfig_r16;
struct SetupRelease_UL_ExcessDelayConfig_r17;
struct SL_MeasReportQuantity_r16;

/* PeriodicalReportConfig */
typedef struct PeriodicalReportConfig {
	NR_RS_Type_t	 rsType;
	ReportInterval_t	 reportInterval;
	long	 reportAmount;
	MeasReportQuantity_t	 reportQuantityCell;
	long	 maxReportCells;
	struct MeasReportQuantity	*reportQuantityRS_Indexes;	/* OPTIONAL */
	long	*maxNrofRS_IndexesToReport;	/* OPTIONAL */
	BOOLEAN_t	 includeBeamMeasurements;
	BOOLEAN_t	 useAllowedCellList;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct PeriodicalReportConfig__ext1 {
		struct MeasRSSI_ReportConfig_r16	*measRSSI_ReportConfig_r16;	/* OPTIONAL */
		long	*includeCommonLocationInfo_r16;	/* OPTIONAL */
		struct SetupRelease_BT_NameList_r16	*includeBT_Meas_r16;	/* OPTIONAL */
		struct SetupRelease_WLAN_NameList_r16	*includeWLAN_Meas_r16;	/* OPTIONAL */
		struct SetupRelease_Sensor_NameList_r16	*includeSensor_Meas_r16;	/* OPTIONAL */
		struct SetupRelease_UL_DelayValueConfig_r16	*ul_DelayValueConfig_r16;	/* OPTIONAL */
		long	*reportAddNeighMeas_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct PeriodicalReportConfig__ext2 {
		struct SetupRelease_UL_ExcessDelayConfig_r17	*ul_ExcessDelayConfig_r17;	/* OPTIONAL */
		long	*coarseLocationRequest_r17;	/* OPTIONAL */
		struct SL_MeasReportQuantity_r16	*reportQuantityRelay_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PeriodicalReportConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_reportAmount_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_includeCommonLocationInfo_r16_22;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_reportAddNeighMeas_r16_28;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_coarseLocationRequest_r17_32;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PeriodicalReportConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_PeriodicalReportConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_PeriodicalReportConfig_1[11];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasReportQuantity.h"
#include "MeasRSSI-ReportConfig-r16.h"
#include "SetupRelease.h"
#include "SL-MeasReportQuantity-r16.h"

#endif	/* _PeriodicalReportConfig_H_ */
#include <asn_internal.h>
