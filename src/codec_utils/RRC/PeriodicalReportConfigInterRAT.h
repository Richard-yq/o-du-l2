/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PeriodicalReportConfigInterRAT_H_
#define	_PeriodicalReportConfigInterRAT_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ReportInterval.h"
#include <NativeEnumerated.h>
#include "MeasReportQuantity.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PeriodicalReportConfigInterRAT__reportAmount {
	PeriodicalReportConfigInterRAT__reportAmount_r1	= 0,
	PeriodicalReportConfigInterRAT__reportAmount_r2	= 1,
	PeriodicalReportConfigInterRAT__reportAmount_r4	= 2,
	PeriodicalReportConfigInterRAT__reportAmount_r8	= 3,
	PeriodicalReportConfigInterRAT__reportAmount_r16	= 4,
	PeriodicalReportConfigInterRAT__reportAmount_r32	= 5,
	PeriodicalReportConfigInterRAT__reportAmount_r64	= 6,
	PeriodicalReportConfigInterRAT__reportAmount_infinity	= 7
} e_PeriodicalReportConfigInterRAT__reportAmount;
typedef enum PeriodicalReportConfigInterRAT__ext2__includeCommonLocationInfo_r16 {
	PeriodicalReportConfigInterRAT__ext2__includeCommonLocationInfo_r16_true	= 0
} e_PeriodicalReportConfigInterRAT__ext2__includeCommonLocationInfo_r16;

/* Forward declarations */
struct MeasReportQuantityUTRA_FDD_r16;
struct SetupRelease_BT_NameList_r16;
struct SetupRelease_WLAN_NameList_r16;
struct SetupRelease_Sensor_NameList_r16;
struct SL_MeasReportQuantity_r16;

/* PeriodicalReportConfigInterRAT */
typedef struct PeriodicalReportConfigInterRAT {
	ReportInterval_t	 reportInterval;
	long	 reportAmount;
	MeasReportQuantity_t	 reportQuantity;
	long	 maxReportCells;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct PeriodicalReportConfigInterRAT__ext1 {
		struct MeasReportQuantityUTRA_FDD_r16	*reportQuantityUTRA_FDD_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct PeriodicalReportConfigInterRAT__ext2 {
		long	*includeCommonLocationInfo_r16;	/* OPTIONAL */
		struct SetupRelease_BT_NameList_r16	*includeBT_Meas_r16;	/* OPTIONAL */
		struct SetupRelease_WLAN_NameList_r16	*includeWLAN_Meas_r16;	/* OPTIONAL */
		struct SetupRelease_Sensor_NameList_r16	*includeSensor_Meas_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct PeriodicalReportConfigInterRAT__ext3 {
		struct SL_MeasReportQuantity_r16	*reportQuantityRelay_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PeriodicalReportConfigInterRAT_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_reportAmount_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_includeCommonLocationInfo_r16_18;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PeriodicalReportConfigInterRAT;
extern asn_SEQUENCE_specifics_t asn_SPC_PeriodicalReportConfigInterRAT_specs_1;
extern asn_TYPE_member_t asn_MBR_PeriodicalReportConfigInterRAT_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasReportQuantityUTRA-FDD-r16.h"
#include "SetupRelease.h"
#include "SL-MeasReportQuantity-r16.h"

#endif	/* _PeriodicalReportConfigInterRAT_H_ */
#include <asn_internal.h>
