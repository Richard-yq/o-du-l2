/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SIB2_H_
#define	_SIB2_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include "RangeToBestCell.h"
#include <NativeEnumerated.h>
#include "MobilityStateParameters.h"
#include <constr_SEQUENCE.h>
#include "ReselectionThreshold.h"
#include "ReselectionThresholdQ.h"
#include "CellReselectionPriority.h"
#include "CellReselectionSubPriority.h"
#include "Q-RxLevMin.h"
#include "Q-QualMin.h"
#include "T-Reselection.h"
#include "P-Max.h"
#include <BOOLEAN.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SIB2__cellReselectionInfoCommon__q_Hyst {
	SIB2__cellReselectionInfoCommon__q_Hyst_dB0	= 0,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB1	= 1,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB2	= 2,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB3	= 3,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB4	= 4,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB5	= 5,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB6	= 6,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB8	= 7,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB10	= 8,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB12	= 9,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB14	= 10,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB16	= 11,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB18	= 12,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB20	= 13,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB22	= 14,
	SIB2__cellReselectionInfoCommon__q_Hyst_dB24	= 15
} e_SIB2__cellReselectionInfoCommon__q_Hyst;
typedef enum SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_Medium {
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_Medium_dB_6	= 0,
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_Medium_dB_4	= 1,
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_Medium_dB_2	= 2,
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_Medium_dB0	= 3
} e_SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_Medium;
typedef enum SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_High {
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_High_dB_6	= 0,
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_High_dB_4	= 1,
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_High_dB_2	= 2,
	SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_High_dB0	= 3
} e_SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF__sf_High;

/* Forward declarations */
struct ThresholdNR;
struct MultiFrequencyBandListNR_SIB;
struct SSB_MTC;
struct SS_RSSI_Measurement;
struct SSB_ToMeasure;

/* SIB2 */
typedef struct SIB2 {
	struct SIB2__cellReselectionInfoCommon {
		long	*nrofSS_BlocksToAverage;	/* OPTIONAL */
		struct ThresholdNR	*absThreshSS_BlocksConsolidation;	/* OPTIONAL */
		RangeToBestCell_t	*rangeToBestCell;	/* OPTIONAL */
		long	 q_Hyst;
		struct SIB2__cellReselectionInfoCommon__speedStateReselectionPars {
			MobilityStateParameters_t	 mobilityStateParameters;
			struct SIB2__cellReselectionInfoCommon__speedStateReselectionPars__q_HystSF {
				long	 sf_Medium;
				long	 sf_High;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} q_HystSF;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *speedStateReselectionPars;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} cellReselectionInfoCommon;
	struct SIB2__cellReselectionServingFreqInfo {
		ReselectionThreshold_t	*s_NonIntraSearchP;	/* OPTIONAL */
		ReselectionThresholdQ_t	*s_NonIntraSearchQ;	/* OPTIONAL */
		ReselectionThreshold_t	 threshServingLowP;
		ReselectionThresholdQ_t	*threshServingLowQ;	/* OPTIONAL */
		CellReselectionPriority_t	 cellReselectionPriority;
		CellReselectionSubPriority_t	*cellReselectionSubPriority;	/* OPTIONAL */
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} cellReselectionServingFreqInfo;
	struct SIB2__intraFreqCellReselectionInfo {
		Q_RxLevMin_t	 q_RxLevMin;
		Q_RxLevMin_t	*q_RxLevMinSUL;	/* OPTIONAL */
		Q_QualMin_t	*q_QualMin;	/* OPTIONAL */
		ReselectionThreshold_t	 s_IntraSearchP;
		ReselectionThresholdQ_t	*s_IntraSearchQ;	/* OPTIONAL */
		T_Reselection_t	 t_ReselectionNR;
		struct MultiFrequencyBandListNR_SIB	*frequencyBandList;	/* OPTIONAL */
		struct MultiFrequencyBandListNR_SIB	*frequencyBandListSUL;	/* OPTIONAL */
		P_Max_t	*p_Max;	/* OPTIONAL */
		struct SSB_MTC	*smtc;	/* OPTIONAL */
		struct SS_RSSI_Measurement	*ss_RSSI_Measurement;	/* OPTIONAL */
		struct SSB_ToMeasure	*ssb_ToMeasure;	/* OPTIONAL */
		BOOLEAN_t	 deriveSSB_IndexFromCell;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} intraFreqCellReselectionInfo;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SIB2_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_q_Hyst_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sf_Medium_26;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sf_High_31;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SIB2;
extern asn_SEQUENCE_specifics_t asn_SPC_SIB2_specs_1;
extern asn_TYPE_member_t asn_MBR_SIB2_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ThresholdNR.h"
#include "MultiFrequencyBandListNR-SIB.h"
#include "SSB-MTC.h"
#include "SS-RSSI-Measurement.h"
#include "SSB-ToMeasure.h"

#endif	/* _SIB2_H_ */
#include <asn_internal.h>
