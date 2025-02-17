/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
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
#include "SSB-PositionQCL-Relation-r16.h"
#include "SSB-PositionQCL-Relation-r17.h"

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
typedef enum SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16 {
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_dB3	= 0,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_dB6	= 1,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_dB9	= 2,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_dB12	= 3,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_dB15	= 4,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_spare3	= 5,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_spare2	= 6,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16_spare1	= 7
} e_SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__s_SearchDeltaP_r16;
typedef enum SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16 {
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s5	= 0,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s10	= 1,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s20	= 2,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s30	= 3,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s60	= 4,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s120	= 5,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s180	= 6,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s240	= 7,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_s300	= 8,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_spare7	= 9,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_spare6	= 10,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_spare5	= 11,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_spare4	= 12,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_spare3	= 13,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_spare2	= 14,
	SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16_spare1	= 15
} e_SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16__t_SearchDeltaP_r16;
typedef enum SIB2__ext1__relaxedMeasurement_r16__combineRelaxedMeasCondition_r16 {
	SIB2__ext1__relaxedMeasurement_r16__combineRelaxedMeasCondition_r16_true	= 0
} e_SIB2__ext1__relaxedMeasurement_r16__combineRelaxedMeasCondition_r16;
typedef enum SIB2__ext1__relaxedMeasurement_r16__highPriorityMeasRelax_r16 {
	SIB2__ext1__relaxedMeasurement_r16__highPriorityMeasRelax_r16_true	= 0
} e_SIB2__ext1__relaxedMeasurement_r16__highPriorityMeasRelax_r16;
typedef enum SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17 {
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_dB2	= 0,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_dB3	= 1,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_dB6	= 2,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_dB9	= 3,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_dB12	= 4,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_dB15	= 5,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_spare2	= 6,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17_spare1	= 7
} e_SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__s_SearchDeltaP_Stationary_r17;
typedef enum SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17 {
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s5	= 0,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s10	= 1,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s20	= 2,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s30	= 3,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s60	= 4,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s120	= 5,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s180	= 6,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s240	= 7,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_s300	= 8,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_spare7	= 9,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_spare6	= 10,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_spare5	= 11,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_spare4	= 12,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_spare3	= 13,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_spare2	= 14,
	SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17_spare1	= 15
} e_SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17__t_SearchDeltaP_Stationary_r17;
typedef enum SIB2__ext2__relaxedMeasurement_r17__combineRelaxedMeasCondition2_r17 {
	SIB2__ext2__relaxedMeasurement_r17__combineRelaxedMeasCondition2_r17_true	= 0
} e_SIB2__ext2__relaxedMeasurement_r17__combineRelaxedMeasCondition2_r17;

/* Forward declarations */
struct ThresholdNR;
struct MultiFrequencyBandListNR_SIB;
struct SSB_MTC;
struct SS_RSSI_Measurement;
struct SSB_ToMeasure;
struct SpeedStateScaleFactors;
struct SSB_MTC2_LP_r16;
struct SSB_MTC4List_r17;

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
		struct SIB2__intraFreqCellReselectionInfo__ext1 {
			struct SpeedStateScaleFactors	*t_ReselectionNR_SF;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *ext1;
		struct SIB2__intraFreqCellReselectionInfo__ext2 {
			struct SSB_MTC2_LP_r16	*smtc2_LP_r16;	/* OPTIONAL */
			SSB_PositionQCL_Relation_r16_t	*ssb_PositionQCL_Common_r16;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *ext2;
		struct SIB2__intraFreqCellReselectionInfo__ext3 {
			SSB_PositionQCL_Relation_r17_t	*ssb_PositionQCL_Common_r17;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *ext3;
		struct SIB2__intraFreqCellReselectionInfo__ext4 {
			struct SSB_MTC4List_r17	*smtc4list_r17;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *ext4;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} intraFreqCellReselectionInfo;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SIB2__ext1 {
		struct SIB2__ext1__relaxedMeasurement_r16 {
			struct SIB2__ext1__relaxedMeasurement_r16__lowMobilityEvaluation_r16 {
				long	 s_SearchDeltaP_r16;
				long	 t_SearchDeltaP_r16;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *lowMobilityEvaluation_r16;
			struct SIB2__ext1__relaxedMeasurement_r16__cellEdgeEvaluation_r16 {
				ReselectionThreshold_t	 s_SearchThresholdP_r16;
				ReselectionThresholdQ_t	*s_SearchThresholdQ_r16;	/* OPTIONAL */
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *cellEdgeEvaluation_r16;
			long	*combineRelaxedMeasCondition_r16;	/* OPTIONAL */
			long	*highPriorityMeasRelax_r16;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *relaxedMeasurement_r16;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct SIB2__ext2 {
		long	*cellEquivalentSize_r17;	/* OPTIONAL */
		struct SIB2__ext2__relaxedMeasurement_r17 {
			struct SIB2__ext2__relaxedMeasurement_r17__stationaryMobilityEvaluation_r17 {
				long	 s_SearchDeltaP_Stationary_r17;
				long	 t_SearchDeltaP_Stationary_r17;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} stationaryMobilityEvaluation_r17;
			struct SIB2__ext2__relaxedMeasurement_r17__cellEdgeEvaluationWhileStationary_r17 {
				ReselectionThreshold_t	 s_SearchThresholdP2_r17;
				ReselectionThresholdQ_t	*s_SearchThresholdQ2_r17;	/* OPTIONAL */
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *cellEdgeEvaluationWhileStationary_r17;
			long	*combineRelaxedMeasCondition2_r17;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *relaxedMeasurement_r17;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SIB2_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_q_Hyst_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sf_Medium_26;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sf_High_31;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_s_SearchDeltaP_r16_73;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_t_SearchDeltaP_r16_82;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_combineRelaxedMeasCondition_r16_102;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_highPriorityMeasRelax_r16_104;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_s_SearchDeltaP_Stationary_r17_110;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_t_SearchDeltaP_Stationary_r17_119;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_combineRelaxedMeasCondition2_r17_139;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SIB2;
extern asn_SEQUENCE_specifics_t asn_SPC_SIB2_specs_1;
extern asn_TYPE_member_t asn_MBR_SIB2_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ThresholdNR.h"
#include "MultiFrequencyBandListNR-SIB.h"
#include "SSB-MTC.h"
#include "SS-RSSI-Measurement.h"
#include "SSB-ToMeasure.h"
#include "SpeedStateScaleFactors.h"
#include "SSB-MTC2-LP-r16.h"
#include "SSB-MTC4List-r17.h"

#endif	/* _SIB2_H_ */
#include <asn_internal.h>
