/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasResults_H_
#define	_MeasResults_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MeasId.h"
#include "MeasResultServMOList.h"
#include <OCTET_STRING.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MeasResults__measResultNeighCells_PR {
	MeasResults__measResultNeighCells_PR_NOTHING,	/* No components present */
	MeasResults__measResultNeighCells_PR_measResultListNR,
	/* Extensions may appear below */
	MeasResults__measResultNeighCells_PR_measResultListEUTRA,
	MeasResults__measResultNeighCells_PR_measResultListUTRA_FDD_r16,
	MeasResults__measResultNeighCells_PR_sl_MeasResultsCandRelay_r17
} MeasResults__measResultNeighCells_PR;

/* Forward declarations */
struct MeasResultListNR;
struct MeasResultListEUTRA;
struct MeasResultListUTRA_FDD_r16;
struct MeasResultServFreqListEUTRA_SCG;
struct MeasResultServFreqListNR_SCG;
struct MeasResultSFTD_EUTRA;
struct MeasResultCellSFTD_NR;
struct MeasResultCellListSFTD_NR;
struct MeasResultForRSSI_r16;
struct LocationInfo_r16;
struct UL_PDCP_DelayValueResultList_r16;
struct MeasResultsSL_r16;
struct MeasResultCLI_r16;
struct MeasResultRxTxTimeDiff_r17;
struct UL_PDCP_ExcessDelayResultList_r17;

/* MeasResults */
typedef struct MeasResults {
	MeasId_t	 measId;
	MeasResultServMOList_t	 measResultServingMOList;
	struct MeasResults__measResultNeighCells {
		MeasResults__measResultNeighCells_PR present;
		union MeasResults__measResultNeighCells_u {
			struct MeasResultListNR	*measResultListNR;
			/*
			 * This type is extensible,
			 * possible extensions are below.
			 */
			struct MeasResultListEUTRA	*measResultListEUTRA;
			struct MeasResultListUTRA_FDD_r16	*measResultListUTRA_FDD_r16;
			OCTET_STRING_t	 sl_MeasResultsCandRelay_r17;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *measResultNeighCells;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct MeasResults__ext1 {
		struct MeasResultServFreqListEUTRA_SCG	*measResultServFreqListEUTRA_SCG;	/* OPTIONAL */
		struct MeasResultServFreqListNR_SCG	*measResultServFreqListNR_SCG;	/* OPTIONAL */
		struct MeasResultSFTD_EUTRA	*measResultSFTD_EUTRA;	/* OPTIONAL */
		struct MeasResultCellSFTD_NR	*measResultSFTD_NR;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct MeasResults__ext2 {
		struct MeasResultCellListSFTD_NR	*measResultCellListSFTD_NR;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct MeasResults__ext3 {
		struct MeasResultForRSSI_r16	*measResultForRSSI_r16;	/* OPTIONAL */
		struct LocationInfo_r16	*locationInfo_r16;	/* OPTIONAL */
		struct UL_PDCP_DelayValueResultList_r16	*ul_PDCP_DelayValueResultList_r16;	/* OPTIONAL */
		struct MeasResultsSL_r16	*measResultsSL_r16;	/* OPTIONAL */
		struct MeasResultCLI_r16	*measResultCLI_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	struct MeasResults__ext4 {
		struct MeasResultRxTxTimeDiff_r17	*measResultRxTxTimeDiff_r17;	/* OPTIONAL */
		OCTET_STRING_t	*sl_MeasResultServingRelay_r17;	/* OPTIONAL */
		struct UL_PDCP_ExcessDelayResultList_r17	*ul_PDCP_ExcessDelayResultList_r17;	/* OPTIONAL */
		OCTET_STRING_t	*coarseLocationInfo_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext4;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasResults_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasResults;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasResults_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasResults_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasResultListNR.h"
#include "MeasResultListEUTRA.h"
#include "MeasResultListUTRA-FDD-r16.h"
#include "MeasResultServFreqListEUTRA-SCG.h"
#include "MeasResultServFreqListNR-SCG.h"
#include "MeasResultSFTD-EUTRA.h"
#include "MeasResultCellSFTD-NR.h"
#include "MeasResultCellListSFTD-NR.h"
#include "MeasResultForRSSI-r16.h"
#include "LocationInfo-r16.h"
#include "UL-PDCP-DelayValueResultList-r16.h"
#include "MeasResultsSL-r16.h"
#include "MeasResultCLI-r16.h"
#include "MeasResultRxTxTimeDiff-r17.h"
#include "UL-PDCP-ExcessDelayResultList-r17.h"

#endif	/* _MeasResults_H_ */
#include <asn_internal.h>
