/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_TCI_UL_State_r17_H_
#define	_TCI_UL_State_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "TCI-UL-State-Id-r17.h"
#include "ServCellIndex.h"
#include "BWP-Id.h"
#include "AdditionalPCIIndex-r17.h"
#include "Uplink-powerControlId-r17.h"
#include "PathlossReferenceRS-Id-r17.h"
#include "SSB-Index.h"
#include "NZP-CSI-RS-ResourceId.h"
#include "SRS-ResourceId.h"
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TCI_UL_State_r17__referenceSignal_r17_PR {
	TCI_UL_State_r17__referenceSignal_r17_PR_NOTHING,	/* No components present */
	TCI_UL_State_r17__referenceSignal_r17_PR_ssb_Index_r17,
	TCI_UL_State_r17__referenceSignal_r17_PR_csi_RS_Index_r17,
	TCI_UL_State_r17__referenceSignal_r17_PR_srs_r17
} TCI_UL_State_r17__referenceSignal_r17_PR;

/* TCI-UL-State-r17 */
typedef struct TCI_UL_State_r17 {
	TCI_UL_State_Id_r17_t	 tci_UL_State_Id_r17;
	ServCellIndex_t	*servingCellId_r17;	/* OPTIONAL */
	BWP_Id_t	*bwp_Id_r17;	/* OPTIONAL */
	struct TCI_UL_State_r17__referenceSignal_r17 {
		TCI_UL_State_r17__referenceSignal_r17_PR present;
		union TCI_UL_State_r17__referenceSignal_r17_u {
			SSB_Index_t	 ssb_Index_r17;
			NZP_CSI_RS_ResourceId_t	 csi_RS_Index_r17;
			SRS_ResourceId_t	 srs_r17;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} referenceSignal_r17;
	AdditionalPCIIndex_r17_t	*additionalPCI_r17;	/* OPTIONAL */
	Uplink_powerControlId_r17_t	*ul_powerControl_r17;	/* OPTIONAL */
	PathlossReferenceRS_Id_r17_t	*pathlossReferenceRS_Id_r17;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TCI_UL_State_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TCI_UL_State_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_TCI_UL_State_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_TCI_UL_State_r17_1[7];

#ifdef __cplusplus
}
#endif

#endif	/* _TCI_UL_State_r17_H_ */
#include <asn_internal.h>
