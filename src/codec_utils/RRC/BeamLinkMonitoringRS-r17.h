/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BeamLinkMonitoringRS_r17_H_
#define	_BeamLinkMonitoringRS_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "BeamLinkMonitoringRS-Id-r17.h"
#include "SSB-Index.h"
#include "NZP-CSI-RS-ResourceId.h"
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BeamLinkMonitoringRS_r17__detectionResource_r17_PR {
	BeamLinkMonitoringRS_r17__detectionResource_r17_PR_NOTHING,	/* No components present */
	BeamLinkMonitoringRS_r17__detectionResource_r17_PR_ssb_Index,
	BeamLinkMonitoringRS_r17__detectionResource_r17_PR_csi_RS_Index
} BeamLinkMonitoringRS_r17__detectionResource_r17_PR;

/* BeamLinkMonitoringRS-r17 */
typedef struct BeamLinkMonitoringRS_r17 {
	BeamLinkMonitoringRS_Id_r17_t	 beamLinkMonitoringRS_Id_r17;
	struct BeamLinkMonitoringRS_r17__detectionResource_r17 {
		BeamLinkMonitoringRS_r17__detectionResource_r17_PR present;
		union BeamLinkMonitoringRS_r17__detectionResource_r17_u {
			SSB_Index_t	 ssb_Index;
			NZP_CSI_RS_ResourceId_t	 csi_RS_Index;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} detectionResource_r17;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BeamLinkMonitoringRS_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BeamLinkMonitoringRS_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_BeamLinkMonitoringRS_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_BeamLinkMonitoringRS_r17_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _BeamLinkMonitoringRS_r17_H_ */
#include <asn_internal.h>
