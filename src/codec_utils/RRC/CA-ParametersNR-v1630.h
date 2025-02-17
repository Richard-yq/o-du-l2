/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CA_ParametersNR_v1630_H_
#define	_CA_ParametersNR_v1630_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CA_ParametersNR_v1630__beamManagementType_r16 {
	CA_ParametersNR_v1630__beamManagementType_r16_ibm	= 0,
	CA_ParametersNR_v1630__beamManagementType_r16_dummy	= 1
} e_CA_ParametersNR_v1630__beamManagementType_r16;
typedef enum CA_ParametersNR_v1630__intraBandFreqSeparationUL_AggBW_GapBW_r16 {
	CA_ParametersNR_v1630__intraBandFreqSeparationUL_AggBW_GapBW_r16_classI	= 0,
	CA_ParametersNR_v1630__intraBandFreqSeparationUL_AggBW_GapBW_r16_classII	= 1,
	CA_ParametersNR_v1630__intraBandFreqSeparationUL_AggBW_GapBW_r16_classIII	= 2
} e_CA_ParametersNR_v1630__intraBandFreqSeparationUL_AggBW_GapBW_r16;
typedef enum CA_ParametersNR_v1630__interCA_NonAlignedFrame_B_r16 {
	CA_ParametersNR_v1630__interCA_NonAlignedFrame_B_r16_supported	= 0
} e_CA_ParametersNR_v1630__interCA_NonAlignedFrame_B_r16;

/* Forward declarations */
struct SimulSRS_ForAntennaSwitching_r16;

/* CA-ParametersNR-v1630 */
typedef struct CA_ParametersNR_v1630 {
	struct SimulSRS_ForAntennaSwitching_r16	*simulTX_SRS_AntSwitchingInterBandUL_CA_r16;	/* OPTIONAL */
	long	*beamManagementType_r16;	/* OPTIONAL */
	long	*intraBandFreqSeparationUL_AggBW_GapBW_r16;	/* OPTIONAL */
	long	*interCA_NonAlignedFrame_B_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CA_ParametersNR_v1630_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_beamManagementType_r16_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_intraBandFreqSeparationUL_AggBW_GapBW_r16_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_interCA_NonAlignedFrame_B_r16_10;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CA_ParametersNR_v1630;
extern asn_SEQUENCE_specifics_t asn_SPC_CA_ParametersNR_v1630_specs_1;
extern asn_TYPE_member_t asn_MBR_CA_ParametersNR_v1630_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SimulSRS-ForAntennaSwitching-r16.h"

#endif	/* _CA_ParametersNR_v1630_H_ */
#include <asn_internal.h>
