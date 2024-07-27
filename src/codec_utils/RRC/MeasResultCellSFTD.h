/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasResultCellSFTD_H_
#define	_MeasResultCellSFTD_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PhysCellId.h"
#include <NativeInteger.h>
#include "RSRP-Range.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MeasResultCellSFTD */
typedef struct MeasResultCellSFTD {
	PhysCellId_t	 physCellId;
	long	 sfn_OffsetResult;
	long	 frameBoundaryOffsetResult;
	RSRP_Range_t	*rsrp_Result;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasResultCellSFTD_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasResultCellSFTD;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasResultCellSFTD_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasResultCellSFTD_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _MeasResultCellSFTD_H_ */
#include <asn_internal.h>
