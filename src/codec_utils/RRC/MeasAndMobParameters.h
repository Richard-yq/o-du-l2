/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasAndMobParameters_H_
#define	_MeasAndMobParameters_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct MeasAndMobParametersCommon;
struct MeasAndMobParametersXDD_Diff;
struct MeasAndMobParametersFRX_Diff;

/* MeasAndMobParameters */
typedef struct MeasAndMobParameters {
	struct MeasAndMobParametersCommon	*measAndMobParametersCommon;	/* OPTIONAL */
	struct MeasAndMobParametersXDD_Diff	*measAndMobParametersXDD_Diff;	/* OPTIONAL */
	struct MeasAndMobParametersFRX_Diff	*measAndMobParametersFRX_Diff;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasAndMobParameters_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasAndMobParameters;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasAndMobParameters_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasAndMobParameters_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasAndMobParametersCommon.h"
#include "MeasAndMobParametersXDD-Diff.h"
#include "MeasAndMobParametersFRX-Diff.h"

#endif	/* _MeasAndMobParameters_H_ */
#include <asn_internal.h>
