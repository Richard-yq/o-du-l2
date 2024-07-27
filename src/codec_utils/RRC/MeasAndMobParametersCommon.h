/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasAndMobParametersCommon_H_
#define	_MeasAndMobParametersCommon_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BIT_STRING.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MeasAndMobParametersCommon__ssb_RLM {
	MeasAndMobParametersCommon__ssb_RLM_supported	= 0
} e_MeasAndMobParametersCommon__ssb_RLM;
typedef enum MeasAndMobParametersCommon__ssb_AndCSI_RS_RLM {
	MeasAndMobParametersCommon__ssb_AndCSI_RS_RLM_supported	= 0
} e_MeasAndMobParametersCommon__ssb_AndCSI_RS_RLM;
typedef enum MeasAndMobParametersCommon__ext1__eventB_MeasAndReport {
	MeasAndMobParametersCommon__ext1__eventB_MeasAndReport_supported	= 0
} e_MeasAndMobParametersCommon__ext1__eventB_MeasAndReport;
typedef enum MeasAndMobParametersCommon__ext1__handoverFDD_TDD {
	MeasAndMobParametersCommon__ext1__handoverFDD_TDD_supported	= 0
} e_MeasAndMobParametersCommon__ext1__handoverFDD_TDD;
typedef enum MeasAndMobParametersCommon__ext1__eutra_CGI_Reporting {
	MeasAndMobParametersCommon__ext1__eutra_CGI_Reporting_supported	= 0
} e_MeasAndMobParametersCommon__ext1__eutra_CGI_Reporting;
typedef enum MeasAndMobParametersCommon__ext1__nr_CGI_Reporting {
	MeasAndMobParametersCommon__ext1__nr_CGI_Reporting_supported	= 0
} e_MeasAndMobParametersCommon__ext1__nr_CGI_Reporting;

/* MeasAndMobParametersCommon */
typedef struct MeasAndMobParametersCommon {
	BIT_STRING_t	*supportedGapPattern;	/* OPTIONAL */
	long	*ssb_RLM;	/* OPTIONAL */
	long	*ssb_AndCSI_RS_RLM;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct MeasAndMobParametersCommon__ext1 {
		long	*eventB_MeasAndReport;	/* OPTIONAL */
		long	*handoverFDD_TDD;	/* OPTIONAL */
		long	*eutra_CGI_Reporting;	/* OPTIONAL */
		long	*nr_CGI_Reporting;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasAndMobParametersCommon_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ssb_RLM_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ssb_AndCSI_RS_RLM_5;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_eventB_MeasAndReport_9;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_handoverFDD_TDD_11;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_eutra_CGI_Reporting_13;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_nr_CGI_Reporting_15;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MeasAndMobParametersCommon;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasAndMobParametersCommon_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasAndMobParametersCommon_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _MeasAndMobParametersCommon_H_ */
#include <asn_internal.h>
