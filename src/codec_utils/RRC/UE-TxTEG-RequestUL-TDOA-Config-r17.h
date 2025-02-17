/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UE_TxTEG_RequestUL_TDOA_Config_r17_H_
#define	_UE_TxTEG_RequestUL_TDOA_Config_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NULL.h>
#include <NativeEnumerated.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UE_TxTEG_RequestUL_TDOA_Config_r17_PR {
	UE_TxTEG_RequestUL_TDOA_Config_r17_PR_NOTHING,	/* No components present */
	UE_TxTEG_RequestUL_TDOA_Config_r17_PR_oneShot_r17,
	UE_TxTEG_RequestUL_TDOA_Config_r17_PR_periodicReporting_r17
} UE_TxTEG_RequestUL_TDOA_Config_r17_PR;
typedef enum UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17 {
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms160	= 0,
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms320	= 1,
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms1280	= 2,
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms2560	= 3,
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms61440	= 4,
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms81920	= 5,
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms368640	= 6,
	UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17_ms737280	= 7
} e_UE_TxTEG_RequestUL_TDOA_Config_r17__periodicReporting_r17;

/* UE-TxTEG-RequestUL-TDOA-Config-r17 */
typedef struct UE_TxTEG_RequestUL_TDOA_Config_r17 {
	UE_TxTEG_RequestUL_TDOA_Config_r17_PR present;
	union UE_TxTEG_RequestUL_TDOA_Config_r17_u {
		NULL_t	 oneShot_r17;
		long	 periodicReporting_r17;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UE_TxTEG_RequestUL_TDOA_Config_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_periodicReporting_r17_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_UE_TxTEG_RequestUL_TDOA_Config_r17;
extern asn_CHOICE_specifics_t asn_SPC_UE_TxTEG_RequestUL_TDOA_Config_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_UE_TxTEG_RequestUL_TDOA_Config_r17_1[2];
extern asn_per_constraints_t asn_PER_type_UE_TxTEG_RequestUL_TDOA_Config_r17_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _UE_TxTEG_RequestUL_TDOA_Config_r17_H_ */
#include <asn_internal.h>
