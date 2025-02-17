/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RMTC_Config_r16_H_
#define	_RMTC_Config_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include "ARFCN-ValueNR.h"
#include "TCI-StateId.h"
#include "ServCellIndex.h"
#include <constr_SEQUENCE.h>
#include "BWP-Id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RMTC_Config_r16__rmtc_Periodicity_r16 {
	RMTC_Config_r16__rmtc_Periodicity_r16_ms40	= 0,
	RMTC_Config_r16__rmtc_Periodicity_r16_ms80	= 1,
	RMTC_Config_r16__rmtc_Periodicity_r16_ms160	= 2,
	RMTC_Config_r16__rmtc_Periodicity_r16_ms320	= 3,
	RMTC_Config_r16__rmtc_Periodicity_r16_ms640	= 4
} e_RMTC_Config_r16__rmtc_Periodicity_r16;
typedef enum RMTC_Config_r16__measDurationSymbols_r16 {
	RMTC_Config_r16__measDurationSymbols_r16_sym1	= 0,
	RMTC_Config_r16__measDurationSymbols_r16_sym14or12	= 1,
	RMTC_Config_r16__measDurationSymbols_r16_sym28or24	= 2,
	RMTC_Config_r16__measDurationSymbols_r16_sym42or36	= 3,
	RMTC_Config_r16__measDurationSymbols_r16_sym70or60	= 4
} e_RMTC_Config_r16__measDurationSymbols_r16;
typedef enum RMTC_Config_r16__ref_SCS_CP_r16 {
	RMTC_Config_r16__ref_SCS_CP_r16_kHz15	= 0,
	RMTC_Config_r16__ref_SCS_CP_r16_kHz30	= 1,
	RMTC_Config_r16__ref_SCS_CP_r16_kHz60_NCP	= 2,
	RMTC_Config_r16__ref_SCS_CP_r16_kHz60_ECP	= 3
} e_RMTC_Config_r16__ref_SCS_CP_r16;
typedef enum RMTC_Config_r16__ext1__rmtc_Bandwidth_r17 {
	RMTC_Config_r16__ext1__rmtc_Bandwidth_r17_mhz100	= 0,
	RMTC_Config_r16__ext1__rmtc_Bandwidth_r17_mhz400	= 1,
	RMTC_Config_r16__ext1__rmtc_Bandwidth_r17_mhz800	= 2,
	RMTC_Config_r16__ext1__rmtc_Bandwidth_r17_mhz1600	= 3,
	RMTC_Config_r16__ext1__rmtc_Bandwidth_r17_mhz2000	= 4
} e_RMTC_Config_r16__ext1__rmtc_Bandwidth_r17;
typedef enum RMTC_Config_r16__ext1__measDurationSymbols_v1700 {
	RMTC_Config_r16__ext1__measDurationSymbols_v1700_sym140	= 0,
	RMTC_Config_r16__ext1__measDurationSymbols_v1700_sym560	= 1,
	RMTC_Config_r16__ext1__measDurationSymbols_v1700_sym1120	= 2
} e_RMTC_Config_r16__ext1__measDurationSymbols_v1700;
typedef enum RMTC_Config_r16__ext1__ref_SCS_CP_v1700 {
	RMTC_Config_r16__ext1__ref_SCS_CP_v1700_kHz120	= 0,
	RMTC_Config_r16__ext1__ref_SCS_CP_v1700_kHz480	= 1,
	RMTC_Config_r16__ext1__ref_SCS_CP_v1700_kHz960	= 2
} e_RMTC_Config_r16__ext1__ref_SCS_CP_v1700;

/* RMTC-Config-r16 */
typedef struct RMTC_Config_r16 {
	long	 rmtc_Periodicity_r16;
	long	*rmtc_SubframeOffset_r16;	/* OPTIONAL */
	long	 measDurationSymbols_r16;
	ARFCN_ValueNR_t	 rmtc_Frequency_r16;
	long	 ref_SCS_CP_r16;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct RMTC_Config_r16__ext1 {
		long	*rmtc_Bandwidth_r17;	/* OPTIONAL */
		long	*measDurationSymbols_v1700;	/* OPTIONAL */
		long	*ref_SCS_CP_v1700;	/* OPTIONAL */
		struct RMTC_Config_r16__ext1__tci_StateInfo_r17 {
			TCI_StateId_t	 tci_StateId_r17;
			ServCellIndex_t	*ref_ServCellId_r17;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *tci_StateInfo_r17;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct RMTC_Config_r16__ext2 {
		BWP_Id_t	*ref_BWPId_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RMTC_Config_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_rmtc_Periodicity_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_measDurationSymbols_r16_9;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ref_SCS_CP_r16_16;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_rmtc_Bandwidth_r17_23;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_measDurationSymbols_v1700_29;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ref_SCS_CP_v1700_33;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_RMTC_Config_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_RMTC_Config_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_RMTC_Config_r16_1[7];

#ifdef __cplusplus
}
#endif

#endif	/* _RMTC_Config_r16_H_ */
#include <asn_internal.h>
