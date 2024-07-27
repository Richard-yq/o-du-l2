/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MRDC_Parameters_H_
#define	_MRDC_Parameters_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MRDC_Parameters__singleUL_Transmission {
	MRDC_Parameters__singleUL_Transmission_supported	= 0
} e_MRDC_Parameters__singleUL_Transmission;
typedef enum MRDC_Parameters__dynamicPowerSharing {
	MRDC_Parameters__dynamicPowerSharing_supported	= 0
} e_MRDC_Parameters__dynamicPowerSharing;
typedef enum MRDC_Parameters__tdm_Pattern {
	MRDC_Parameters__tdm_Pattern_supported	= 0
} e_MRDC_Parameters__tdm_Pattern;
typedef enum MRDC_Parameters__ul_SharingEUTRA_NR {
	MRDC_Parameters__ul_SharingEUTRA_NR_tdm	= 0,
	MRDC_Parameters__ul_SharingEUTRA_NR_fdm	= 1,
	MRDC_Parameters__ul_SharingEUTRA_NR_both	= 2
} e_MRDC_Parameters__ul_SharingEUTRA_NR;
typedef enum MRDC_Parameters__ul_SwitchingTimeEUTRA_NR {
	MRDC_Parameters__ul_SwitchingTimeEUTRA_NR_type1	= 0,
	MRDC_Parameters__ul_SwitchingTimeEUTRA_NR_type2	= 1
} e_MRDC_Parameters__ul_SwitchingTimeEUTRA_NR;
typedef enum MRDC_Parameters__simultaneousRxTxInterBandENDC {
	MRDC_Parameters__simultaneousRxTxInterBandENDC_supported	= 0
} e_MRDC_Parameters__simultaneousRxTxInterBandENDC;
typedef enum MRDC_Parameters__asyncIntraBandENDC {
	MRDC_Parameters__asyncIntraBandENDC_supported	= 0
} e_MRDC_Parameters__asyncIntraBandENDC;

/* MRDC-Parameters */
typedef struct MRDC_Parameters {
	long	*singleUL_Transmission;	/* OPTIONAL */
	long	*dynamicPowerSharing;	/* OPTIONAL */
	long	*tdm_Pattern;	/* OPTIONAL */
	long	*ul_SharingEUTRA_NR;	/* OPTIONAL */
	long	*ul_SwitchingTimeEUTRA_NR;	/* OPTIONAL */
	long	*simultaneousRxTxInterBandENDC;	/* OPTIONAL */
	long	*asyncIntraBandENDC;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MRDC_Parameters_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_singleUL_Transmission_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dynamicPowerSharing_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_tdm_Pattern_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ul_SharingEUTRA_NR_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ul_SwitchingTimeEUTRA_NR_12;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_simultaneousRxTxInterBandENDC_15;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_asyncIntraBandENDC_17;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MRDC_Parameters;
extern asn_SEQUENCE_specifics_t asn_SPC_MRDC_Parameters_specs_1;
extern asn_TYPE_member_t asn_MBR_MRDC_Parameters_1[7];

#ifdef __cplusplus
}
#endif

#endif	/* _MRDC_Parameters_H_ */
#include <asn_internal.h>
