/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-Sidelink-Preconf"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_RoHC_Profiles_r16_H_
#define	_SL_RoHC_Profiles_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SL-RoHC-Profiles-r16 */
typedef struct SL_RoHC_Profiles_r16 {
	BOOLEAN_t	 profile0x0001_r16;
	BOOLEAN_t	 profile0x0002_r16;
	BOOLEAN_t	 profile0x0003_r16;
	BOOLEAN_t	 profile0x0004_r16;
	BOOLEAN_t	 profile0x0006_r16;
	BOOLEAN_t	 profile0x0101_r16;
	BOOLEAN_t	 profile0x0102_r16;
	BOOLEAN_t	 profile0x0103_r16;
	BOOLEAN_t	 profile0x0104_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_RoHC_Profiles_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SL_RoHC_Profiles_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_RoHC_Profiles_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_RoHC_Profiles_r16_1[9];

#ifdef __cplusplus
}
#endif

#endif	/* _SL_RoHC_Profiles_r16_H_ */
#include <asn_internal.h>
