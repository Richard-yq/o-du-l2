/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "F1AP-IEs"
 * 	found in "../F1.asn1"
 * 	`asn1c -D ../F1_output/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#ifndef	_DRX_Config_H_
#define	_DRX_Config_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DRX-Config */
typedef OCTET_STRING_t	 DRX_Config_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DRX_ConfigF1AP;
asn_struct_free_f DRX_Config_free;
asn_struct_print_f DRX_Config_print;
asn_constr_check_f DRX_Config_constraint;
ber_type_decoder_f DRX_Config_decode_ber;
der_type_encoder_f DRX_Config_encode_der;
xer_type_decoder_f DRX_Config_decode_xer;
xer_type_encoder_f DRX_Config_encode_xer;
oer_type_decoder_f DRX_Config_decode_oer;
oer_type_encoder_f DRX_Config_encode_oer;
per_type_decoder_f DRX_Config_decode_uper;
per_type_encoder_f DRX_Config_encode_uper;
per_type_decoder_f DRX_Config_decode_aper;
per_type_encoder_f DRX_Config_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _DRX_Config_H_ */
#include <asn_internal.h>
