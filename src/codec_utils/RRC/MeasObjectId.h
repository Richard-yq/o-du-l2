/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasObjectId_H_
#define	_MeasObjectId_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MeasObjectId */
typedef long	 MeasObjectId_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_MeasObjectId_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_MeasObjectId;
asn_struct_free_f MeasObjectId_free;
asn_struct_print_f MeasObjectId_print;
asn_constr_check_f MeasObjectId_constraint;
xer_type_decoder_f MeasObjectId_decode_xer;
xer_type_encoder_f MeasObjectId_encode_xer;
per_type_decoder_f MeasObjectId_decode_uper;
per_type_encoder_f MeasObjectId_encode_uper;
per_type_decoder_f MeasObjectId_decode_aper;
per_type_encoder_f MeasObjectId_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _MeasObjectId_H_ */
#include <asn_internal.h>
