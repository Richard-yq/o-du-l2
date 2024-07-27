/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_TrackingAreaCode_H_
#define	_TrackingAreaCode_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TrackingAreaCode */
typedef BIT_STRING_t	 TrackingAreaCode_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_TrackingAreaCode_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_TrackingAreaCode;
asn_struct_free_f TrackingAreaCode_free;
asn_struct_print_f TrackingAreaCode_print;
asn_constr_check_f TrackingAreaCode_constraint;
xer_type_decoder_f TrackingAreaCode_decode_xer;
xer_type_encoder_f TrackingAreaCode_encode_xer;
per_type_decoder_f TrackingAreaCode_decode_uper;
per_type_encoder_f TrackingAreaCode_encode_uper;
per_type_decoder_f TrackingAreaCode_decode_aper;
per_type_encoder_f TrackingAreaCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _TrackingAreaCode_H_ */
#include <asn_internal.h>
