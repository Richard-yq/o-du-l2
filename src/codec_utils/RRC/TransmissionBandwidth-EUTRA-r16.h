/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_TransmissionBandwidth_EUTRA_r16_H_
#define	_TransmissionBandwidth_EUTRA_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TransmissionBandwidth_EUTRA_r16 {
	TransmissionBandwidth_EUTRA_r16_rb6	= 0,
	TransmissionBandwidth_EUTRA_r16_rb15	= 1,
	TransmissionBandwidth_EUTRA_r16_rb25	= 2,
	TransmissionBandwidth_EUTRA_r16_rb50	= 3,
	TransmissionBandwidth_EUTRA_r16_rb75	= 4,
	TransmissionBandwidth_EUTRA_r16_rb100	= 5
} e_TransmissionBandwidth_EUTRA_r16;

/* TransmissionBandwidth-EUTRA-r16 */
typedef long	 TransmissionBandwidth_EUTRA_r16_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_TransmissionBandwidth_EUTRA_r16_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_TransmissionBandwidth_EUTRA_r16;
extern const asn_INTEGER_specifics_t asn_SPC_TransmissionBandwidth_EUTRA_r16_specs_1;
asn_struct_free_f TransmissionBandwidth_EUTRA_r16_free;
asn_struct_print_f TransmissionBandwidth_EUTRA_r16_print;
asn_constr_check_f TransmissionBandwidth_EUTRA_r16_constraint;
xer_type_decoder_f TransmissionBandwidth_EUTRA_r16_decode_xer;
xer_type_encoder_f TransmissionBandwidth_EUTRA_r16_encode_xer;
per_type_decoder_f TransmissionBandwidth_EUTRA_r16_decode_uper;
per_type_encoder_f TransmissionBandwidth_EUTRA_r16_encode_uper;
per_type_decoder_f TransmissionBandwidth_EUTRA_r16_decode_aper;
per_type_encoder_f TransmissionBandwidth_EUTRA_r16_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _TransmissionBandwidth_EUTRA_r16_H_ */
#include <asn_internal.h>
