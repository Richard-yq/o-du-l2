/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_LoggingInterval_r16_H_
#define	_LoggingInterval_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum LoggingInterval_r16 {
	LoggingInterval_r16_ms320	= 0,
	LoggingInterval_r16_ms640	= 1,
	LoggingInterval_r16_ms1280	= 2,
	LoggingInterval_r16_ms2560	= 3,
	LoggingInterval_r16_ms5120	= 4,
	LoggingInterval_r16_ms10240	= 5,
	LoggingInterval_r16_ms20480	= 6,
	LoggingInterval_r16_ms30720	= 7,
	LoggingInterval_r16_ms40960	= 8,
	LoggingInterval_r16_ms61440	= 9,
	LoggingInterval_r16_infinity	= 10
} e_LoggingInterval_r16;

/* LoggingInterval-r16 */
typedef long	 LoggingInterval_r16_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_LoggingInterval_r16_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_LoggingInterval_r16;
extern const asn_INTEGER_specifics_t asn_SPC_LoggingInterval_r16_specs_1;
asn_struct_free_f LoggingInterval_r16_free;
asn_struct_print_f LoggingInterval_r16_print;
asn_constr_check_f LoggingInterval_r16_constraint;
xer_type_decoder_f LoggingInterval_r16_decode_xer;
xer_type_encoder_f LoggingInterval_r16_encode_xer;
per_type_decoder_f LoggingInterval_r16_decode_uper;
per_type_encoder_f LoggingInterval_r16_encode_uper;
per_type_decoder_f LoggingInterval_r16_decode_aper;
per_type_encoder_f LoggingInterval_r16_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _LoggingInterval_r16_H_ */
#include <asn_internal.h>
