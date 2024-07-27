/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CSI_SSB_ResourceSetId_H_
#define	_CSI_SSB_ResourceSetId_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CSI-SSB-ResourceSetId */
typedef long	 CSI_SSB_ResourceSetId_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_CSI_SSB_ResourceSetId_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_CSI_SSB_ResourceSetId;
asn_struct_free_f CSI_SSB_ResourceSetId_free;
asn_struct_print_f CSI_SSB_ResourceSetId_print;
asn_constr_check_f CSI_SSB_ResourceSetId_constraint;
xer_type_decoder_f CSI_SSB_ResourceSetId_decode_xer;
xer_type_encoder_f CSI_SSB_ResourceSetId_encode_xer;
per_type_decoder_f CSI_SSB_ResourceSetId_decode_uper;
per_type_encoder_f CSI_SSB_ResourceSetId_encode_uper;
per_type_decoder_f CSI_SSB_ResourceSetId_decode_aper;
per_type_encoder_f CSI_SSB_ResourceSetId_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _CSI_SSB_ResourceSetId_H_ */
#include <asn_internal.h>
