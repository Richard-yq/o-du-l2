/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BandParametersSidelink_r16_H_
#define	_BandParametersSidelink_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "FreqBandIndicatorNR.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BandParametersSidelink-r16 */
typedef struct BandParametersSidelink_r16 {
	FreqBandIndicatorNR_t	 freqBandSidelink_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BandParametersSidelink_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BandParametersSidelink_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_BandParametersSidelink_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_BandParametersSidelink_r16_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _BandParametersSidelink_r16_H_ */
#include <asn_internal.h>
