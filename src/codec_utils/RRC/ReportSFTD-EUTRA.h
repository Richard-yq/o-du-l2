/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_ReportSFTD_EUTRA_H_
#define	_ReportSFTD_EUTRA_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ReportSFTD-EUTRA */
typedef struct ReportSFTD_EUTRA {
	BOOLEAN_t	 reportSFTD_Meas;
	BOOLEAN_t	 reportRSRP;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ReportSFTD_EUTRA_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ReportSFTD_EUTRA;
extern asn_SEQUENCE_specifics_t asn_SPC_ReportSFTD_EUTRA_specs_1;
extern asn_TYPE_member_t asn_MBR_ReportSFTD_EUTRA_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ReportSFTD_EUTRA_H_ */
#include <asn_internal.h>
