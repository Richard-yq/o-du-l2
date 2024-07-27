/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_FreqPriorityNR_H_
#define	_FreqPriorityNR_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ARFCN-ValueNR.h"
#include "CellReselectionPriority.h"
#include "CellReselectionSubPriority.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FreqPriorityNR */
typedef struct FreqPriorityNR {
	ARFCN_ValueNR_t	 carrierFreq;
	CellReselectionPriority_t	 cellReselectionPriority;
	CellReselectionSubPriority_t	*cellReselectionSubPriority;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} FreqPriorityNR_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_FreqPriorityNR;
extern asn_SEQUENCE_specifics_t asn_SPC_FreqPriorityNR_specs_1;
extern asn_TYPE_member_t asn_MBR_FreqPriorityNR_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _FreqPriorityNR_H_ */
#include <asn_internal.h>
