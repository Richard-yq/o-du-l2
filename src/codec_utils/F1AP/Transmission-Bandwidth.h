/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "F1AP-IEs"
 * 	found in "../F1.asn1"
 * 	`asn1c -D ../F1_output/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#ifndef	_Transmission_Bandwidth_H_
#define	_Transmission_Bandwidth_H_


#include <asn_application.h>

/* Including external dependencies */
#include "NRSCS.h"
#include "NRNRB.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ProtocolExtensionContainer;

/* Transmission-Bandwidth */
typedef struct Transmission_Bandwidth {
	NRSCS_t	 nRSCS;
	NRNRB_t	 nRNRB;
	struct ProtocolExtensionContainer	*iE_Extensions;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Transmission_Bandwidth_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Transmission_Bandwidth;
extern asn_SEQUENCE_specifics_t asn_SPC_Transmission_Bandwidth_specs_1;
extern asn_TYPE_member_t asn_MBR_Transmission_Bandwidth_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _Transmission_Bandwidth_H_ */
#include <asn_internal.h>
