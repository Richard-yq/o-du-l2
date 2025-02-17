/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_TMGI_r17_H_
#define	_TMGI_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>
#include <NativeInteger.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TMGI_r17__plmn_Id_r17_PR {
	TMGI_r17__plmn_Id_r17_PR_NOTHING,	/* No components present */
	TMGI_r17__plmn_Id_r17_PR_plmn_Index,
	TMGI_r17__plmn_Id_r17_PR_explicitValue
} TMGI_r17__plmn_Id_r17_PR;

/* Forward declarations */
struct PLMN_Identity;

/* TMGI-r17 */
typedef struct TMGI_r17 {
	struct TMGI_r17__plmn_Id_r17 {
		TMGI_r17__plmn_Id_r17_PR present;
		union TMGI_r17__plmn_Id_r17_u {
			long	 plmn_Index;
			struct PLMN_Identity	*explicitValue;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} plmn_Id_r17;
	OCTET_STRING_t	 serviceId_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TMGI_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TMGI_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_TMGI_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_TMGI_r17_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PLMN-Identity.h"

#endif	/* _TMGI_r17_H_ */
#include <asn_internal.h>
