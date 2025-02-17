/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MRB_ToAddMod_r17_H_
#define	_MRB_ToAddMod_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MRB-Identity-r17.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MRB_ToAddMod_r17__reestablishPDCP_r17 {
	MRB_ToAddMod_r17__reestablishPDCP_r17_true	= 0
} e_MRB_ToAddMod_r17__reestablishPDCP_r17;
typedef enum MRB_ToAddMod_r17__recoverPDCP_r17 {
	MRB_ToAddMod_r17__recoverPDCP_r17_true	= 0
} e_MRB_ToAddMod_r17__recoverPDCP_r17;

/* Forward declarations */
struct TMGI_r17;
struct PDCP_Config;

/* MRB-ToAddMod-r17 */
typedef struct MRB_ToAddMod_r17 {
	struct TMGI_r17	*mbs_SessionId_r17;	/* OPTIONAL */
	MRB_Identity_r17_t	 mrb_Identity_r17;
	MRB_Identity_r17_t	*mrb_IdentityNew_r17;	/* OPTIONAL */
	long	*reestablishPDCP_r17;	/* OPTIONAL */
	long	*recoverPDCP_r17;	/* OPTIONAL */
	struct PDCP_Config	*pdcp_Config_r17;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MRB_ToAddMod_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_reestablishPDCP_r17_5;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_recoverPDCP_r17_7;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MRB_ToAddMod_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_MRB_ToAddMod_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_MRB_ToAddMod_r17_1[6];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "TMGI-r17.h"
#include "PDCP-Config.h"

#endif	/* _MRB_ToAddMod_r17_H_ */
#include <asn_internal.h>
