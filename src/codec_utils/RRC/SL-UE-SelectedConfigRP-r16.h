/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_UE_SelectedConfigRP_r16_H_
#define	_SL_UE_SelectedConfigRP_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_UE_SelectedConfigRP_r16__sl_MultiReserveResource_r16 {
	SL_UE_SelectedConfigRP_r16__sl_MultiReserveResource_r16_enabled	= 0
} e_SL_UE_SelectedConfigRP_r16__sl_MultiReserveResource_r16;
typedef enum SL_UE_SelectedConfigRP_r16__sl_MaxNumPerReserve_r16 {
	SL_UE_SelectedConfigRP_r16__sl_MaxNumPerReserve_r16_n2	= 0,
	SL_UE_SelectedConfigRP_r16__sl_MaxNumPerReserve_r16_n3	= 1
} e_SL_UE_SelectedConfigRP_r16__sl_MaxNumPerReserve_r16;
typedef enum SL_UE_SelectedConfigRP_r16__sl_SensingWindow_r16 {
	SL_UE_SelectedConfigRP_r16__sl_SensingWindow_r16_ms100	= 0,
	SL_UE_SelectedConfigRP_r16__sl_SensingWindow_r16_ms1100	= 1
} e_SL_UE_SelectedConfigRP_r16__sl_SensingWindow_r16;
typedef enum SL_UE_SelectedConfigRP_r16__sl_RS_ForSensing_r16 {
	SL_UE_SelectedConfigRP_r16__sl_RS_ForSensing_r16_pscch	= 0,
	SL_UE_SelectedConfigRP_r16__sl_RS_ForSensing_r16_pssch	= 1
} e_SL_UE_SelectedConfigRP_r16__sl_RS_ForSensing_r16;

/* Forward declarations */
struct SL_CBR_PriorityTxConfigList_r16;
struct SL_Thres_RSRP_List_r16;
struct SL_SelectionWindowList_r16;
struct SL_ResourceReservePeriod_r16;
struct SL_CBR_PriorityTxConfigList_v1650;

/* SL-UE-SelectedConfigRP-r16 */
typedef struct SL_UE_SelectedConfigRP_r16 {
	struct SL_CBR_PriorityTxConfigList_r16	*sl_CBR_PriorityTxConfigList_r16;	/* OPTIONAL */
	struct SL_Thres_RSRP_List_r16	*sl_Thres_RSRP_List_r16;	/* OPTIONAL */
	long	*sl_MultiReserveResource_r16;	/* OPTIONAL */
	long	*sl_MaxNumPerReserve_r16;	/* OPTIONAL */
	long	*sl_SensingWindow_r16;	/* OPTIONAL */
	struct SL_SelectionWindowList_r16	*sl_SelectionWindowList_r16;	/* OPTIONAL */
	struct SL_UE_SelectedConfigRP_r16__sl_ResourceReservePeriodList_r16 {
		A_SEQUENCE_OF(struct SL_ResourceReservePeriod_r16) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *sl_ResourceReservePeriodList_r16;
	long	 sl_RS_ForSensing_r16;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SL_UE_SelectedConfigRP_r16__ext1 {
		struct SL_CBR_PriorityTxConfigList_v1650	*sl_CBR_PriorityTxConfigList_v1650;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_UE_SelectedConfigRP_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_MultiReserveResource_r16_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_MaxNumPerReserve_r16_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_SensingWindow_r16_9;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_RS_ForSensing_r16_15;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_UE_SelectedConfigRP_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_UE_SelectedConfigRP_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_UE_SelectedConfigRP_r16_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SL-CBR-PriorityTxConfigList-r16.h"
#include "SL-Thres-RSRP-List-r16.h"
#include "SL-SelectionWindowList-r16.h"
#include "SL-ResourceReservePeriod-r16.h"
#include "SL-CBR-PriorityTxConfigList-v1650.h"

#endif	/* _SL_UE_SelectedConfigRP_r16_H_ */
#include <asn_internal.h>
