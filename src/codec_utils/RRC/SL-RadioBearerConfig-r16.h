/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_RadioBearerConfig_r16_H_
#define	_SL_RadioBearerConfig_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SLRB-Uu-ConfigIndex-r16.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_RadioBearerConfig_r16__sl_TransRange_r16 {
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m20	= 0,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m50	= 1,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m80	= 2,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m100	= 3,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m120	= 4,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m150	= 5,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m180	= 6,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m200	= 7,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m220	= 8,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m250	= 9,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m270	= 10,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m300	= 11,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m350	= 12,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m370	= 13,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m400	= 14,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m420	= 15,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m450	= 16,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m480	= 17,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m500	= 18,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m550	= 19,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m600	= 20,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m700	= 21,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_m1000	= 22,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare9	= 23,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare8	= 24,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare7	= 25,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare6	= 26,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare5	= 27,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare4	= 28,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare3	= 29,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare2	= 30,
	SL_RadioBearerConfig_r16__sl_TransRange_r16_spare1	= 31
} e_SL_RadioBearerConfig_r16__sl_TransRange_r16;

/* Forward declarations */
struct SL_SDAP_Config_r16;
struct SL_PDCP_Config_r16;

/* SL-RadioBearerConfig-r16 */
typedef struct SL_RadioBearerConfig_r16 {
	SLRB_Uu_ConfigIndex_r16_t	 slrb_Uu_ConfigIndex_r16;
	struct SL_SDAP_Config_r16	*sl_SDAP_Config_r16;	/* OPTIONAL */
	struct SL_PDCP_Config_r16	*sl_PDCP_Config_r16;	/* OPTIONAL */
	long	*sl_TransRange_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_RadioBearerConfig_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_TransRange_r16_5;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_RadioBearerConfig_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_RadioBearerConfig_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_RadioBearerConfig_r16_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SL-SDAP-Config-r16.h"
#include "SL-PDCP-Config-r16.h"

#endif	/* _SL_RadioBearerConfig_r16_H_ */
#include <asn_internal.h>
