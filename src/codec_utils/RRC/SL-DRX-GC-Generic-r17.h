/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_DRX_GC_Generic_r17_H_
#define	_SL_DRX_GC_Generic_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17 {
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_sl0	= 0,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_sl1	= 1,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_sl2	= 2,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_sl4	= 3,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_spare4	= 4,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_spare3	= 5,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_spare2	= 6,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17_spare1	= 7
} e_SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer1_r17;
typedef enum SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17 {
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_sl0	= 0,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_sl1	= 1,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_sl2	= 2,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_sl4	= 3,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_spare4	= 4,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_spare3	= 5,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_spare2	= 6,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17_spare1	= 7
} e_SL_DRX_GC_Generic_r17__sl_DRX_GC_HARQ_RTT_Timer2_r17;
typedef enum SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17 {
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl0	= 0,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl1	= 1,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl2	= 2,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl4	= 3,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl6	= 4,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl8	= 5,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl16	= 6,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl24	= 7,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl33	= 8,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl40	= 9,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl64	= 10,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl80	= 11,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl96	= 12,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl112	= 13,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl128	= 14,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl160	= 15,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_sl320	= 16,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare15	= 17,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare14	= 18,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare13	= 19,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare12	= 20,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare11	= 21,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare10	= 22,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare9	= 23,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare8	= 24,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare7	= 25,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare6	= 26,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare5	= 27,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare4	= 28,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare3	= 29,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare2	= 30,
	SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17_spare1	= 31
} e_SL_DRX_GC_Generic_r17__sl_DRX_GC_RetransmissionTimer_r17;

/* SL-DRX-GC-Generic-r17 */
typedef struct SL_DRX_GC_Generic_r17 {
	long	*sl_DRX_GC_HARQ_RTT_Timer1_r17;	/* OPTIONAL */
	long	*sl_DRX_GC_HARQ_RTT_Timer2_r17;	/* OPTIONAL */
	long	 sl_DRX_GC_RetransmissionTimer_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_DRX_GC_Generic_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_DRX_GC_HARQ_RTT_Timer1_r17_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_DRX_GC_HARQ_RTT_Timer2_r17_11;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_DRX_GC_RetransmissionTimer_r17_20;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_DRX_GC_Generic_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_DRX_GC_Generic_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_DRX_GC_Generic_r17_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _SL_DRX_GC_Generic_r17_H_ */
#include <asn_internal.h>
