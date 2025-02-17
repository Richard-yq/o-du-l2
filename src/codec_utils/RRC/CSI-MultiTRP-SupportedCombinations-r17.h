/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CSI_MultiTRP_SupportedCombinations_r17_H_
#define	_CSI_MultiTRP_SupportedCombinations_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17 {
	CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17_n2	= 0,
	CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17_n4	= 1,
	CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17_n8	= 2,
	CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17_n12	= 3,
	CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17_n16	= 4,
	CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17_n24	= 5,
	CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17_n32	= 6
} e_CSI_MultiTRP_SupportedCombinations_r17__maxNumTx_Ports_r17;

/* CSI-MultiTRP-SupportedCombinations-r17 */
typedef struct CSI_MultiTRP_SupportedCombinations_r17 {
	long	 maxNumTx_Ports_r17;
	long	 maxTotalNumCMR_r17;
	long	 maxTotalNumTx_PortsNZP_CSI_RS_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CSI_MultiTRP_SupportedCombinations_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumTx_Ports_r17_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CSI_MultiTRP_SupportedCombinations_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_CSI_MultiTRP_SupportedCombinations_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_CSI_MultiTRP_SupportedCombinations_r17_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _CSI_MultiTRP_SupportedCombinations_r17_H_ */
#include <asn_internal.h>
