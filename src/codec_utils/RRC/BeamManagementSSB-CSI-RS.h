/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BeamManagementSSB_CSI_RS_H_
#define	_BeamManagementSSB_CSI_RS_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceOneTx {
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceOneTx_n8	= 0,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceOneTx_n16	= 1,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceOneTx_n32	= 2,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceOneTx_n64	= 3
} e_BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceOneTx;
typedef enum BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx {
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx_n0	= 0,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx_n4	= 1,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx_n8	= 2,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx_n16	= 3,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx_n32	= 4,
	BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx_n64	= 5
} e_BeamManagementSSB_CSI_RS__maxNumberSSB_CSI_RS_ResourceTwoTx;
typedef enum BeamManagementSSB_CSI_RS__supportedCSI_RS_Density {
	BeamManagementSSB_CSI_RS__supportedCSI_RS_Density_one	= 0,
	BeamManagementSSB_CSI_RS__supportedCSI_RS_Density_three	= 1,
	BeamManagementSSB_CSI_RS__supportedCSI_RS_Density_oneAndThree	= 2
} e_BeamManagementSSB_CSI_RS__supportedCSI_RS_Density;

/* BeamManagementSSB-CSI-RS */
typedef struct BeamManagementSSB_CSI_RS {
	long	 maxNumberSSB_CSI_RS_ResourceOneTx;
	long	 maxNumberSSB_CSI_RS_ResourceTwoTx;
	long	 supportedCSI_RS_Density;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BeamManagementSSB_CSI_RS_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_7;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_supportedCSI_RS_Density_14;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_BeamManagementSSB_CSI_RS;
extern asn_SEQUENCE_specifics_t asn_SPC_BeamManagementSSB_CSI_RS_specs_1;
extern asn_TYPE_member_t asn_MBR_BeamManagementSSB_CSI_RS_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _BeamManagementSSB_CSI_RS_H_ */
#include <asn_internal.h>
