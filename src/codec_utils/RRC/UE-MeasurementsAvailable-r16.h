/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UE_MeasurementsAvailable_r16_H_
#define	_UE_MeasurementsAvailable_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UE_MeasurementsAvailable_r16__logMeasAvailable_r16 {
	UE_MeasurementsAvailable_r16__logMeasAvailable_r16_true	= 0
} e_UE_MeasurementsAvailable_r16__logMeasAvailable_r16;
typedef enum UE_MeasurementsAvailable_r16__logMeasAvailableBT_r16 {
	UE_MeasurementsAvailable_r16__logMeasAvailableBT_r16_true	= 0
} e_UE_MeasurementsAvailable_r16__logMeasAvailableBT_r16;
typedef enum UE_MeasurementsAvailable_r16__logMeasAvailableWLAN_r16 {
	UE_MeasurementsAvailable_r16__logMeasAvailableWLAN_r16_true	= 0
} e_UE_MeasurementsAvailable_r16__logMeasAvailableWLAN_r16;
typedef enum UE_MeasurementsAvailable_r16__connEstFailInfoAvailable_r16 {
	UE_MeasurementsAvailable_r16__connEstFailInfoAvailable_r16_true	= 0
} e_UE_MeasurementsAvailable_r16__connEstFailInfoAvailable_r16;
typedef enum UE_MeasurementsAvailable_r16__rlf_InfoAvailable_r16 {
	UE_MeasurementsAvailable_r16__rlf_InfoAvailable_r16_true	= 0
} e_UE_MeasurementsAvailable_r16__rlf_InfoAvailable_r16;
typedef enum UE_MeasurementsAvailable_r16__ext1__successHO_InfoAvailable_r17 {
	UE_MeasurementsAvailable_r16__ext1__successHO_InfoAvailable_r17_true	= 0
} e_UE_MeasurementsAvailable_r16__ext1__successHO_InfoAvailable_r17;

/* UE-MeasurementsAvailable-r16 */
typedef struct UE_MeasurementsAvailable_r16 {
	long	*logMeasAvailable_r16;	/* OPTIONAL */
	long	*logMeasAvailableBT_r16;	/* OPTIONAL */
	long	*logMeasAvailableWLAN_r16;	/* OPTIONAL */
	long	*connEstFailInfoAvailable_r16;	/* OPTIONAL */
	long	*rlf_InfoAvailable_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct UE_MeasurementsAvailable_r16__ext1 {
		long	*successHO_InfoAvailable_r17;	/* OPTIONAL */
		BOOLEAN_t	*sigLogMeasConfigAvailable_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UE_MeasurementsAvailable_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_logMeasAvailable_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_logMeasAvailableBT_r16_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_logMeasAvailableWLAN_r16_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_connEstFailInfoAvailable_r16_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_rlf_InfoAvailable_r16_10;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_successHO_InfoAvailable_r17_14;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_UE_MeasurementsAvailable_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_UE_MeasurementsAvailable_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_UE_MeasurementsAvailable_r16_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _UE_MeasurementsAvailable_r16_H_ */
#include <asn_internal.h>
