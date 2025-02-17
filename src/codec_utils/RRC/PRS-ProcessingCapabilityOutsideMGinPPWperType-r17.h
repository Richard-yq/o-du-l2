/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17_H_
#define	_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__prsProcessingType_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__prsProcessingType_r17_type1A	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__prsProcessingType_r17_type1B	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__prsProcessingType_r17_type2	= 2
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__prsProcessingType_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_dl_PRS_BufferType_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_dl_PRS_BufferType_r17_type1	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_dl_PRS_BufferType_r17_type2	= 1
	/*
	 * Enumeration is extensible
	 */
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_dl_PRS_BufferType_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17_PR {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17_PR_NOTHING,	/* No components present */
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17_PR_ppw_durationOfPRS_Processing1_r17,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17_PR_ppw_durationOfPRS_Processing2_r17
} PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17_PR;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_msDot125	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_msDot25	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_msDot5	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms1	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms2	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms4	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms6	= 6,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms8	= 7,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms12	= 8,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms16	= 9,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms20	= 10,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms25	= 11,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms30	= 12,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms32	= 13,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms35	= 14,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms40	= 15,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms45	= 16,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17_ms50	= 17
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsN_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms1	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms2	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms4	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms8	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms16	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms20	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms30	= 6,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms40	= 7,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms80	= 8,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms160	= 9,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms320	= 10,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms640	= 11,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17_ms1280	= 12
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17__ppw_durationOfPRS_ProcessingSymbolsT_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_msDot125	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_msDot25	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_msDot5	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms1	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms2	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms3	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms4	= 6,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms5	= 7,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms6	= 8,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms8	= 9,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17_ms12	= 10
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsN2_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsT2_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsT2_r17_ms4	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsT2_r17_ms5	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsT2_r17_ms6	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsT2_r17_ms8	= 3
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17__ppw_durationOfPRS_ProcessingSymbolsT2_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n1	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n2	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n4	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n6	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n8	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n12	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n16	= 6,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n24	= 7,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n32	= 8,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n48	= 9,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17_n64	= 10
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs15_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n1	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n2	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n4	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n6	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n8	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n12	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n16	= 6,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n24	= 7,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n32	= 8,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n48	= 9,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17_n64	= 10
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs30_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n1	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n2	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n4	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n6	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n8	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n12	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n16	= 6,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n24	= 7,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n32	= 8,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n48	= 9,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17_n64	= 10
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs60_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n1	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n2	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n4	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n6	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n8	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n12	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n16	= 6,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n24	= 7,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n32	= 8,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n48	= 9,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17_n64	= 10
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17__scs120_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17_PR {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17_PR_NOTHING,	/* No components present */
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17_PR_fr1_r17,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17_PR_fr2_r17
} PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17_PR;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17_mhz5	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17_mhz10	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17_mhz20	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17_mhz40	= 3,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17_mhz50	= 4,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17_mhz80	= 5,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17_mhz100	= 6
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr1_r17;
typedef enum PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr2_r17 {
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr2_r17_mhz50	= 0,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr2_r17_mhz100	= 1,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr2_r17_mhz200	= 2,
	PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr2_r17_mhz400	= 3
} e_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17__fr2_r17;

/* PRS-ProcessingCapabilityOutsideMGinPPWperType-r17 */
typedef struct PRS_ProcessingCapabilityOutsideMGinPPWperType_r17 {
	long	 prsProcessingType_r17;
	long	 ppw_dl_PRS_BufferType_r17;
	struct PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17 {
		PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17_PR present;
		union PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17_u {
			struct PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing1_r17 {
				long	 ppw_durationOfPRS_ProcessingSymbolsN_r17;
				long	 ppw_durationOfPRS_ProcessingSymbolsT_r17;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *ppw_durationOfPRS_Processing1_r17;
			struct PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_durationOfPRS_Processing_r17__ppw_durationOfPRS_Processing2_r17 {
				long	 ppw_durationOfPRS_ProcessingSymbolsN2_r17;
				long	 ppw_durationOfPRS_ProcessingSymbolsT2_r17;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *ppw_durationOfPRS_Processing2_r17;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ppw_durationOfPRS_Processing_r17;
	struct PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17 {
		long	*scs15_r17;	/* OPTIONAL */
		long	*scs30_r17;	/* OPTIONAL */
		long	*scs60_r17;	/* OPTIONAL */
		long	*scs120_r17;	/* OPTIONAL */
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} ppw_maxNumOfDL_PRS_ResProcessedPerSlot_r17;
	struct PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17 {
		PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17_PR present;
		union PRS_ProcessingCapabilityOutsideMGinPPWperType_r17__ppw_maxNumOfDL_Bandwidth_r17_u {
			long	 fr1_r17;
			long	 fr2_r17;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ppw_maxNumOfDL_Bandwidth_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PRS_ProcessingCapabilityOutsideMGinPPWperType_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_prsProcessingType_r17_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ppw_dl_PRS_BufferType_r17_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ppw_durationOfPRS_ProcessingSymbolsN_r17_12;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ppw_durationOfPRS_ProcessingSymbolsT_r17_31;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ppw_durationOfPRS_ProcessingSymbolsN2_r17_46;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ppw_durationOfPRS_ProcessingSymbolsT2_r17_58;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs15_r17_64;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs30_r17_76;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs60_r17_88;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs120_r17_100;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_fr1_r17_114;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_fr2_r17_122;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_PRS_ProcessingCapabilityOutsideMGinPPWperType_r17_1[5];

#ifdef __cplusplus
}
#endif

#endif	/* _PRS_ProcessingCapabilityOutsideMGinPPWperType_r17_H_ */
#include <asn_internal.h>
