/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CodebookConfig_r16_H_
#define	_CodebookConfig_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <BIT_STRING.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CodebookConfig_r16__codebookType_PR {
	CodebookConfig_r16__codebookType_PR_NOTHING,	/* No components present */
	CodebookConfig_r16__codebookType_PR_type2
} CodebookConfig_r16__codebookType_PR;
typedef enum CodebookConfig_r16__codebookType__type2__subType_PR {
	CodebookConfig_r16__codebookType__type2__subType_PR_NOTHING,	/* No components present */
	CodebookConfig_r16__codebookType__type2__subType_PR_typeII_r16,
	CodebookConfig_r16__codebookType__type2__subType_PR_typeII_PortSelection_r16
} CodebookConfig_r16__codebookType__type2__subType_PR;
typedef enum CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR {
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_NOTHING,	/* No components present */
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_two_one,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_two_two,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_four_one,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_three_two,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_six_one,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_four_two,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_eight_one,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_four_three,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_six_two,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_twelve_one,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_four_four,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_eight_two,
	CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR_sixteen_one
} CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR;
typedef enum CodebookConfig_r16__codebookType__type2__subType__typeII_PortSelection_r16__portSelectionSamplingSize_r16 {
	CodebookConfig_r16__codebookType__type2__subType__typeII_PortSelection_r16__portSelectionSamplingSize_r16_n1	= 0,
	CodebookConfig_r16__codebookType__type2__subType__typeII_PortSelection_r16__portSelectionSamplingSize_r16_n2	= 1,
	CodebookConfig_r16__codebookType__type2__subType__typeII_PortSelection_r16__portSelectionSamplingSize_r16_n3	= 2,
	CodebookConfig_r16__codebookType__type2__subType__typeII_PortSelection_r16__portSelectionSamplingSize_r16_n4	= 3
} e_CodebookConfig_r16__codebookType__type2__subType__typeII_PortSelection_r16__portSelectionSamplingSize_r16;

/* CodebookConfig-r16 */
typedef struct CodebookConfig_r16 {
	struct CodebookConfig_r16__codebookType {
		CodebookConfig_r16__codebookType_PR present;
		union CodebookConfig_r16__codebookType_u {
			struct CodebookConfig_r16__codebookType__type2 {
				struct CodebookConfig_r16__codebookType__type2__subType {
					CodebookConfig_r16__codebookType__type2__subType_PR present;
					union CodebookConfig_r16__codebookType__type2__subType_u {
						struct CodebookConfig_r16__codebookType__type2__subType__typeII_r16 {
							struct CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16 {
								CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_PR present;
								union CodebookConfig_r16__codebookType__type2__subType__typeII_r16__n1_n2_codebookSubsetRestriction_r16_u {
									BIT_STRING_t	 two_one;
									BIT_STRING_t	 two_two;
									BIT_STRING_t	 four_one;
									BIT_STRING_t	 three_two;
									BIT_STRING_t	 six_one;
									BIT_STRING_t	 four_two;
									BIT_STRING_t	 eight_one;
									BIT_STRING_t	 four_three;
									BIT_STRING_t	 six_two;
									BIT_STRING_t	 twelve_one;
									BIT_STRING_t	 four_four;
									BIT_STRING_t	 eight_two;
									BIT_STRING_t	 sixteen_one;
								} choice;
								
								/* Context for parsing across buffer boundaries */
								asn_struct_ctx_t _asn_ctx;
							} n1_n2_codebookSubsetRestriction_r16;
							BIT_STRING_t	 typeII_RI_Restriction_r16;
							
							/* Context for parsing across buffer boundaries */
							asn_struct_ctx_t _asn_ctx;
						} *typeII_r16;
						struct CodebookConfig_r16__codebookType__type2__subType__typeII_PortSelection_r16 {
							long	 portSelectionSamplingSize_r16;
							BIT_STRING_t	 typeII_PortSelectionRI_Restriction_r16;
							
							/* Context for parsing across buffer boundaries */
							asn_struct_ctx_t _asn_ctx;
						} *typeII_PortSelection_r16;
					} choice;
					
					/* Context for parsing across buffer boundaries */
					asn_struct_ctx_t _asn_ctx;
				} subType;
				long	 numberOfPMI_SubbandsPerCQI_Subband_r16;
				long	 paramCombination_r16;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *type2;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} codebookType;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CodebookConfig_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_portSelectionSamplingSize_r16_22;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CodebookConfig_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_CodebookConfig_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_CodebookConfig_r16_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _CodebookConfig_r16_H_ */
#include <asn_internal.h>
