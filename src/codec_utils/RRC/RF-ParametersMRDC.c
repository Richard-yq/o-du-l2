/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "RF-ParametersMRDC.h"

asn_TYPE_member_t asn_MBR_RF_ParametersMRDC_1[] = {
	{ ATF_POINTER, 2, offsetof(struct RF_ParametersMRDC, supportedBandCombinationList),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BandCombinationList,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"supportedBandCombinationList"
		},
	{ ATF_POINTER, 1, offsetof(struct RF_ParametersMRDC, appliedFreqBandListFilter),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FreqBandList,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"appliedFreqBandListFilter"
		},
};
static const int asn_MAP_RF_ParametersMRDC_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_RF_ParametersMRDC_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RF_ParametersMRDC_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* supportedBandCombinationList */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* appliedFreqBandListFilter */
};
asn_SEQUENCE_specifics_t asn_SPC_RF_ParametersMRDC_specs_1 = {
	sizeof(struct RF_ParametersMRDC),
	offsetof(struct RF_ParametersMRDC, _asn_ctx),
	asn_MAP_RF_ParametersMRDC_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_RF_ParametersMRDC_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	2,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RF_ParametersMRDC = {
	"RF-ParametersMRDC",
	"RF-ParametersMRDC",
	&asn_OP_SEQUENCE,
	asn_DEF_RF_ParametersMRDC_tags_1,
	sizeof(asn_DEF_RF_ParametersMRDC_tags_1)
		/sizeof(asn_DEF_RF_ParametersMRDC_tags_1[0]), /* 1 */
	asn_DEF_RF_ParametersMRDC_tags_1,	/* Same as above */
	sizeof(asn_DEF_RF_ParametersMRDC_tags_1)
		/sizeof(asn_DEF_RF_ParametersMRDC_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_RF_ParametersMRDC_1,
	2,	/* Elements count */
	&asn_SPC_RF_ParametersMRDC_specs_1	/* Additional specs */
};

