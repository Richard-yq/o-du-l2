/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "QuantityConfigNR.h"

asn_TYPE_member_t asn_MBR_QuantityConfigNR_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct QuantityConfigNR, quantityConfigCell),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_QuantityConfigRS,
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
		"quantityConfigCell"
		},
	{ ATF_POINTER, 1, offsetof(struct QuantityConfigNR, quantityConfigRS_Index),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_QuantityConfigRS,
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
		"quantityConfigRS-Index"
		},
};
static const int asn_MAP_QuantityConfigNR_oms_1[] = { 1 };
static const ber_tlv_tag_t asn_DEF_QuantityConfigNR_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_QuantityConfigNR_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* quantityConfigCell */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* quantityConfigRS-Index */
};
asn_SEQUENCE_specifics_t asn_SPC_QuantityConfigNR_specs_1 = {
	sizeof(struct QuantityConfigNR),
	offsetof(struct QuantityConfigNR, _asn_ctx),
	asn_MAP_QuantityConfigNR_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_QuantityConfigNR_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_QuantityConfigNR = {
	"QuantityConfigNR",
	"QuantityConfigNR",
	&asn_OP_SEQUENCE,
	asn_DEF_QuantityConfigNR_tags_1,
	sizeof(asn_DEF_QuantityConfigNR_tags_1)
		/sizeof(asn_DEF_QuantityConfigNR_tags_1[0]), /* 1 */
	asn_DEF_QuantityConfigNR_tags_1,	/* Same as above */
	sizeof(asn_DEF_QuantityConfigNR_tags_1)
		/sizeof(asn_DEF_QuantityConfigNR_tags_1[0]), /* 1 */
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
	asn_MBR_QuantityConfigNR_1,
	2,	/* Elements count */
	&asn_SPC_QuantityConfigNR_specs_1	/* Additional specs */
};

