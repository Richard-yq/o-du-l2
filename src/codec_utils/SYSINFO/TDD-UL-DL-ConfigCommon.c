/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./new_out -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#include "TDD-UL-DL-ConfigCommon.h"

#include "TDD-UL-DL-Pattern.h"
asn_TYPE_member_t asn_MBR_TDD_UL_DL_ConfigCommon_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct TDD_UL_DL_ConfigCommon, referenceSubcarrierSpacing),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SubcarrierSpacing,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"referenceSubcarrierSpacing"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TDD_UL_DL_ConfigCommon, pattern1),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TDD_UL_DL_Pattern,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pattern1"
		},
	{ ATF_POINTER, 1, offsetof(struct TDD_UL_DL_ConfigCommon, pattern2),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TDD_UL_DL_Pattern,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pattern2"
		},
};
static const int asn_MAP_TDD_UL_DL_ConfigCommon_oms_1[] = { 2 };
static const ber_tlv_tag_t asn_DEF_TDD_UL_DL_ConfigCommon_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_TDD_UL_DL_ConfigCommon_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* referenceSubcarrierSpacing */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* pattern1 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* pattern2 */
};
asn_SEQUENCE_specifics_t asn_SPC_TDD_UL_DL_ConfigCommon_specs_1 = {
	sizeof(struct TDD_UL_DL_ConfigCommon),
	offsetof(struct TDD_UL_DL_ConfigCommon, _asn_ctx),
	asn_MAP_TDD_UL_DL_ConfigCommon_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_TDD_UL_DL_ConfigCommon_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_TDD_UL_DL_ConfigCommon = {
	"TDD-UL-DL-ConfigCommon",
	"TDD-UL-DL-ConfigCommon",
	&asn_OP_SEQUENCE,
	asn_DEF_TDD_UL_DL_ConfigCommon_tags_1,
	sizeof(asn_DEF_TDD_UL_DL_ConfigCommon_tags_1)
		/sizeof(asn_DEF_TDD_UL_DL_ConfigCommon_tags_1[0]), /* 1 */
	asn_DEF_TDD_UL_DL_ConfigCommon_tags_1,	/* Same as above */
	sizeof(asn_DEF_TDD_UL_DL_ConfigCommon_tags_1)
		/sizeof(asn_DEF_TDD_UL_DL_ConfigCommon_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_TDD_UL_DL_ConfigCommon_1,
	3,	/* Elements count */
	&asn_SPC_TDD_UL_DL_ConfigCommon_specs_1	/* Additional specs */
};

