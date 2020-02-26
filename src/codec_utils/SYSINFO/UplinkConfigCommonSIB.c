/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./new_out -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#include "UplinkConfigCommonSIB.h"

asn_TYPE_member_t asn_MBR_UplinkConfigCommonSIB_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct UplinkConfigCommonSIB, frequencyInfoUL),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FrequencyInfoUL_SIB,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"frequencyInfoUL"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct UplinkConfigCommonSIB, initialUplinkBWP),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BWP_UplinkCommon,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"initialUplinkBWP"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct UplinkConfigCommonSIB, timeAlignmentTimerCommon),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TimeAlignmentTimer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"timeAlignmentTimerCommon"
		},
};
static const ber_tlv_tag_t asn_DEF_UplinkConfigCommonSIB_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_UplinkConfigCommonSIB_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* frequencyInfoUL */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* initialUplinkBWP */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* timeAlignmentTimerCommon */
};
asn_SEQUENCE_specifics_t asn_SPC_UplinkConfigCommonSIB_specs_1 = {
	sizeof(struct UplinkConfigCommonSIB),
	offsetof(struct UplinkConfigCommonSIB, _asn_ctx),
	asn_MAP_UplinkConfigCommonSIB_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_UplinkConfigCommonSIB = {
	"UplinkConfigCommonSIB",
	"UplinkConfigCommonSIB",
	&asn_OP_SEQUENCE,
	asn_DEF_UplinkConfigCommonSIB_tags_1,
	sizeof(asn_DEF_UplinkConfigCommonSIB_tags_1)
		/sizeof(asn_DEF_UplinkConfigCommonSIB_tags_1[0]), /* 1 */
	asn_DEF_UplinkConfigCommonSIB_tags_1,	/* Same as above */
	sizeof(asn_DEF_UplinkConfigCommonSIB_tags_1)
		/sizeof(asn_DEF_UplinkConfigCommonSIB_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_UplinkConfigCommonSIB_1,
	3,	/* Elements count */
	&asn_SPC_UplinkConfigCommonSIB_specs_1	/* Additional specs */
};

