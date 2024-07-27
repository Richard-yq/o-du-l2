/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "ReconfigurationWithSync.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_t304_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_rach_ConfigDedicated_constr_13 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_t304_value2enum_4[] = {
	{ 0,	4,	"ms50" },
	{ 1,	5,	"ms100" },
	{ 2,	5,	"ms150" },
	{ 3,	5,	"ms200" },
	{ 4,	5,	"ms500" },
	{ 5,	6,	"ms1000" },
	{ 6,	6,	"ms2000" },
	{ 7,	7,	"ms10000" }
};
static const unsigned int asn_MAP_t304_enum2value_4[] = {
	1,	/* ms100(1) */
	5,	/* ms1000(5) */
	7,	/* ms10000(7) */
	2,	/* ms150(2) */
	3,	/* ms200(3) */
	6,	/* ms2000(6) */
	0,	/* ms50(0) */
	4	/* ms500(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_t304_specs_4 = {
	asn_MAP_t304_value2enum_4,	/* "tag" => N; sorted by tag */
	asn_MAP_t304_enum2value_4,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_t304_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_t304_4 = {
	"t304",
	"t304",
	&asn_OP_NativeEnumerated,
	asn_DEF_t304_tags_4,
	sizeof(asn_DEF_t304_tags_4)
		/sizeof(asn_DEF_t304_tags_4[0]) - 1, /* 1 */
	asn_DEF_t304_tags_4,	/* Same as above */
	sizeof(asn_DEF_t304_tags_4)
		/sizeof(asn_DEF_t304_tags_4[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_t304_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_t304_specs_4	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_rach_ConfigDedicated_13[] = {
	{ ATF_POINTER, 0, offsetof(struct ReconfigurationWithSync__rach_ConfigDedicated, choice.uplink),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RACH_ConfigDedicated,
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
		"uplink"
		},
	{ ATF_POINTER, 0, offsetof(struct ReconfigurationWithSync__rach_ConfigDedicated, choice.supplementaryUplink),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RACH_ConfigDedicated,
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
		"supplementaryUplink"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_rach_ConfigDedicated_tag2el_13[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* uplink */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* supplementaryUplink */
};
static asn_CHOICE_specifics_t asn_SPC_rach_ConfigDedicated_specs_13 = {
	sizeof(struct ReconfigurationWithSync__rach_ConfigDedicated),
	offsetof(struct ReconfigurationWithSync__rach_ConfigDedicated, _asn_ctx),
	offsetof(struct ReconfigurationWithSync__rach_ConfigDedicated, present),
	sizeof(((struct ReconfigurationWithSync__rach_ConfigDedicated *)0)->present),
	asn_MAP_rach_ConfigDedicated_tag2el_13,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_rach_ConfigDedicated_13 = {
	"rach-ConfigDedicated",
	"rach-ConfigDedicated",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_rach_ConfigDedicated_constr_13,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		CHOICE_constraint
	},
	asn_MBR_rach_ConfigDedicated_13,
	2,	/* Elements count */
	&asn_SPC_rach_ConfigDedicated_specs_13	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_ext1_17[] = {
	{ ATF_POINTER, 1, offsetof(struct ReconfigurationWithSync__ext1, smtc),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SSB_MTC,
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
		"smtc"
		},
};
static const int asn_MAP_ext1_oms_17[] = { 0 };
static const ber_tlv_tag_t asn_DEF_ext1_tags_17[] = {
	(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ext1_tag2el_17[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* smtc */
};
static asn_SEQUENCE_specifics_t asn_SPC_ext1_specs_17 = {
	sizeof(struct ReconfigurationWithSync__ext1),
	offsetof(struct ReconfigurationWithSync__ext1, _asn_ctx),
	asn_MAP_ext1_tag2el_17,
	1,	/* Count of tags in the map */
	asn_MAP_ext1_oms_17,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ext1_17 = {
	"ext1",
	"ext1",
	&asn_OP_SEQUENCE,
	asn_DEF_ext1_tags_17,
	sizeof(asn_DEF_ext1_tags_17)
		/sizeof(asn_DEF_ext1_tags_17[0]) - 1, /* 1 */
	asn_DEF_ext1_tags_17,	/* Same as above */
	sizeof(asn_DEF_ext1_tags_17)
		/sizeof(asn_DEF_ext1_tags_17[0]), /* 2 */
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
	asn_MBR_ext1_17,
	1,	/* Elements count */
	&asn_SPC_ext1_specs_17	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_ReconfigurationWithSync_1[] = {
	{ ATF_POINTER, 1, offsetof(struct ReconfigurationWithSync, spCellConfigCommon),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ServingCellConfigCommon,
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
		"spCellConfigCommon"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ReconfigurationWithSync, newUE_Identity),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RNTI_Value,
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
		"newUE-Identity"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ReconfigurationWithSync, t304),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_t304_4,
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
		"t304"
		},
	{ ATF_POINTER, 2, offsetof(struct ReconfigurationWithSync, rach_ConfigDedicated),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_rach_ConfigDedicated_13,
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
		"rach-ConfigDedicated"
		},
	{ ATF_POINTER, 1, offsetof(struct ReconfigurationWithSync, ext1),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		0,
		&asn_DEF_ext1_17,
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
		"ext1"
		},
};
static const int asn_MAP_ReconfigurationWithSync_oms_1[] = { 0, 3, 4 };
static const ber_tlv_tag_t asn_DEF_ReconfigurationWithSync_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ReconfigurationWithSync_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* spCellConfigCommon */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* newUE-Identity */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* t304 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* rach-ConfigDedicated */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* ext1 */
};
asn_SEQUENCE_specifics_t asn_SPC_ReconfigurationWithSync_specs_1 = {
	sizeof(struct ReconfigurationWithSync),
	offsetof(struct ReconfigurationWithSync, _asn_ctx),
	asn_MAP_ReconfigurationWithSync_tag2el_1,
	5,	/* Count of tags in the map */
	asn_MAP_ReconfigurationWithSync_oms_1,	/* Optional members */
	2, 1,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ReconfigurationWithSync = {
	"ReconfigurationWithSync",
	"ReconfigurationWithSync",
	&asn_OP_SEQUENCE,
	asn_DEF_ReconfigurationWithSync_tags_1,
	sizeof(asn_DEF_ReconfigurationWithSync_tags_1)
		/sizeof(asn_DEF_ReconfigurationWithSync_tags_1[0]), /* 1 */
	asn_DEF_ReconfigurationWithSync_tags_1,	/* Same as above */
	sizeof(asn_DEF_ReconfigurationWithSync_tags_1)
		/sizeof(asn_DEF_ReconfigurationWithSync_tags_1[0]), /* 1 */
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
	asn_MBR_ReconfigurationWithSync_1,
	5,	/* Elements count */
	&asn_SPC_ReconfigurationWithSync_specs_1	/* Additional specs */
};

