/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "BeamManagementSSB-CSI-RS.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_maxNumberSSB_CSI_RS_ResourceOneTx_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  3 }	/* (0..3) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_maxNumberSSB_CSI_RS_ResourceTwoTx_constr_7 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  5 }	/* (0..5) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_supportedCSI_RS_Density_constr_14 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_maxNumberSSB_CSI_RS_ResourceOneTx_value2enum_2[] = {
	{ 0,	2,	"n8" },
	{ 1,	3,	"n16" },
	{ 2,	3,	"n32" },
	{ 3,	3,	"n64" }
};
static const unsigned int asn_MAP_maxNumberSSB_CSI_RS_ResourceOneTx_enum2value_2[] = {
	1,	/* n16(1) */
	2,	/* n32(2) */
	3,	/* n64(3) */
	0	/* n8(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_maxNumberSSB_CSI_RS_ResourceOneTx_specs_2 = {
	asn_MAP_maxNumberSSB_CSI_RS_ResourceOneTx_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_maxNumberSSB_CSI_RS_ResourceOneTx_enum2value_2,	/* N => "tag"; sorted by N */
	4,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_2 = {
	"maxNumberSSB-CSI-RS-ResourceOneTx",
	"maxNumberSSB-CSI-RS-ResourceOneTx",
	&asn_OP_NativeEnumerated,
	asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_tags_2,
	sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_tags_2)
		/sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_tags_2[0]) - 1, /* 1 */
	asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_tags_2,	/* Same as above */
	sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_tags_2)
		/sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_maxNumberSSB_CSI_RS_ResourceOneTx_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_maxNumberSSB_CSI_RS_ResourceOneTx_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_maxNumberSSB_CSI_RS_ResourceTwoTx_value2enum_7[] = {
	{ 0,	2,	"n0" },
	{ 1,	2,	"n4" },
	{ 2,	2,	"n8" },
	{ 3,	3,	"n16" },
	{ 4,	3,	"n32" },
	{ 5,	3,	"n64" }
};
static const unsigned int asn_MAP_maxNumberSSB_CSI_RS_ResourceTwoTx_enum2value_7[] = {
	0,	/* n0(0) */
	3,	/* n16(3) */
	4,	/* n32(4) */
	1,	/* n4(1) */
	5,	/* n64(5) */
	2	/* n8(2) */
};
static const asn_INTEGER_specifics_t asn_SPC_maxNumberSSB_CSI_RS_ResourceTwoTx_specs_7 = {
	asn_MAP_maxNumberSSB_CSI_RS_ResourceTwoTx_value2enum_7,	/* "tag" => N; sorted by tag */
	asn_MAP_maxNumberSSB_CSI_RS_ResourceTwoTx_enum2value_7,	/* N => "tag"; sorted by N */
	6,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_tags_7[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_7 = {
	"maxNumberSSB-CSI-RS-ResourceTwoTx",
	"maxNumberSSB-CSI-RS-ResourceTwoTx",
	&asn_OP_NativeEnumerated,
	asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_tags_7,
	sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_tags_7)
		/sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_tags_7[0]) - 1, /* 1 */
	asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_tags_7,	/* Same as above */
	sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_tags_7)
		/sizeof(asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_tags_7[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_maxNumberSSB_CSI_RS_ResourceTwoTx_constr_7,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_maxNumberSSB_CSI_RS_ResourceTwoTx_specs_7	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_supportedCSI_RS_Density_value2enum_14[] = {
	{ 0,	3,	"one" },
	{ 1,	5,	"three" },
	{ 2,	11,	"oneAndThree" }
};
static const unsigned int asn_MAP_supportedCSI_RS_Density_enum2value_14[] = {
	0,	/* one(0) */
	2,	/* oneAndThree(2) */
	1	/* three(1) */
};
static const asn_INTEGER_specifics_t asn_SPC_supportedCSI_RS_Density_specs_14 = {
	asn_MAP_supportedCSI_RS_Density_value2enum_14,	/* "tag" => N; sorted by tag */
	asn_MAP_supportedCSI_RS_Density_enum2value_14,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_supportedCSI_RS_Density_tags_14[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_supportedCSI_RS_Density_14 = {
	"supportedCSI-RS-Density",
	"supportedCSI-RS-Density",
	&asn_OP_NativeEnumerated,
	asn_DEF_supportedCSI_RS_Density_tags_14,
	sizeof(asn_DEF_supportedCSI_RS_Density_tags_14)
		/sizeof(asn_DEF_supportedCSI_RS_Density_tags_14[0]) - 1, /* 1 */
	asn_DEF_supportedCSI_RS_Density_tags_14,	/* Same as above */
	sizeof(asn_DEF_supportedCSI_RS_Density_tags_14)
		/sizeof(asn_DEF_supportedCSI_RS_Density_tags_14[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_supportedCSI_RS_Density_constr_14,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_supportedCSI_RS_Density_specs_14	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_BeamManagementSSB_CSI_RS_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct BeamManagementSSB_CSI_RS, maxNumberSSB_CSI_RS_ResourceOneTx),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_maxNumberSSB_CSI_RS_ResourceOneTx_2,
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
		"maxNumberSSB-CSI-RS-ResourceOneTx"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct BeamManagementSSB_CSI_RS, maxNumberSSB_CSI_RS_ResourceTwoTx),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_maxNumberSSB_CSI_RS_ResourceTwoTx_7,
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
		"maxNumberSSB-CSI-RS-ResourceTwoTx"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct BeamManagementSSB_CSI_RS, supportedCSI_RS_Density),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_supportedCSI_RS_Density_14,
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
		"supportedCSI-RS-Density"
		},
};
static const ber_tlv_tag_t asn_DEF_BeamManagementSSB_CSI_RS_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_BeamManagementSSB_CSI_RS_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* maxNumberSSB-CSI-RS-ResourceOneTx */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* maxNumberSSB-CSI-RS-ResourceTwoTx */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* supportedCSI-RS-Density */
};
asn_SEQUENCE_specifics_t asn_SPC_BeamManagementSSB_CSI_RS_specs_1 = {
	sizeof(struct BeamManagementSSB_CSI_RS),
	offsetof(struct BeamManagementSSB_CSI_RS, _asn_ctx),
	asn_MAP_BeamManagementSSB_CSI_RS_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_BeamManagementSSB_CSI_RS = {
	"BeamManagementSSB-CSI-RS",
	"BeamManagementSSB-CSI-RS",
	&asn_OP_SEQUENCE,
	asn_DEF_BeamManagementSSB_CSI_RS_tags_1,
	sizeof(asn_DEF_BeamManagementSSB_CSI_RS_tags_1)
		/sizeof(asn_DEF_BeamManagementSSB_CSI_RS_tags_1[0]), /* 1 */
	asn_DEF_BeamManagementSSB_CSI_RS_tags_1,	/* Same as above */
	sizeof(asn_DEF_BeamManagementSSB_CSI_RS_tags_1)
		/sizeof(asn_DEF_BeamManagementSSB_CSI_RS_tags_1[0]), /* 1 */
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
	asn_MBR_BeamManagementSSB_CSI_RS_1,
	3,	/* Elements count */
	&asn_SPC_BeamManagementSSB_CSI_RS_specs_1	/* Additional specs */
};

