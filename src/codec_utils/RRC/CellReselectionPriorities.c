/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "CellReselectionPriorities.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_t320_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_t320_value2enum_4[] = {
	{ 0,	4,	"min5" },
	{ 1,	5,	"min10" },
	{ 2,	5,	"min20" },
	{ 3,	5,	"min30" },
	{ 4,	5,	"min60" },
	{ 5,	6,	"min120" },
	{ 6,	6,	"min180" },
	{ 7,	6,	"spare1" }
};
static const unsigned int asn_MAP_t320_enum2value_4[] = {
	1,	/* min10(1) */
	5,	/* min120(5) */
	6,	/* min180(6) */
	2,	/* min20(2) */
	3,	/* min30(3) */
	0,	/* min5(0) */
	4,	/* min60(4) */
	7	/* spare1(7) */
};
static const asn_INTEGER_specifics_t asn_SPC_t320_specs_4 = {
	asn_MAP_t320_value2enum_4,	/* "tag" => N; sorted by tag */
	asn_MAP_t320_enum2value_4,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_t320_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_t320_4 = {
	"t320",
	"t320",
	&asn_OP_NativeEnumerated,
	asn_DEF_t320_tags_4,
	sizeof(asn_DEF_t320_tags_4)
		/sizeof(asn_DEF_t320_tags_4[0]) - 1, /* 1 */
	asn_DEF_t320_tags_4,	/* Same as above */
	sizeof(asn_DEF_t320_tags_4)
		/sizeof(asn_DEF_t320_tags_4[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_t320_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_t320_specs_4	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_CellReselectionPriorities_1[] = {
	{ ATF_POINTER, 3, offsetof(struct CellReselectionPriorities, freqPriorityListEUTRA),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FreqPriorityListEUTRA,
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
		"freqPriorityListEUTRA"
		},
	{ ATF_POINTER, 2, offsetof(struct CellReselectionPriorities, freqPriorityListNR),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FreqPriorityListNR,
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
		"freqPriorityListNR"
		},
	{ ATF_POINTER, 1, offsetof(struct CellReselectionPriorities, t320),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_t320_4,
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
		"t320"
		},
};
static const int asn_MAP_CellReselectionPriorities_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_CellReselectionPriorities_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CellReselectionPriorities_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* freqPriorityListEUTRA */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* freqPriorityListNR */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* t320 */
};
asn_SEQUENCE_specifics_t asn_SPC_CellReselectionPriorities_specs_1 = {
	sizeof(struct CellReselectionPriorities),
	offsetof(struct CellReselectionPriorities, _asn_ctx),
	asn_MAP_CellReselectionPriorities_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_CellReselectionPriorities_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CellReselectionPriorities = {
	"CellReselectionPriorities",
	"CellReselectionPriorities",
	&asn_OP_SEQUENCE,
	asn_DEF_CellReselectionPriorities_tags_1,
	sizeof(asn_DEF_CellReselectionPriorities_tags_1)
		/sizeof(asn_DEF_CellReselectionPriorities_tags_1[0]), /* 1 */
	asn_DEF_CellReselectionPriorities_tags_1,	/* Same as above */
	sizeof(asn_DEF_CellReselectionPriorities_tags_1)
		/sizeof(asn_DEF_CellReselectionPriorities_tags_1[0]), /* 1 */
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
	asn_MBR_CellReselectionPriorities_1,
	3,	/* Elements count */
	&asn_SPC_CellReselectionPriorities_specs_1	/* Additional specs */
};

