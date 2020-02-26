/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./new_out -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#include "PUCCH-ConfigCommon.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_pucch_ResourceCommon_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 15)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_hoppingId_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 1023)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_p0_nominal_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -202 && value <= 24)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_pucch_GroupHopping_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_pucch_GroupHopping_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_pucch_ResourceCommon_constr_2 CC_NOTUSED = {
	{ 1, 1 }	/* (0..15) */,
	-1};
static asn_per_constraints_t asn_PER_memb_pucch_ResourceCommon_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_hoppingId_constr_7 CC_NOTUSED = {
	{ 2, 1 }	/* (0..1023) */,
	-1};
static asn_per_constraints_t asn_PER_memb_hoppingId_constr_7 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 10,  10,  0,  1023 }	/* (0..1023) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_p0_nominal_constr_8 CC_NOTUSED = {
	{ 2, 0 }	/* (-202..24) */,
	-1};
static asn_per_constraints_t asn_PER_memb_p0_nominal_constr_8 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 8,  8, -202,  24 }	/* (-202..24) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_pucch_GroupHopping_value2enum_3[] = {
	{ 0,	7,	"neither" },
	{ 1,	6,	"enable" },
	{ 2,	7,	"disable" }
};
static const unsigned int asn_MAP_pucch_GroupHopping_enum2value_3[] = {
	2,	/* disable(2) */
	1,	/* enable(1) */
	0	/* neither(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_pucch_GroupHopping_specs_3 = {
	asn_MAP_pucch_GroupHopping_value2enum_3,	/* "tag" => N; sorted by tag */
	asn_MAP_pucch_GroupHopping_enum2value_3,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_pucch_GroupHopping_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_pucch_GroupHopping_3 = {
	"pucch-GroupHopping",
	"pucch-GroupHopping",
	&asn_OP_NativeEnumerated,
	asn_DEF_pucch_GroupHopping_tags_3,
	sizeof(asn_DEF_pucch_GroupHopping_tags_3)
		/sizeof(asn_DEF_pucch_GroupHopping_tags_3[0]) - 1, /* 1 */
	asn_DEF_pucch_GroupHopping_tags_3,	/* Same as above */
	sizeof(asn_DEF_pucch_GroupHopping_tags_3)
		/sizeof(asn_DEF_pucch_GroupHopping_tags_3[0]), /* 2 */
	{ &asn_OER_type_pucch_GroupHopping_constr_3, &asn_PER_type_pucch_GroupHopping_constr_3, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_pucch_GroupHopping_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_PUCCH_ConfigCommon_1[] = {
	{ ATF_POINTER, 1, offsetof(struct PUCCH_ConfigCommon, pucch_ResourceCommon),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_pucch_ResourceCommon_constr_2, &asn_PER_memb_pucch_ResourceCommon_constr_2,  memb_pucch_ResourceCommon_constraint_1 },
		0, 0, /* No default value */
		"pucch-ResourceCommon"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PUCCH_ConfigCommon, pucch_GroupHopping),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_pucch_GroupHopping_3,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pucch-GroupHopping"
		},
	{ ATF_POINTER, 2, offsetof(struct PUCCH_ConfigCommon, hoppingId),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_hoppingId_constr_7, &asn_PER_memb_hoppingId_constr_7,  memb_hoppingId_constraint_1 },
		0, 0, /* No default value */
		"hoppingId"
		},
	{ ATF_POINTER, 1, offsetof(struct PUCCH_ConfigCommon, p0_nominal),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_p0_nominal_constr_8, &asn_PER_memb_p0_nominal_constr_8,  memb_p0_nominal_constraint_1 },
		0, 0, /* No default value */
		"p0-nominal"
		},
};
static const int asn_MAP_PUCCH_ConfigCommon_oms_1[] = { 0, 2, 3 };
static const ber_tlv_tag_t asn_DEF_PUCCH_ConfigCommon_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PUCCH_ConfigCommon_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* pucch-ResourceCommon */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* pucch-GroupHopping */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* hoppingId */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* p0-nominal */
};
asn_SEQUENCE_specifics_t asn_SPC_PUCCH_ConfigCommon_specs_1 = {
	sizeof(struct PUCCH_ConfigCommon),
	offsetof(struct PUCCH_ConfigCommon, _asn_ctx),
	asn_MAP_PUCCH_ConfigCommon_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_PUCCH_ConfigCommon_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PUCCH_ConfigCommon = {
	"PUCCH-ConfigCommon",
	"PUCCH-ConfigCommon",
	&asn_OP_SEQUENCE,
	asn_DEF_PUCCH_ConfigCommon_tags_1,
	sizeof(asn_DEF_PUCCH_ConfigCommon_tags_1)
		/sizeof(asn_DEF_PUCCH_ConfigCommon_tags_1[0]), /* 1 */
	asn_DEF_PUCCH_ConfigCommon_tags_1,	/* Same as above */
	sizeof(asn_DEF_PUCCH_ConfigCommon_tags_1)
		/sizeof(asn_DEF_PUCCH_ConfigCommon_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_PUCCH_ConfigCommon_1,
	4,	/* Elements count */
	&asn_SPC_PUCCH_ConfigCommon_specs_1	/* Additional specs */
};

