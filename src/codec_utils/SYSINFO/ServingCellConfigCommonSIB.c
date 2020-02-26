/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./new_out -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#include "ServingCellConfigCommonSIB.h"

#include "UplinkConfigCommonSIB.h"
#include "TDD-UL-DL-ConfigCommon.h"
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_inOneGroup_constraint_9(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const BIT_STRING_t *st = (const BIT_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	if(st->size > 0) {
		/* Size in bits */
		size = 8 * st->size - (st->bits_unused & 0x07);
	} else {
		size = 0;
	}
	
	if((size == 8)) {
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
memb_groupPresence_constraint_9(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const BIT_STRING_t *st = (const BIT_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	if(st->size > 0) {
		/* Size in bits */
		size = 8 * st->size - (st->bits_unused & 0x07);
	} else {
		size = 0;
	}
	
	if((size == 8)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_ss_PBCH_BlockPower_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -60 && value <= 50)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_n_TimingAdvanceOffset_constr_5 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_n_TimingAdvanceOffset_constr_5 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_inOneGroup_constr_10 CC_NOTUSED = {
	{ 0, 0 },
	8	/* (SIZE(8..8)) */};
static asn_per_constraints_t asn_PER_memb_inOneGroup_constr_10 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 0,  0,  8,  8 }	/* (SIZE(8..8)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_groupPresence_constr_11 CC_NOTUSED = {
	{ 0, 0 },
	8	/* (SIZE(8..8)) */};
static asn_per_constraints_t asn_PER_memb_groupPresence_constr_11 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 0,  0,  8,  8 }	/* (SIZE(8..8)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_ssb_PeriodicityServingCell_constr_12 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_ssb_PeriodicityServingCell_constr_12 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  5 }	/* (0..5) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_ss_PBCH_BlockPower_constr_20 CC_NOTUSED = {
	{ 1, 0 }	/* (-60..50) */,
	-1};
static asn_per_constraints_t asn_PER_memb_ss_PBCH_BlockPower_constr_20 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 7,  7, -60,  50 }	/* (-60..50) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_n_TimingAdvanceOffset_value2enum_5[] = {
	{ 0,	2,	"n0" },
	{ 1,	6,	"n25600" },
	{ 2,	6,	"n39936" }
};
static const unsigned int asn_MAP_n_TimingAdvanceOffset_enum2value_5[] = {
	0,	/* n0(0) */
	1,	/* n25600(1) */
	2	/* n39936(2) */
};
static const asn_INTEGER_specifics_t asn_SPC_n_TimingAdvanceOffset_specs_5 = {
	asn_MAP_n_TimingAdvanceOffset_value2enum_5,	/* "tag" => N; sorted by tag */
	asn_MAP_n_TimingAdvanceOffset_enum2value_5,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_n_TimingAdvanceOffset_tags_5[] = {
	(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_n_TimingAdvanceOffset_5 = {
	"n-TimingAdvanceOffset",
	"n-TimingAdvanceOffset",
	&asn_OP_NativeEnumerated,
	asn_DEF_n_TimingAdvanceOffset_tags_5,
	sizeof(asn_DEF_n_TimingAdvanceOffset_tags_5)
		/sizeof(asn_DEF_n_TimingAdvanceOffset_tags_5[0]) - 1, /* 1 */
	asn_DEF_n_TimingAdvanceOffset_tags_5,	/* Same as above */
	sizeof(asn_DEF_n_TimingAdvanceOffset_tags_5)
		/sizeof(asn_DEF_n_TimingAdvanceOffset_tags_5[0]), /* 2 */
	{ &asn_OER_type_n_TimingAdvanceOffset_constr_5, &asn_PER_type_n_TimingAdvanceOffset_constr_5, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_n_TimingAdvanceOffset_specs_5	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_ssb_PositionsInBurst_9[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ServingCellConfigCommonSIB__ssb_PositionsInBurst, inOneGroup),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BIT_STRING,
		0,
		{ &asn_OER_memb_inOneGroup_constr_10, &asn_PER_memb_inOneGroup_constr_10,  memb_inOneGroup_constraint_9 },
		0, 0, /* No default value */
		"inOneGroup"
		},
	{ ATF_POINTER, 1, offsetof(struct ServingCellConfigCommonSIB__ssb_PositionsInBurst, groupPresence),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BIT_STRING,
		0,
		{ &asn_OER_memb_groupPresence_constr_11, &asn_PER_memb_groupPresence_constr_11,  memb_groupPresence_constraint_9 },
		0, 0, /* No default value */
		"groupPresence"
		},
};
static const int asn_MAP_ssb_PositionsInBurst_oms_9[] = { 1 };
static const ber_tlv_tag_t asn_DEF_ssb_PositionsInBurst_tags_9[] = {
	(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ssb_PositionsInBurst_tag2el_9[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* inOneGroup */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* groupPresence */
};
static asn_SEQUENCE_specifics_t asn_SPC_ssb_PositionsInBurst_specs_9 = {
	sizeof(struct ServingCellConfigCommonSIB__ssb_PositionsInBurst),
	offsetof(struct ServingCellConfigCommonSIB__ssb_PositionsInBurst, _asn_ctx),
	asn_MAP_ssb_PositionsInBurst_tag2el_9,
	2,	/* Count of tags in the map */
	asn_MAP_ssb_PositionsInBurst_oms_9,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ssb_PositionsInBurst_9 = {
	"ssb-PositionsInBurst",
	"ssb-PositionsInBurst",
	&asn_OP_SEQUENCE,
	asn_DEF_ssb_PositionsInBurst_tags_9,
	sizeof(asn_DEF_ssb_PositionsInBurst_tags_9)
		/sizeof(asn_DEF_ssb_PositionsInBurst_tags_9[0]) - 1, /* 1 */
	asn_DEF_ssb_PositionsInBurst_tags_9,	/* Same as above */
	sizeof(asn_DEF_ssb_PositionsInBurst_tags_9)
		/sizeof(asn_DEF_ssb_PositionsInBurst_tags_9[0]), /* 2 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_ssb_PositionsInBurst_9,
	2,	/* Elements count */
	&asn_SPC_ssb_PositionsInBurst_specs_9	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_ssb_PeriodicityServingCell_value2enum_12[] = {
	{ 0,	3,	"ms5" },
	{ 1,	4,	"ms10" },
	{ 2,	4,	"ms20" },
	{ 3,	4,	"ms40" },
	{ 4,	4,	"ms80" },
	{ 5,	5,	"ms160" }
};
static const unsigned int asn_MAP_ssb_PeriodicityServingCell_enum2value_12[] = {
	1,	/* ms10(1) */
	5,	/* ms160(5) */
	2,	/* ms20(2) */
	3,	/* ms40(3) */
	0,	/* ms5(0) */
	4	/* ms80(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_ssb_PeriodicityServingCell_specs_12 = {
	asn_MAP_ssb_PeriodicityServingCell_value2enum_12,	/* "tag" => N; sorted by tag */
	asn_MAP_ssb_PeriodicityServingCell_enum2value_12,	/* N => "tag"; sorted by N */
	6,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_ssb_PeriodicityServingCell_tags_12[] = {
	(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ssb_PeriodicityServingCell_12 = {
	"ssb-PeriodicityServingCell",
	"ssb-PeriodicityServingCell",
	&asn_OP_NativeEnumerated,
	asn_DEF_ssb_PeriodicityServingCell_tags_12,
	sizeof(asn_DEF_ssb_PeriodicityServingCell_tags_12)
		/sizeof(asn_DEF_ssb_PeriodicityServingCell_tags_12[0]) - 1, /* 1 */
	asn_DEF_ssb_PeriodicityServingCell_tags_12,	/* Same as above */
	sizeof(asn_DEF_ssb_PeriodicityServingCell_tags_12)
		/sizeof(asn_DEF_ssb_PeriodicityServingCell_tags_12[0]), /* 2 */
	{ &asn_OER_type_ssb_PeriodicityServingCell_constr_12, &asn_PER_type_ssb_PeriodicityServingCell_constr_12, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_ssb_PeriodicityServingCell_specs_12	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_ServingCellConfigCommonSIB_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ServingCellConfigCommonSIB, downlinkConfigCommon),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DownlinkConfigCommonSIB,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"downlinkConfigCommon"
		},
	{ ATF_POINTER, 3, offsetof(struct ServingCellConfigCommonSIB, uplinkConfigCommon),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_UplinkConfigCommonSIB,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"uplinkConfigCommon"
		},
	{ ATF_POINTER, 2, offsetof(struct ServingCellConfigCommonSIB, supplementaryUplink),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_UplinkConfigCommonSIB,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"supplementaryUplink"
		},
	{ ATF_POINTER, 1, offsetof(struct ServingCellConfigCommonSIB, n_TimingAdvanceOffset),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_n_TimingAdvanceOffset_5,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"n-TimingAdvanceOffset"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ServingCellConfigCommonSIB, ssb_PositionsInBurst),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		0,
		&asn_DEF_ssb_PositionsInBurst_9,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"ssb-PositionsInBurst"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ServingCellConfigCommonSIB, ssb_PeriodicityServingCell),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ssb_PeriodicityServingCell_12,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"ssb-PeriodicityServingCell"
		},
	{ ATF_POINTER, 1, offsetof(struct ServingCellConfigCommonSIB, tdd_UL_DL_ConfigurationCommon),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TDD_UL_DL_ConfigCommon,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"tdd-UL-DL-ConfigurationCommon"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ServingCellConfigCommonSIB, ss_PBCH_BlockPower),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_ss_PBCH_BlockPower_constr_20, &asn_PER_memb_ss_PBCH_BlockPower_constr_20,  memb_ss_PBCH_BlockPower_constraint_1 },
		0, 0, /* No default value */
		"ss-PBCH-BlockPower"
		},
};
static const int asn_MAP_ServingCellConfigCommonSIB_oms_1[] = { 1, 2, 3, 6 };
static const ber_tlv_tag_t asn_DEF_ServingCellConfigCommonSIB_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ServingCellConfigCommonSIB_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* downlinkConfigCommon */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* uplinkConfigCommon */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* supplementaryUplink */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* n-TimingAdvanceOffset */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* ssb-PositionsInBurst */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* ssb-PeriodicityServingCell */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* tdd-UL-DL-ConfigurationCommon */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 } /* ss-PBCH-BlockPower */
};
asn_SEQUENCE_specifics_t asn_SPC_ServingCellConfigCommonSIB_specs_1 = {
	sizeof(struct ServingCellConfigCommonSIB),
	offsetof(struct ServingCellConfigCommonSIB, _asn_ctx),
	asn_MAP_ServingCellConfigCommonSIB_tag2el_1,
	8,	/* Count of tags in the map */
	asn_MAP_ServingCellConfigCommonSIB_oms_1,	/* Optional members */
	4, 0,	/* Root/Additions */
	8,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ServingCellConfigCommonSIB = {
	"ServingCellConfigCommonSIB",
	"ServingCellConfigCommonSIB",
	&asn_OP_SEQUENCE,
	asn_DEF_ServingCellConfigCommonSIB_tags_1,
	sizeof(asn_DEF_ServingCellConfigCommonSIB_tags_1)
		/sizeof(asn_DEF_ServingCellConfigCommonSIB_tags_1[0]), /* 1 */
	asn_DEF_ServingCellConfigCommonSIB_tags_1,	/* Same as above */
	sizeof(asn_DEF_ServingCellConfigCommonSIB_tags_1)
		/sizeof(asn_DEF_ServingCellConfigCommonSIB_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_ServingCellConfigCommonSIB_1,
	8,	/* Elements count */
	&asn_SPC_ServingCellConfigCommonSIB_specs_1	/* Additional specs */
};

