/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./new_out -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#include "SI-SchedulingInfo.h"

#include "SI-RequestConfig.h"
#include "SchedulingInfo.h"
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_schedulingInfoList_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1 && size <= 32)) {
		/* Perform validation of the inner elements */
		return td->encoding_constraints.general_constraints(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_systemInformationAreaID_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size == 24)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_schedulingInfoList_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..32)) */};
static asn_per_constraints_t asn_PER_type_schedulingInfoList_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (SIZE(1..32)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_si_WindowLength_constr_4 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_si_WindowLength_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  8 }	/* (0..8) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_schedulingInfoList_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..32)) */};
static asn_per_constraints_t asn_PER_memb_schedulingInfoList_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (SIZE(1..32)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_systemInformationAreaID_constr_16 CC_NOTUSED = {
	{ 0, 0 },
	24	/* (SIZE(24..24)) */};
static asn_per_constraints_t asn_PER_memb_systemInformationAreaID_constr_16 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 0,  0,  24,  24 }	/* (SIZE(24..24)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_schedulingInfoList_2[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_SchedulingInfo,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_schedulingInfoList_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_schedulingInfoList_specs_2 = {
	sizeof(struct SI_SchedulingInfo__schedulingInfoList),
	offsetof(struct SI_SchedulingInfo__schedulingInfoList, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_schedulingInfoList_2 = {
	"schedulingInfoList",
	"schedulingInfoList",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_schedulingInfoList_tags_2,
	sizeof(asn_DEF_schedulingInfoList_tags_2)
		/sizeof(asn_DEF_schedulingInfoList_tags_2[0]) - 1, /* 1 */
	asn_DEF_schedulingInfoList_tags_2,	/* Same as above */
	sizeof(asn_DEF_schedulingInfoList_tags_2)
		/sizeof(asn_DEF_schedulingInfoList_tags_2[0]), /* 2 */
	{ &asn_OER_type_schedulingInfoList_constr_2, &asn_PER_type_schedulingInfoList_constr_2, SEQUENCE_OF_constraint },
	asn_MBR_schedulingInfoList_2,
	1,	/* Single element */
	&asn_SPC_schedulingInfoList_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_si_WindowLength_value2enum_4[] = {
	{ 0,	2,	"s5" },
	{ 1,	3,	"s10" },
	{ 2,	3,	"s20" },
	{ 3,	3,	"s40" },
	{ 4,	3,	"s80" },
	{ 5,	4,	"s160" },
	{ 6,	4,	"s320" },
	{ 7,	4,	"s640" },
	{ 8,	5,	"s1280" }
};
static const unsigned int asn_MAP_si_WindowLength_enum2value_4[] = {
	1,	/* s10(1) */
	8,	/* s1280(8) */
	5,	/* s160(5) */
	2,	/* s20(2) */
	6,	/* s320(6) */
	3,	/* s40(3) */
	0,	/* s5(0) */
	7,	/* s640(7) */
	4	/* s80(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_si_WindowLength_specs_4 = {
	asn_MAP_si_WindowLength_value2enum_4,	/* "tag" => N; sorted by tag */
	asn_MAP_si_WindowLength_enum2value_4,	/* N => "tag"; sorted by N */
	9,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_si_WindowLength_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_si_WindowLength_4 = {
	"si-WindowLength",
	"si-WindowLength",
	&asn_OP_NativeEnumerated,
	asn_DEF_si_WindowLength_tags_4,
	sizeof(asn_DEF_si_WindowLength_tags_4)
		/sizeof(asn_DEF_si_WindowLength_tags_4[0]) - 1, /* 1 */
	asn_DEF_si_WindowLength_tags_4,	/* Same as above */
	sizeof(asn_DEF_si_WindowLength_tags_4)
		/sizeof(asn_DEF_si_WindowLength_tags_4[0]), /* 2 */
	{ &asn_OER_type_si_WindowLength_constr_4, &asn_PER_type_si_WindowLength_constr_4, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_si_WindowLength_specs_4	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_SI_SchedulingInfo_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SI_SchedulingInfo, schedulingInfoList),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_schedulingInfoList_2,
		0,
		{ &asn_OER_memb_schedulingInfoList_constr_2, &asn_PER_memb_schedulingInfoList_constr_2,  memb_schedulingInfoList_constraint_1 },
		0, 0, /* No default value */
		"schedulingInfoList"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SI_SchedulingInfo, si_WindowLength),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_si_WindowLength_4,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"si-WindowLength"
		},
	{ ATF_POINTER, 3, offsetof(struct SI_SchedulingInfo, si_RequestConfig),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SI_RequestConfig,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"si-RequestConfig"
		},
	{ ATF_POINTER, 2, offsetof(struct SI_SchedulingInfo, si_RequestConfigSUL),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SI_RequestConfig,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"si-RequestConfigSUL"
		},
	{ ATF_POINTER, 1, offsetof(struct SI_SchedulingInfo, systemInformationAreaID),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BIT_STRING,
		0,
		{ &asn_OER_memb_systemInformationAreaID_constr_16, &asn_PER_memb_systemInformationAreaID_constr_16,  memb_systemInformationAreaID_constraint_1 },
		0, 0, /* No default value */
		"systemInformationAreaID"
		},
};
static const int asn_MAP_SI_SchedulingInfo_oms_1[] = { 2, 3, 4 };
static const ber_tlv_tag_t asn_DEF_SI_SchedulingInfo_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SI_SchedulingInfo_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* schedulingInfoList */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* si-WindowLength */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* si-RequestConfig */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* si-RequestConfigSUL */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* systemInformationAreaID */
};
asn_SEQUENCE_specifics_t asn_SPC_SI_SchedulingInfo_specs_1 = {
	sizeof(struct SI_SchedulingInfo),
	offsetof(struct SI_SchedulingInfo, _asn_ctx),
	asn_MAP_SI_SchedulingInfo_tag2el_1,
	5,	/* Count of tags in the map */
	asn_MAP_SI_SchedulingInfo_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	5,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SI_SchedulingInfo = {
	"SI-SchedulingInfo",
	"SI-SchedulingInfo",
	&asn_OP_SEQUENCE,
	asn_DEF_SI_SchedulingInfo_tags_1,
	sizeof(asn_DEF_SI_SchedulingInfo_tags_1)
		/sizeof(asn_DEF_SI_SchedulingInfo_tags_1[0]), /* 1 */
	asn_DEF_SI_SchedulingInfo_tags_1,	/* Same as above */
	sizeof(asn_DEF_SI_SchedulingInfo_tags_1)
		/sizeof(asn_DEF_SI_SchedulingInfo_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_SI_SchedulingInfo_1,
	5,	/* Elements count */
	&asn_SPC_SI_SchedulingInfo_specs_1	/* Additional specs */
};

