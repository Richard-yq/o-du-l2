/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "T-Reassembly.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_T_Reassembly_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 5,  5,  0,  31 }	/* (0..31) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_T_Reassembly_value2enum_1[] = {
	{ 0,	3,	"ms0" },
	{ 1,	3,	"ms5" },
	{ 2,	4,	"ms10" },
	{ 3,	4,	"ms15" },
	{ 4,	4,	"ms20" },
	{ 5,	4,	"ms25" },
	{ 6,	4,	"ms30" },
	{ 7,	4,	"ms35" },
	{ 8,	4,	"ms40" },
	{ 9,	4,	"ms45" },
	{ 10,	4,	"ms50" },
	{ 11,	4,	"ms55" },
	{ 12,	4,	"ms60" },
	{ 13,	4,	"ms65" },
	{ 14,	4,	"ms70" },
	{ 15,	4,	"ms75" },
	{ 16,	4,	"ms80" },
	{ 17,	4,	"ms85" },
	{ 18,	4,	"ms90" },
	{ 19,	4,	"ms95" },
	{ 20,	5,	"ms100" },
	{ 21,	5,	"ms110" },
	{ 22,	5,	"ms120" },
	{ 23,	5,	"ms130" },
	{ 24,	5,	"ms140" },
	{ 25,	5,	"ms150" },
	{ 26,	5,	"ms160" },
	{ 27,	5,	"ms170" },
	{ 28,	5,	"ms180" },
	{ 29,	5,	"ms190" },
	{ 30,	5,	"ms200" },
	{ 31,	6,	"spare1" }
};
static const unsigned int asn_MAP_T_Reassembly_enum2value_1[] = {
	0,	/* ms0(0) */
	2,	/* ms10(2) */
	20,	/* ms100(20) */
	21,	/* ms110(21) */
	22,	/* ms120(22) */
	23,	/* ms130(23) */
	24,	/* ms140(24) */
	3,	/* ms15(3) */
	25,	/* ms150(25) */
	26,	/* ms160(26) */
	27,	/* ms170(27) */
	28,	/* ms180(28) */
	29,	/* ms190(29) */
	4,	/* ms20(4) */
	30,	/* ms200(30) */
	5,	/* ms25(5) */
	6,	/* ms30(6) */
	7,	/* ms35(7) */
	8,	/* ms40(8) */
	9,	/* ms45(9) */
	1,	/* ms5(1) */
	10,	/* ms50(10) */
	11,	/* ms55(11) */
	12,	/* ms60(12) */
	13,	/* ms65(13) */
	14,	/* ms70(14) */
	15,	/* ms75(15) */
	16,	/* ms80(16) */
	17,	/* ms85(17) */
	18,	/* ms90(18) */
	19,	/* ms95(19) */
	31	/* spare1(31) */
};
const asn_INTEGER_specifics_t asn_SPC_T_Reassembly_specs_1 = {
	asn_MAP_T_Reassembly_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_T_Reassembly_enum2value_1,	/* N => "tag"; sorted by N */
	32,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_T_Reassembly_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_T_Reassembly = {
	"T-Reassembly",
	"T-Reassembly",
	&asn_OP_NativeEnumerated,
	asn_DEF_T_Reassembly_tags_1,
	sizeof(asn_DEF_T_Reassembly_tags_1)
		/sizeof(asn_DEF_T_Reassembly_tags_1[0]), /* 1 */
	asn_DEF_T_Reassembly_tags_1,	/* Same as above */
	sizeof(asn_DEF_T_Reassembly_tags_1)
		/sizeof(asn_DEF_T_Reassembly_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_T_Reassembly_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_T_Reassembly_specs_1	/* Additional specs */
};

