/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "F1AP-PDU-Contents"
 * 	found in "../F1.asn1"
 * 	`asn1c -D ../F1_output/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#include "SCell-FailedtoSetup-List.h"

#include "ProtocolIE-SingleContainer.h"
static asn_oer_constraints_t asn_OER_type_SCell_FailedtoSetup_List_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..32)) */};
static asn_per_constraints_t asn_PER_type_SCell_FailedtoSetup_List_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (SIZE(1..32)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_SCell_FailedtoSetup_List_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_ProtocolIE_SingleContainer_4590P27,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_SCell_FailedtoSetup_List_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_SCell_FailedtoSetup_List_specs_1 = {
	sizeof(struct SCell_FailedtoSetup_List),
	offsetof(struct SCell_FailedtoSetup_List, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
asn_TYPE_descriptor_t asn_DEF_SCell_FailedtoSetup_List = {
	"SCell-FailedtoSetup-List",
	"SCell-FailedtoSetup-List",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_SCell_FailedtoSetup_List_tags_1,
	sizeof(asn_DEF_SCell_FailedtoSetup_List_tags_1)
		/sizeof(asn_DEF_SCell_FailedtoSetup_List_tags_1[0]), /* 1 */
	asn_DEF_SCell_FailedtoSetup_List_tags_1,	/* Same as above */
	sizeof(asn_DEF_SCell_FailedtoSetup_List_tags_1)
		/sizeof(asn_DEF_SCell_FailedtoSetup_List_tags_1[0]), /* 1 */
	{ &asn_OER_type_SCell_FailedtoSetup_List_constr_1, &asn_PER_type_SCell_FailedtoSetup_List_constr_1, SEQUENCE_OF_constraint },
	asn_MBR_SCell_FailedtoSetup_List_1,
	1,	/* Single element */
	&asn_SPC_SCell_FailedtoSetup_List_specs_1	/* Additional specs */
};

