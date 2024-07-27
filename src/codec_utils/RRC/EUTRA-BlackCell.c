/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "EUTRA-BlackCell.h"

asn_TYPE_member_t asn_MBR_EUTRA_BlackCell_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct EUTRA_BlackCell, cellIndexEUTRA),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EUTRA_CellIndex,
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
		"cellIndexEUTRA"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EUTRA_BlackCell, physCellIdRange),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EUTRA_PhysCellIdRange,
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
		"physCellIdRange"
		},
};
static const ber_tlv_tag_t asn_DEF_EUTRA_BlackCell_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_EUTRA_BlackCell_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* cellIndexEUTRA */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* physCellIdRange */
};
asn_SEQUENCE_specifics_t asn_SPC_EUTRA_BlackCell_specs_1 = {
	sizeof(struct EUTRA_BlackCell),
	offsetof(struct EUTRA_BlackCell, _asn_ctx),
	asn_MAP_EUTRA_BlackCell_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_EUTRA_BlackCell = {
	"EUTRA-BlackCell",
	"EUTRA-BlackCell",
	&asn_OP_SEQUENCE,
	asn_DEF_EUTRA_BlackCell_tags_1,
	sizeof(asn_DEF_EUTRA_BlackCell_tags_1)
		/sizeof(asn_DEF_EUTRA_BlackCell_tags_1[0]), /* 1 */
	asn_DEF_EUTRA_BlackCell_tags_1,	/* Same as above */
	sizeof(asn_DEF_EUTRA_BlackCell_tags_1)
		/sizeof(asn_DEF_EUTRA_BlackCell_tags_1[0]), /* 1 */
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
	asn_MBR_EUTRA_BlackCell_1,
	2,	/* Elements count */
	&asn_SPC_EUTRA_BlackCell_specs_1	/* Additional specs */
};

