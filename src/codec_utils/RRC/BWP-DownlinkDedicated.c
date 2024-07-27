/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "./osc-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "BWP-DownlinkDedicated.h"

asn_TYPE_member_t asn_MBR_BWP_DownlinkDedicated_1[] = {
	{ ATF_POINTER, 4, offsetof(struct BWP_DownlinkDedicated, pdcch_Config),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_PDCCH_Config,
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
		"pdcch-Config"
		},
	{ ATF_POINTER, 3, offsetof(struct BWP_DownlinkDedicated, pdsch_Config),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_PDSCH_Config,
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
		"pdsch-Config"
		},
	{ ATF_POINTER, 2, offsetof(struct BWP_DownlinkDedicated, sps_Config),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_SPS_Config,
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
		"sps-Config"
		},
	{ ATF_POINTER, 1, offsetof(struct BWP_DownlinkDedicated, radioLinkMonitoringConfig),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_RadioLinkMonitoringConfig,
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
		"radioLinkMonitoringConfig"
		},
};
static const int asn_MAP_BWP_DownlinkDedicated_oms_1[] = { 0, 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_BWP_DownlinkDedicated_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_BWP_DownlinkDedicated_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* pdcch-Config */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* pdsch-Config */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* sps-Config */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* radioLinkMonitoringConfig */
};
asn_SEQUENCE_specifics_t asn_SPC_BWP_DownlinkDedicated_specs_1 = {
	sizeof(struct BWP_DownlinkDedicated),
	offsetof(struct BWP_DownlinkDedicated, _asn_ctx),
	asn_MAP_BWP_DownlinkDedicated_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_BWP_DownlinkDedicated_oms_1,	/* Optional members */
	4, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_BWP_DownlinkDedicated = {
	"BWP-DownlinkDedicated",
	"BWP-DownlinkDedicated",
	&asn_OP_SEQUENCE,
	asn_DEF_BWP_DownlinkDedicated_tags_1,
	sizeof(asn_DEF_BWP_DownlinkDedicated_tags_1)
		/sizeof(asn_DEF_BWP_DownlinkDedicated_tags_1[0]), /* 1 */
	asn_DEF_BWP_DownlinkDedicated_tags_1,	/* Same as above */
	sizeof(asn_DEF_BWP_DownlinkDedicated_tags_1)
		/sizeof(asn_DEF_BWP_DownlinkDedicated_tags_1[0]), /* 1 */
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
	asn_MBR_BWP_DownlinkDedicated_1,
	4,	/* Elements count */
	&asn_SPC_BWP_DownlinkDedicated_specs_1	/* Additional specs */
};

