// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASoC codec driver for Awinic AW88266A (PID 0x2013) Smart PA
 *
 * Supports multi-PA arrays (e.g. quad speakers on Galaxy Tab A8 / gta8wifi)
 * sharing a common active-low reset GPIO and configured via Awinic ACF
 * firmware (aw882xx_pid_2013_acf.bin).
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define AW88266_CHIP_ID			0x2013
#define AW88266_FW_NAME			"aw882xx_pid_2013_acf.bin"
#define AW88266_MAX_DEVICES		4
#define AW88266_MAX_PROFILES		11

/* Registers (16-bit big-endian) */
#define AW88266_REG_ID			0x00
#define AW88266_REG_SYSST		0x01
#define AW88266_REG_SYSINT		0x02
#define AW88266_REG_SYSINTM		0x03
#define AW88266_REG_SYSCTRL		0x04
#define AW88266_REG_SYSCTRL2		0x05
#define AW88266_REG_I2SCTRL		0x06
#define AW88266_REG_VSNTM1		0x50
#define AW88266_REG_PLLCTRL1		0x55
#define AW88266_REG_BSTCTRL2		0x61
#define AW88266_REG_EFRL		0x78
#define AW88266_REG_EFRH		0x79
#define AW88266_REG_EFR_BST		0x7a
#define AW88266_REG_MAX			0x7c

/* Register bit definitions */
#define AW88266_SOFT_RESET_VAL		0x55aa

#define AW88266_SYSST_PLLS		BIT(0)
#define AW88266_SYSST_CLKS		BIT(4)
#define AW88266_SYSST_SWS		BIT(8)
#define AW88266_SYSST_BSTS		BIT(9)
#define AW88266_SYSST_PLL_OK		(AW88266_SYSST_PLLS | AW88266_SYSST_CLKS)
#define AW88266_SYSST_START_OK		(AW88266_SYSST_PLL_OK | AW88266_SYSST_SWS | AW88266_SYSST_BSTS)

#define AW88266_SYSCTRL_PWDN		BIT(0)
#define AW88266_SYSCTRL_AMPPD		BIT(1)
#define AW88266_SYSCTRL_BSTPD		BIT(7)
#define AW88266_SYSCTRL_HMUTE		BIT(8)
#define AW88266_SYSCTRL_KEEP_MASK	(AW88266_SYSCTRL_PWDN | AW88266_SYSCTRL_AMPPD | AW88266_SYSCTRL_HMUTE)

#define AW88266_VOL_MASK		0x03ff
#define AW88266_VOL_MUTE		0x02d0

#define AW88266_I2S_CHSEL_MASK		0x0c00
#define AW88266_I2S_CHSEL_LEFT		0x0400
#define AW88266_I2S_CHSEL_RIGHT		0x0800
#define AW88266_I2S_TXEN		BIT(14)

#define AW88266_PLLCTRL1_RST		BIT(2)

#define AW88266_BSTCTRL2_MODE_MASK	0x0030
#define AW88266_BSTCTRL2_MODE_TRANSP	0x0000
#define AW88266_BSTCTRL2_MODE_FORCE	0x0010

/* ACF container format */
#define AW_ACF_FILE_ID			0x0a15f908
#define AW_ACF_HDR_VER			1
#define AW_ACF_HDR_SIZE			0x50
#define AW_ACF_DDT_SIZE			0x40
#define AW_ACF_SEC_TYPE_MULTI_DEV	2
#define AW_ACF_DATA_TYPE_REG		0
#define AW_ACF_DATA_TYPE_BIN		4
#define AW_ACF_DATA_TYPE_SKIP		10

#define AW_BIN_HDR_SIZE			0x40
#define AW_BIN_HDR_VER			0x01000000

static const char * const aw88266_profile_names[AW88266_MAX_PROFILES] = {
	"Music",
	"Voice",
	"Voip",
	"Ringtone",
	"Ringtone Headset",
	"Low Power",
	"Bypass",
	"MMI",
	"FM",
	"Notification",
	"Receiver",
};

static const char * const aw88266_rx_texts[] = { "Off", "On" };
static SOC_ENUM_SINGLE_EXT_DECL(aw88266_rx_enum, aw88266_rx_texts);

static const char * const aw88266_spin_texts[] = { "0", "90", "180", "270" };
static SOC_ENUM_SINGLE_EXT_DECL(aw88266_spin_enum, aw88266_spin_texts);

struct aw88266_profile {
	bool valid;
	const __le16 *regs;
	u32 nregs;
};

struct aw88266 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	const struct firmware *fw;

	u32 dev_index;
	u32 channel;
	u32 spin_ch[4];
	u32 spin;

	struct aw88266_profile profiles[AW88266_MAX_PROFILES];
	u32 prof_map[AW88266_MAX_PROFILES];
	const char *prof_texts[AW88266_MAX_PROFILES];
	struct soc_enum prof_enum;
	struct snd_kcontrol_new prof_kctl;
	u32 prof_count;
	u32 cur_prof;
	u32 applied_prof;

	u32 saved_vol;
	u32 bst_mode;
	u32 bst_en;

	bool rx_enable;
	bool stream_active;
	bool started;

	struct mutex lock;
};

static const struct regmap_config aw88266_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = AW88266_REG_MAX,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static bool aw88266_bstcfg_enabled(struct aw88266 *aw)
{
	unsigned int val = 0;

	if (regmap_read(aw->regmap, AW88266_REG_EFR_BST, &val))
		return false;

	return (((val >> 13) & 0x7) != 0x7) && (((val >> 10) & 0x7) != 0x7);
}

static int aw88266_reg_update(struct aw88266 *aw, unsigned int reg,
			      unsigned int mask, unsigned int val)
{
	unsigned int cur = 0;
	int ret;

	ret = regmap_read(aw->regmap, reg, &cur);
	if (ret)
		return ret;

	cur = (cur & ~mask) | (val & mask);
	return regmap_write(aw->regmap, reg, cur);
}

static void aw88266_apply_efuse_vcalb(struct aw88266 *aw)
{
	unsigned int reg78 = 0, reg79 = 0;
	u32 icalk, vcalk, vcalb;

	if (regmap_read(aw->regmap, AW88266_REG_EFRL, &reg78))
		return;
	if (regmap_read(aw->regmap, AW88266_REG_EFRH, &reg79))
		return;

	icalk = reg78 & 0x3ff;
	if (icalk & BIT(9))
		icalk = 0x3ff;

	vcalk = reg79 & 0x3ff;
	if (vcalk & BIT(9))
		vcalk = 0x3ff;

	vcalb = ((icalk + 1000) << 13) / (vcalk + 1000);
	regmap_write(aw->regmap, AW88266_REG_VSNTM1, vcalb);
	dev_dbg(aw->dev, "icalk %u vcalk %u vcalb %#x\n",
		icalk + 1000, vcalk + 1000, vcalb);
}

static int aw88266_apply_profile(struct aw88266 *aw)
{
	const struct aw88266_profile *prof;
	unsigned int cur_vol = 0;
	u32 i;
	int ret;

	if (aw->cur_prof >= AW88266_MAX_PROFILES)
		return -EINVAL;

	prof = &aw->profiles[aw->cur_prof];
	if (!prof->valid)
		return -EINVAL;

	mutex_lock(&aw->lock);

	for (i = 0; i < prof->nregs; i++) {
		u16 reg = le16_to_cpu(prof->regs[i * 2]);
		u16 val = le16_to_cpu(prof->regs[i * 2 + 1]);

		if (reg == AW88266_REG_VSNTM1)
			continue;

		if (reg == AW88266_REG_SYSCTRL) {
			unsigned int cur = 0;

			ret = regmap_read(aw->regmap, AW88266_REG_SYSCTRL, &cur);
			if (ret)
				goto out;

			val = (cur & AW88266_SYSCTRL_KEEP_MASK) |
			      (val & ~AW88266_SYSCTRL_KEEP_MASK);
			if (aw88266_bstcfg_enabled(aw))
				aw->bst_en = val & AW88266_SYSCTRL_BSTPD;
		} else if (reg == AW88266_REG_BSTCTRL2) {
			aw->bst_mode = val & AW88266_BSTCTRL2_MODE_MASK;
		}

		ret = regmap_write(aw->regmap, reg, val);
		if (ret)
			goto out;
	}

	aw88266_reg_update(aw, AW88266_REG_I2SCTRL, AW88266_I2S_CHSEL_MASK,
			   aw->spin_ch[aw->spin]);

	aw88266_apply_efuse_vcalb(aw);

	if (!regmap_read(aw->regmap, AW88266_REG_SYSCTRL2, &cur_vol))
		aw->saved_vol = cur_vol & AW88266_VOL_MASK;

	regmap_update_bits(aw->regmap, AW88266_REG_SYSCTRL2,
			   AW88266_VOL_MASK, AW88266_VOL_MUTE);

	aw->applied_prof = aw->cur_prof;
	ret = 0;
out:
	mutex_unlock(&aw->lock);
	return ret;
}

static void aw88266_clear_int(struct aw88266 *aw)
{
	unsigned int dummy = 0;

	regmap_read(aw->regmap, AW88266_REG_SYSINT, &dummy);
	regmap_read(aw->regmap, AW88266_REG_SYSINT, &dummy);
}

static void aw88266_stop(struct aw88266 *aw)
{
	mutex_lock(&aw->lock);
	if (!aw->started) {
		mutex_unlock(&aw->lock);
		return;
	}

	aw88266_clear_int(aw);
	regmap_write(aw->regmap, AW88266_REG_SYSINTM, 0xffff);
	regmap_update_bits(aw->regmap, AW88266_REG_SYSCTRL2,
			   AW88266_VOL_MASK, AW88266_VOL_MUTE);
	aw88266_reg_update(aw, AW88266_REG_SYSCTRL, AW88266_SYSCTRL_HMUTE,
			   AW88266_SYSCTRL_HMUTE);
	usleep_range(1000, 1100);
	aw88266_reg_update(aw, AW88266_REG_SYSCTRL, AW88266_SYSCTRL_AMPPD,
			   AW88266_SYSCTRL_AMPPD);
	aw88266_reg_update(aw, AW88266_REG_SYSCTRL, AW88266_SYSCTRL_PWDN,
			   AW88266_SYSCTRL_PWDN);
	aw->started = false;

	mutex_unlock(&aw->lock);
}

static int aw88266_start(struct aw88266 *aw)
{
	unsigned int st = 0;
	int attempt, i, ret, last_err = 0;

	if (aw->applied_prof != aw->cur_prof) {
		ret = aw88266_apply_profile(aw);
		if (ret)
			return ret;
	}

	mutex_lock(&aw->lock);
	if (aw->started) {
		mutex_unlock(&aw->lock);
		return 0;
	}

	for (attempt = 0; attempt < 5; attempt++) {
		bool pll_ok = false;
		bool sysst_ok = false;

		if (aw88266_bstcfg_enabled(aw)) {
			if (!aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
						AW88266_SYSCTRL_BSTPD, 0))
				aw88266_reg_update(aw, AW88266_REG_BSTCTRL2,
						   AW88266_BSTCTRL2_MODE_MASK,
						   AW88266_BSTCTRL2_MODE_FORCE);
		}

		aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
				   AW88266_SYSCTRL_PWDN, 0);
		usleep_range(2000, 2010);

		for (i = 0; i < 200; i++) {
			ret = regmap_read(aw->regmap, AW88266_REG_SYSST, &st);
			if (ret) {
				last_err = -EIO;
				break;
			}
			if ((st & AW88266_SYSST_PLL_OK) == AW88266_SYSST_PLL_OK) {
				pll_ok = true;
				break;
			}
			usleep_range(2000, 2010);
		}

		if (!pll_ok && last_err != -EIO) {
			aw88266_reg_update(aw, AW88266_REG_PLLCTRL1,
					   AW88266_PLLCTRL1_RST,
					   AW88266_PLLCTRL1_RST);
			for (i = 0; i < 200; i++) {
				ret = regmap_read(aw->regmap, AW88266_REG_SYSST, &st);
				if (ret) {
					last_err = -EIO;
					break;
				}
				if ((st & AW88266_SYSST_PLL_OK) == AW88266_SYSST_PLL_OK) {
					pll_ok = true;
					break;
				}
				usleep_range(2000, 2010);
			}
			aw88266_reg_update(aw, AW88266_REG_PLLCTRL1,
					   AW88266_PLLCTRL1_RST, 0);
			if (!pll_ok && last_err != -EIO)
				last_err = -ETIMEDOUT;
		}

		if (!pll_ok) {
			dev_dbg(aw->dev, "PLL check failed, retry %d\n", attempt);
			aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
					   AW88266_SYSCTRL_PWDN,
					   AW88266_SYSCTRL_PWDN);
			continue;
		}

		aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
				   AW88266_SYSCTRL_AMPPD, 0);
		usleep_range(1000, 1050);

		for (i = 0; i < 200; i++) {
			ret = regmap_read(aw->regmap, AW88266_REG_SYSST, &st);
			if (ret) {
				last_err = -EIO;
				break;
			}
			if ((st & AW88266_SYSST_START_OK) == AW88266_SYSST_START_OK) {
				sysst_ok = true;
				break;
			}
			usleep_range(2000, 2010);
		}

		if (!sysst_ok) {
			if (last_err != -EIO) {
				last_err = -ETIMEDOUT;
				dev_err(aw->dev, "SYSST check failed, last %#06x\n", st);
			}
			aw88266_reg_update(aw, AW88266_REG_I2SCTRL,
					   AW88266_I2S_TXEN, 0);
			aw88266_clear_int(aw);
			aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
					   AW88266_SYSCTRL_AMPPD,
					   AW88266_SYSCTRL_AMPPD);
			aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
					   AW88266_SYSCTRL_PWDN,
					   AW88266_SYSCTRL_PWDN);
			continue;
		}

		if (aw88266_bstcfg_enabled(aw)) {
			aw88266_reg_update(aw, AW88266_REG_BSTCTRL2,
					   AW88266_BSTCTRL2_MODE_MASK,
					   AW88266_BSTCTRL2_MODE_TRANSP);
			usleep_range(5000, 5050);
			aw88266_reg_update(aw, AW88266_REG_BSTCTRL2,
					   AW88266_BSTCTRL2_MODE_MASK,
					   aw->bst_mode);
			aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
					   AW88266_SYSCTRL_BSTPD,
					   aw->bst_en);
		}

		aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
				   AW88266_SYSCTRL_HMUTE, 0);
		regmap_update_bits(aw->regmap, AW88266_REG_SYSCTRL2,
				   AW88266_VOL_MASK, aw->saved_vol);
		aw88266_clear_int(aw);
		regmap_write(aw->regmap, AW88266_REG_SYSINTM, 0x0000);
		aw->started = true;
		dev_dbg(aw->dev, "PA started\n");
		mutex_unlock(&aw->lock);
		return 0;
	}

	dev_err(aw->dev, "PA start failed (%d), continuing\n", last_err);
	mutex_unlock(&aw->lock);
	return 0;
}

static int aw88266_get_rx(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = aw->rx_enable ? 1 : 0;
	return 0;
}

static int aw88266_put_rx(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);
	bool enable = ucontrol->value.enumerated.item[0] != 0;

	if (aw->rx_enable == enable)
		return 0;

	aw->rx_enable = enable;
	if (!enable && aw->started)
		aw88266_stop(aw);
	else if (enable && aw->stream_active && !aw->started)
		aw88266_start(aw);

	return 1;
}

static int aw88266_get_spin(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = aw->spin;
	return 0;
}

static int aw88266_put_spin(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);
	unsigned int spin = ucontrol->value.enumerated.item[0];

	if (spin > 3)
		return -EINVAL;
	if (aw->spin == spin)
		return 0;

	aw->spin = spin;
	if (aw->started) {
		int ret;

		mutex_lock(&aw->lock);
		ret = aw88266_reg_update(aw, AW88266_REG_I2SCTRL,
					 AW88266_I2S_CHSEL_MASK,
					 aw->spin_ch[spin]);
		mutex_unlock(&aw->lock);
		if (ret)
			return ret;
	}

	return 1;
}

static int aw88266_get_profile(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);
	u32 i;

	for (i = 0; i < aw->prof_count; i++) {
		if (aw->prof_map[i] == aw->cur_prof) {
			ucontrol->value.enumerated.item[0] = i;
			return 0;
		}
	}

	ucontrol->value.enumerated.item[0] = 0;
	return 0;
}

static int aw88266_put_profile(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);
	unsigned int idx = ucontrol->value.enumerated.item[0];
	u32 target;

	if (idx >= aw->prof_count)
		return -EINVAL;

	target = aw->prof_map[idx];
	if (target == aw->cur_prof)
		return 0;

	aw->cur_prof = target;
	if (aw->started) {
		aw88266_stop(aw);
		if (aw->rx_enable && aw->stream_active)
			return aw88266_start(aw);
	}

	return 1;
}

static int aw88266_stream_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		aw->stream_active = true;
		if (aw->rx_enable && !aw->started)
			return aw88266_start(aw);
		return 0;
	case SND_SOC_DAPM_POST_PMD:
		aw->stream_active = false;
		aw88266_stop(aw);
		return 0;
	default:
		return 0;
	}
}

static const struct snd_kcontrol_new aw88266_controls[] = {
	SOC_SINGLE("Speaker Volume", AW88266_REG_SYSCTRL2, 0, AW88266_VOL_MASK, 1),
	SOC_ENUM_EXT("Speaker RX Switch", aw88266_rx_enum,
		     aw88266_get_rx, aw88266_put_rx),
	SOC_ENUM_EXT("Speaker Rotation", aw88266_spin_enum,
		     aw88266_get_spin, aw88266_put_spin),
};

static const struct snd_soc_dapm_widget aw88266_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN_E("AIF_RX", "Speaker_Playback", 0,
			      SND_SOC_NOPM, 0, 0, aw88266_stream_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route aw88266_dapm_routes[] = {
	{ "SPK", NULL, "AIF_RX" },
};

static int aw88266_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S) {
		dev_err(dai->dev, "unsupported DAI format %#x\n", fmt);
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC) {
		dev_err(dai->dev, "AW88266A must be an I2S clock consumer\n");
		return -EINVAL;
	}

	return 0;
}

static int aw88266_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct aw88266 *aw = snd_soc_component_get_drvdata(dai->component);

	dev_dbg(aw->dev, "hw_params rate %u width %d\n",
		params_rate(params), snd_pcm_format_width(params_format(params)));
	return 0;
}

static const struct snd_soc_dai_ops aw88266_dai_ops = {
	.set_fmt = aw88266_set_fmt,
	.hw_params = aw88266_hw_params,
};

static struct snd_soc_dai_driver aw88266_dai = {
	.name = "aw88266-aif",
	.playback = {
		.stream_name = "Speaker_Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &aw88266_dai_ops,
};

static int aw88266_parse_acf(struct aw88266 *aw)
{
	const u8 *data = aw->fw->data;
	size_t size = aw->fw->size;
	u32 file_id, hdr_ver, ddt_num, ddt_off, i;
	const u8 *ddt;

	if (size < AW_ACF_HDR_SIZE)
		return -EINVAL;

	file_id = get_unaligned_le32(data + 0x00);
	hdr_ver = get_unaligned_le32(data + 0x40);
	if (file_id != AW_ACF_FILE_ID || hdr_ver != AW_ACF_HDR_VER) {
		dev_err(aw->dev, "not an ACF file (id %#x ver %#x)\n",
			file_id, hdr_ver);
		return -EINVAL;
	}

	ddt_num = get_unaligned_le32(data + 0x38);
	ddt_off = get_unaligned_le32(data + 0x3c);
	if ((u64)ddt_off + (u64)ddt_num * AW_ACF_DDT_SIZE > size)
		return -EINVAL;

	dev_info(aw->dev, "ACF %.24s/%.28s: %u sections, %zu bytes\n",
		 (const char *)(data + 0x04), (const char *)(data + 0x1c),
		 ddt_num, size);

	ddt = data + ddt_off;
	for (i = 0; i < ddt_num; i++, ddt += AW_ACF_DDT_SIZE) {
		u32 sec_type = get_unaligned_le32(ddt + 0x00);
		u16 dev_idx = get_unaligned_le16(ddt + 0x14);
		u16 prof_id = get_unaligned_le16(ddt + 0x1a);
		u32 data_type = get_unaligned_le32(ddt + 0x1c);
		u32 sec_size = get_unaligned_le32(ddt + 0x20);
		u32 sec_off = get_unaligned_le32(ddt + 0x24);
		const u8 *sec;

		if (sec_type != AW_ACF_SEC_TYPE_MULTI_DEV ||
		    dev_idx != aw->dev_index)
			continue;

		if ((u64)sec_off + sec_size > size)
			continue;

		if (data_type == AW_ACF_DATA_TYPE_SKIP)
			continue;

		if (data_type != AW_ACF_DATA_TYPE_REG &&
		    data_type != AW_ACF_DATA_TYPE_BIN) {
			dev_warn(aw->dev, "ignoring ACF section type %u\n", data_type);
			continue;
		}

		if (prof_id >= AW88266_MAX_PROFILES)
			continue;

		sec = data + sec_off;
		if (data_type == AW_ACF_DATA_TYPE_REG) {
			aw->profiles[prof_id].regs = (const __le16 *)sec;
			aw->profiles[prof_id].nregs = sec_size / 4;
			aw->profiles[prof_id].valid = true;
		} else {
			u32 bin_hdr_ver, bin_type, bin_ver, bin_len;
			u32 reg_bytes, val_bytes, count, entry_sz;

			if (sec_size < AW_BIN_HDR_SIZE)
				goto bad_bin;

			bin_hdr_ver = get_unaligned_le32(sec + 0x04);
			bin_type = get_unaligned_le32(sec + 0x08);
			bin_ver = get_unaligned_le32(sec + 0x0c);
			bin_len = get_unaligned_le32(sec + 0x10);
			reg_bytes = get_unaligned_le32(sec + 0x20);
			val_bytes = get_unaligned_le32(sec + 0x24);
			count = get_unaligned_le32(sec + 0x3c);

			if (bin_hdr_ver != AW_BIN_HDR_VER || bin_type != 0 ||
			    bin_ver != 1 || reg_bytes != 2 || val_bytes != 2)
				goto bad_bin;

			entry_sz = reg_bytes + val_bytes;
			if (bin_len < 4 || (bin_len - 4) / entry_sz != count ||
			    sec_size < AW_BIN_HDR_SIZE + count * entry_sz)
				goto bad_bin;

			aw->profiles[prof_id].regs = (const __le16 *)(sec + AW_BIN_HDR_SIZE);
			aw->profiles[prof_id].nregs = count;
			aw->profiles[prof_id].valid = true;
			continue;
bad_bin:
			dev_err(aw->dev, "bad register section for profile %u\n", prof_id);
			aw->profiles[prof_id].valid = false;
			aw->profiles[prof_id].nregs = 0;
		}
	}

	aw->prof_count = 0;
	for (i = 0; i < AW88266_MAX_PROFILES; i++) {
		if (!aw->profiles[i].valid)
			continue;
		aw->prof_map[aw->prof_count] = i;
		aw->prof_texts[aw->prof_count] = aw88266_profile_names[i];
		aw->prof_count++;
	}

	if (!aw->prof_count) {
		dev_err(aw->dev, "ACF has no register section for device %u\n",
			aw->dev_index);
		return -EINVAL;
	}

	return 0;
}

static int aw88266_component_probe(struct snd_soc_component *comp)
{
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);
	int ret;

	ret = request_firmware(&aw->fw, AW88266_FW_NAME, aw->dev);
	if (ret) {
		dev_err(aw->dev, "cannot load %s: %d\n", AW88266_FW_NAME, ret);
		return ret;
	}

	ret = aw88266_parse_acf(aw);
	if (ret) {
		release_firmware(aw->fw);
		aw->fw = NULL;
		return ret;
	}

	aw->cur_prof = aw->prof_map[0];
	aw->applied_prof = AW88266_MAX_PROFILES;

	regmap_write(aw->regmap, AW88266_REG_ID, AW88266_SOFT_RESET_VAL);
	usleep_range(1000, 1100);

	ret = aw88266_apply_profile(aw);
	if (ret) {
		dev_err(aw->dev, "cannot apply profile: %d\n", ret);
		return ret;
	}

	aw88266_clear_int(aw);
	regmap_write(aw->regmap, AW88266_REG_SYSINTM, 0xffff);
	aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
			   AW88266_SYSCTRL_AMPPD, AW88266_SYSCTRL_AMPPD);
	aw88266_reg_update(aw, AW88266_REG_SYSCTRL,
			   AW88266_SYSCTRL_PWDN, AW88266_SYSCTRL_PWDN);

	aw->prof_enum.items = aw->prof_count;
	aw->prof_enum.texts = aw->prof_texts;

	aw->prof_kctl.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw->prof_kctl.name = "Speaker Profile";
	aw->prof_kctl.info = snd_soc_info_enum_double;
	aw->prof_kctl.get = aw88266_get_profile;
	aw->prof_kctl.put = aw88266_put_profile;
	aw->prof_kctl.private_value = (unsigned long)&aw->prof_enum;

	ret = snd_soc_add_component_controls(comp, &aw->prof_kctl, 1);
	if (ret)
		return ret;

	dev_info(aw->dev, "AW88266A ready: index %u, %s channel, %u ACF profile(s)\n",
		 aw->dev_index, aw->channel ? "right" : "left", aw->prof_count);
	return 0;
}

static void aw88266_component_remove(struct snd_soc_component *comp)
{
	struct aw88266 *aw = snd_soc_component_get_drvdata(comp);

	aw88266_stop(aw);
	release_firmware(aw->fw);
	aw->fw = NULL;
}

static const struct snd_soc_component_driver aw88266_component_driver = {
	.probe = aw88266_component_probe,
	.remove = aw88266_component_remove,
	.controls = aw88266_controls,
	.num_controls = ARRAY_SIZE(aw88266_controls),
	.dapm_widgets = aw88266_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw88266_dapm_widgets),
	.dapm_routes = aw88266_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(aw88266_dapm_routes),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
};

static int aw88266_parse_spin(struct aw88266 *aw, const char *spin_str)
{
	char c[4] = { 0 };
	int i;

	if (sscanf(spin_str, "%c %c %c %c", &c[0], &c[1], &c[2], &c[3]) != 4)
		return -EINVAL;

	for (i = 0; i < 4; i++) {
		char ch = c[i] & ~0x20;

		if (ch == 'L')
			aw->spin_ch[i] = AW88266_I2S_CHSEL_LEFT;
		else if (ch == 'R')
			aw->spin_ch[i] = AW88266_I2S_CHSEL_RIGHT;
		else
			return -EINVAL;
	}

	return 0;
}

static int aw88266_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;
	struct aw88266 *aw;
	const char *spin_str = NULL;
	u32 def_ch, val = 0;
	unsigned int chip_id = 0;
	int i, ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw->dev = dev;
	aw->rx_enable = true;
	mutex_init(&aw->lock);
	i2c_set_clientdata(i2c, aw);

	aw->regmap = devm_regmap_init_i2c(i2c, &aw88266_regmap_config);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(dev, PTR_ERR(aw->regmap),
				     "regmap init failed\n");

	if (!of_property_read_u32(np, "awinic,dev-index", &val)) {
		if (val >= AW88266_MAX_DEVICES)
			return dev_err_probe(dev, -EINVAL,
					     "awinic,dev-index must be 0..%d\n",
					     AW88266_MAX_DEVICES - 1);
		aw->dev_index = val;
	} else {
		if (i2c->addr < 0x34 || i2c->addr > 0x37)
			return dev_err_probe(dev, -EINVAL,
					     "cannot derive dev-index from address %#x\n",
					     i2c->addr);
		aw->dev_index = i2c->addr - 0x34;
	}

	if (of_property_read_u32(np, "sound-channel", &val) || val > 1) {
		dev_err(dev, "sound-channel must be 0 or 1\n");
		return -EINVAL;
	}
	aw->channel = val;
	def_ch = val ? AW88266_I2S_CHSEL_RIGHT : AW88266_I2S_CHSEL_LEFT;
	for (i = 0; i < 4; i++)
		aw->spin_ch[i] = def_ch;

	if (!of_property_read_string(np, "spin-data", &spin_str)) {
		ret = aw88266_parse_spin(aw, spin_str);
		if (ret) {
			dev_err(dev, "bad spin-data '%s'\n", spin_str);
			return ret;
		}
	}

	aw->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset)) {
		dev_dbg(dev, "reset line not taken (%ld), assuming a sibling reset the bank\n",
			PTR_ERR(aw->reset));
		aw->reset = NULL;
	} else if (aw->reset) {
		gpiod_set_value_cansleep(aw->reset, 1);
		usleep_range(1000, 1250);
		gpiod_set_value_cansleep(aw->reset, 0);
		usleep_range(2000, 2500);
	}

	ret = -ENODEV;
	for (i = 0; i < 5; i++) {
		int r = regmap_read(aw->regmap, AW88266_REG_ID, &chip_id);

		if (r) {
			ret = r;
			break;
		}
		if (chip_id == AW88266_CHIP_ID) {
			ret = 0;
			break;
		}
		dev_dbg(dev, "unexpected chip id %#x\n", chip_id);
		msleep(10);
	}
	if (ret)
		return dev_err_probe(dev, ret, "no AW88266A at %#x\n", i2c->addr);

	return devm_snd_soc_register_component(dev, &aw88266_component_driver,
					       &aw88266_dai, 1);
}

static const struct i2c_device_id aw88266_i2c_id[] = {
	{ "aw88266a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw88266_i2c_id);

static const struct of_device_id aw88266_of_match[] = {
	{ .compatible = "awinic,aw88266a" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw88266_of_match);

static struct i2c_driver aw88266_i2c_driver = {
	.driver = {
		.name = "snd_soc_aw88266",
		.of_match_table = aw88266_of_match,
	},
	.probe = aw88266_i2c_probe,
	.id_table = aw88266_i2c_id,
};
module_i2c_driver(aw88266_i2c_driver);

MODULE_DESCRIPTION("ASoC Awinic AW88266A Smart PA driver");
MODULE_LICENSE("GPL");
