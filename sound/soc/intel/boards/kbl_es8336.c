// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2018-19 Canonical Corporation.

/*
 * Intel Kabylake I2S Machine Driver with es8336 Codec
 *
 * Modified from:
 *   Intel Kabylake I2S Machine driver supporting MAXIM98357a and
 *   DA7219 codecs
 * Also referred to:
 *   Intel Broadwell I2S Machine driver supporting RT5677 codec
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/acpi.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/input.h>

#include "../../codecs/hdac_hdmi.h"

#define KBL_ES8316_DAI "ES8316 HiFi"
#define DUAL_CHANNEL 2

static struct snd_soc_card *kabylake_audio_card;
static struct snd_soc_jack skylake_hdmi[3];

struct kbl_hdmi_pcm {
	struct list_head head;
	struct snd_soc_dai *codec_dai;
	int device;
};

struct kbl_codec_private {
	struct device *codec_dev;
	struct gpio_desc *gpio_pa;
	struct snd_soc_jack jack;
	struct gpio_desc *gpio_lo_mute;
	struct list_head hdmi_pcm_list;
	bool speaker_en;
};

enum {
	KBL_DPCM_AUDIO_PB = 0,
	KBL_DPCM_AUDIO_CP,
	KBL_DPCM_AUDIO_HDMI1_PB,
	KBL_DPCM_AUDIO_HDMI2_PB,
	KBL_DPCM_AUDIO_HDMI3_PB,
};

static const struct acpi_gpio_params pa_enable_gpio = { 0, 0, true };
static const struct acpi_gpio_mapping acpi_es8336_gpios[] = {
	{ "pa-enable-gpios", &pa_enable_gpio, 1 },
	{ }
};

static int sof_es8316_speaker_power_event(struct snd_soc_dapm_widget *w,
					  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;
	struct kbl_codec_private *priv = snd_soc_card_get_drvdata(card);

	if (SND_SOC_DAPM_EVENT_ON(event))
		priv->speaker_en = false;
	else
		priv->speaker_en = true;

	gpiod_set_value_cansleep(priv->gpio_pa, priv->speaker_en);

	return 0;
}

static const struct snd_soc_dapm_widget sof_es8316_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),

	SND_SOC_DAPM_SUPPLY("Speaker Power", SND_SOC_NOPM, 0, 0,
			    sof_es8316_speaker_power_event,
			    SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
};

static const struct snd_soc_dapm_widget dmic_widgets[] = {
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
};

static const struct snd_soc_dapm_route sof_es8316_audio_map[] = {
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},

	/*
	 * There is no separate speaker output instead the speakers are muxed to
	 * the HP outputs. The mux is controlled by the "Speaker Power" supply.
	 */
	{"Speaker", NULL, "HPOL"},
	{"Speaker", NULL, "HPOR"},
	{"Speaker", NULL, "Speaker Power"},

	/* CODEC BE connections */
	{ "AIF1 Playback", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "codec0_out"},

	{ "codec0_in", NULL, "ssp0 Rx" },
	{ "ssp0 Rx", NULL, "AIF1 Capture" },

	{ "hifi1", NULL, "iDisp1 Tx"},
	{ "iDisp1 Tx", NULL, "iDisp1_out"},
	{ "hifi2", NULL, "iDisp2 Tx"},
	{ "iDisp2 Tx", NULL, "iDisp2_out"},
	{ "hifi3", NULL, "iDisp3 Tx"},
	{ "iDisp3 Tx", NULL, "iDisp3_out"},
};

static const struct snd_soc_dapm_route sof_es8316_intmic_in1_map[] = {
	{"MIC1", NULL, "Internal Mic"},
	{"MIC2", NULL, "Headset Mic"},
};

static const struct snd_soc_dapm_route dmic_map[] = {
	/* digital mics */
	{"DMic", NULL, "SoC DMIC"},
};

static const struct snd_kcontrol_new sof_es8316_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Internal Mic"),
};

static struct snd_soc_jack_pin sof_es8316_jack_pins[] = {
	{
		.pin	= "Headphone",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Headset Mic",
		.mask	= SND_JACK_MICROPHONE,
	},
};

static int kabylake_ssp0_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *chan = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* The ADSP will convert the FE rate to 48k, stereo */
	rate->min = rate->max = 48000;
	chan->min = chan->max = DUAL_CHANNEL;

	/* set SSP0 to 24 bit */
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

static int kabylake_es8336_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;
	struct kbl_codec_private *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_component *component = asoc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	const struct snd_soc_dapm_route *custom_map;
	int num_routes;

	card->dapm.idle_bias_off = true;

	custom_map = sof_es8316_intmic_in1_map;
	num_routes = ARRAY_SIZE(sof_es8316_intmic_in1_map);

	ret = snd_soc_dapm_add_routes(&card->dapm, custom_map, num_routes);
	if (ret)
		return ret;
	ret = devm_acpi_dev_add_driver_gpios(component->dev, acpi_es8336_gpios);
	if (ret)
		dev_warn(component->dev, "Failed to add driver gpios\n");

	priv->gpio_pa = gpiod_get_optional(component->dev, "pa-enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpio_pa)) {
		ret = dev_err_probe(component->dev, PTR_ERR(priv->gpio_pa),
				    "could not get pa-enable GPIO\n");
		return -1;
	}

	ret = snd_soc_card_jack_new(card, "Headset",
				    SND_JACK_HEADSET | SND_JACK_BTN_0,
				    &priv->jack, sof_es8316_jack_pins,
				    ARRAY_SIZE(sof_es8316_jack_pins));
	if (ret) {
		dev_err(card->dev, "jack creation failed %d\n", ret);
		return ret;
	}

	snd_jack_set_key(priv->jack.jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);

	snd_soc_component_set_jack(component, &priv->jack, NULL);

	return 0;
}

static int kabylake_hdmi_init(struct snd_soc_pcm_runtime *rtd, int device)
{
	struct kbl_codec_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *dai = asoc_rtd_to_codec(rtd, 0);
	struct kbl_hdmi_pcm *pcm;

	pcm = devm_kzalloc(rtd->card->dev, sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

	pcm->device = device;
	pcm->codec_dai = dai;

	list_add_tail(&pcm->head, &ctx->hdmi_pcm_list);

	return 0;
}

static int kabylake_hdmi1_init(struct snd_soc_pcm_runtime *rtd)
{
	return kabylake_hdmi_init(rtd, KBL_DPCM_AUDIO_HDMI1_PB);
}

static int kabylake_hdmi2_init(struct snd_soc_pcm_runtime *rtd)
{
	return kabylake_hdmi_init(rtd, KBL_DPCM_AUDIO_HDMI2_PB);
}

static int kabylake_hdmi3_init(struct snd_soc_pcm_runtime *rtd)
{
	return kabylake_hdmi_init(rtd, KBL_DPCM_AUDIO_HDMI3_PB);
}

static int kabylake_es8336_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	const int sysclk = 19200000;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 1, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->dev, "%s, Failed to set ES8336 SYSCLK: %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops kabylake_es8336_ops = {
	.hw_params = kabylake_es8336_hw_params,
};

static const unsigned int rates[] = {
	48000,
};

static const struct snd_pcm_hw_constraint_list constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list  = rates,
	.mask = 0,
};

static const unsigned int channels[] = {
	DUAL_CHANNEL,
};

static const struct snd_pcm_hw_constraint_list constraints_channels = {
	.count = ARRAY_SIZE(channels),
	.list = channels,
	.mask = 0,
};

static int kbl_fe_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	/*
	 * On this platform for PCM device we support,
	 * 48Khz
	 * stereo
	 * 16 bit audio
	 */

	runtime->hw.channels_max = DUAL_CHANNEL;
	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
					   &constraints_channels);

	runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
	snd_pcm_hw_constraint_msbits(runtime, 0, 16, 16);

	snd_pcm_hw_constraint_list(runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE, &constraints_rates);

	return 0;
}

static const struct snd_soc_ops kabylake_es8336_fe_ops = {
	.startup = kbl_fe_startup,
};

SND_SOC_DAILINK_DEF(dummy,
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(system,
	DAILINK_COMP_ARRAY(COMP_CPU("System Pin")));

SND_SOC_DAILINK_DEF(hdmi1,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI1 Pin")));

SND_SOC_DAILINK_DEF(hdmi2,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI2 Pin")));

SND_SOC_DAILINK_DEF(hdmi3,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI3 Pin")));

SND_SOC_DAILINK_DEF(ssp0_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP0 Pin")));
SND_SOC_DAILINK_DEF(ssp0_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("i2c-ESSX8336:00", KBL_ES8316_DAI)));

SND_SOC_DAILINK_DEF(idisp1_pin,
		    DAILINK_COMP_ARRAY(COMP_CPU("iDisp1 Pin")));
SND_SOC_DAILINK_DEF(idisp1_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("ehdaudio0D2", "intel-hdmi-hifi1")));

SND_SOC_DAILINK_DEF(idisp2_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("iDisp2 Pin")));
SND_SOC_DAILINK_DEF(idisp2_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("ehdaudio0D2", "intel-hdmi-hifi2")));

SND_SOC_DAILINK_DEF(idisp3_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("iDisp3 Pin")));
SND_SOC_DAILINK_DEF(idisp3_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("ehdaudio0D2", "intel-hdmi-hifi3")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("0000:00:1f.3")));

/* kabylake digital audio interface glue - connects es8336 codec <--> CPU */
static struct snd_soc_dai_link kabylake_es8336_dais[] = {
	/* Front End DAI links */
	[KBL_DPCM_AUDIO_PB] = {
		.name = "Kbl Audio Port",
		.stream_name = "Audio",
		.dynamic = 1,
		.nonatomic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		.ops = &kabylake_es8336_fe_ops,
		SND_SOC_DAILINK_REG(system, dummy, platform),
	},
	[KBL_DPCM_AUDIO_CP] = {
		.name = "Kbl Audio Capture Port",
		.stream_name = "Audio Record",
		.dynamic = 1,
		.nonatomic = 1,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
		.ops = &kabylake_es8336_fe_ops,
		SND_SOC_DAILINK_REG(system, dummy, platform),
	},
	[KBL_DPCM_AUDIO_HDMI1_PB] = {
		.name = "Kbl HDMI Port1",
		.stream_name = "Hdmi1",
		.dpcm_playback = 1,
		.init = NULL,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hdmi1, dummy, platform),
	},
	[KBL_DPCM_AUDIO_HDMI2_PB] = {
		.name = "Kbl HDMI Port2",
		.stream_name = "Hdmi2",
		.dpcm_playback = 1,
		.init = NULL,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hdmi2, dummy, platform),
	},
	[KBL_DPCM_AUDIO_HDMI3_PB] = {
		.name = "Kbl HDMI Port3",
		.stream_name = "Hdmi3",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hdmi3, dummy, platform),
	},

	/* Back End DAI links */
	{
		/* SSP0 - Codec */
		.name = "SSP0-Codec",
		.id = 0,
		.no_pcm = 1,
		.init = kabylake_es8336_codec_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBC_CFC,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = kabylake_ssp0_fixup,
		.ops = &kabylake_es8336_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp0_pin, ssp0_codec, platform),
	},
	{
		.name = "iDisp1",
		.id = 1,
		.dpcm_playback = 1,
		.init = kabylake_hdmi1_init,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp1_pin, idisp1_codec, platform),
	},
	{
		.name = "iDisp2",
		.id = 2,
		.init = kabylake_hdmi2_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp2_pin, idisp2_codec, platform),
	},
	{
		.name = "iDisp3",
		.id = 3,
		.init = kabylake_hdmi3_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(idisp3_pin, idisp3_codec, platform),
	},
};


#define NAME_SIZE	32
static int kabylake_card_late_probe(struct snd_soc_card *card)
{
	struct kbl_codec_private *ctx = snd_soc_card_get_drvdata(card);
	struct kbl_hdmi_pcm *pcm;
	struct snd_soc_component *component = NULL;
	int err, i = 0;
	char jack_name[NAME_SIZE];

	list_for_each_entry(pcm, &ctx->hdmi_pcm_list, head) {
		component = pcm->codec_dai->component;
		snprintf(jack_name, sizeof(jack_name),
			"HDMI/DP, pcm=%d Jack", pcm->device);
		err = snd_soc_card_jack_new(card, jack_name,
					SND_JACK_AVOUT, &skylake_hdmi[i],
					NULL, 0);

		if (err)
			return err;

		err = hdac_hdmi_jack_init(pcm->codec_dai, pcm->device,
				&skylake_hdmi[i]);
		if (err < 0)
			return err;

		i++;

	}

	if (!component)
		return -EINVAL;

	return hdac_hdmi_jack_port_init(component, &card->dapm);
}

/* kabylake audio machine driver for es8336 */
static struct snd_soc_card kabylake_audio_card_es8336 = {
	.name = "kbles8336",
	.owner = THIS_MODULE,
	.dai_link = kabylake_es8336_dais,
	.num_links = ARRAY_SIZE(kabylake_es8336_dais),
	.controls = sof_es8316_controls,
	.num_controls = ARRAY_SIZE(sof_es8316_controls),
	.dapm_widgets = sof_es8316_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sof_es8316_widgets),
	.dapm_routes = sof_es8316_audio_map,
	.num_dapm_routes = ARRAY_SIZE(sof_es8316_audio_map),
	.fully_routed = true,
	.late_probe = kabylake_card_late_probe,
};

static int kabylake_audio_probe(struct platform_device *pdev)
{
	struct kbl_codec_private *ctx;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	INIT_LIST_HEAD(&ctx->hdmi_pcm_list);

	kabylake_audio_card =
		(struct snd_soc_card *)pdev->id_entry->driver_data;

	kabylake_audio_card->dev = &pdev->dev;
	snd_soc_card_set_drvdata(kabylake_audio_card, ctx);
	return devm_snd_soc_register_card(&pdev->dev, kabylake_audio_card);
}

static const struct platform_device_id kbl_board_ids[] = {
	{
		.name = "kbl_es8336",
		.driver_data =
			(kernel_ulong_t)&kabylake_audio_card_es8336,
	},
	{ }
};
MODULE_DEVICE_TABLE(platform, kbl_board_ids);

static struct platform_driver kabylake_audio = {
	.probe = kabylake_audio_probe,
	.driver = {
		.name = "kbl_es8336",
		.pm = &snd_soc_pm_ops,
	},
	.id_table = kbl_board_ids,
};

module_platform_driver(kabylake_audio)

/* Module information */
MODULE_DESCRIPTION("Audio Machine driver-es8336 in I2S mode");
MODULE_AUTHOR("Hui Wang <hui.wang@canonical.com>");
MODULE_LICENSE("GPL v2");
