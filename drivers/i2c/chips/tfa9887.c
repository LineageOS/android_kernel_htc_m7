#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/tfa9887.h>
#include <sound/initval.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

//#define I2C_DEBUGGING

#define TAG "[TFA9887]"
#define TFA_LOGI(fmt, ...) printk(KERN_INFO "%s: %s: " fmt, TAG, __func__, ##__VA_ARGS__)
#define TFA_LOGW(fmt, ...) printk(KERN_WARNING "%s: %s: " fmt, TAG, __func__, ##__VA_ARGS__)
#define TFA_LOGE(fmt, ...) printk(KERN_ERR "%s: %s: " fmt, TAG, __func__, ##__VA_ARGS__)

#define STATUS_OK 0

/* Binary configuration data */
#include "tfa9887_data.c"

/* Objects */
static struct tfa9887_t *tfa9887R, *tfa9887L;

/* Helper functions */

static void convert_bytes2data(const unsigned char bytes[], int num_bytes, int data[])
{
	int i; /* index for data */
	int k; /* index for bytes */
	int d;
	int num_data = num_bytes/3;

	for (i = 0, k = 0; i < num_data; ++i, k += 3) {
		d = (bytes[k] << 16) | (bytes[k+1] << 8) | (bytes[k+2]);
		if (bytes[k] & 0x80) {/* sign bit was set*/
			d = - ((1<<24)-d);
		}
		data[i] = d;
	}
}

static int tfa9887_write_reg(struct tfa9887_t *tfa9887,
		unsigned int subaddress, unsigned int value)
{
	int error = regmap_write(tfa9887->regmap,subaddress,value);
#ifdef I2C_DEBUGGING
	TFA_LOGI("Writing 0x%02x: 0x%04x\n", (0xFF & subaddress),
			(0xFFFF & value));
#endif
	return error;
}

static int tfa9887_read_reg(struct tfa9887_t *tfa9887,
		unsigned int subaddress, unsigned int* pValue)
{
	int error = regmap_read(tfa9887->regmap,subaddress,pValue);
#ifdef I2C_DEBUGGING
	TFA_LOGI("Read 0x%02x: 0x%04x\n", (0xFF & subaddress),
			(0xFFFF & (*pValue)));
#endif
	return error;
}

static int dsp_read_mem(struct tfa9887_t *tfa9887, unsigned short start_offset,
		int num_words, int *pValues)
{
	unsigned int cf_ctrl; /* the value to sent to the CF_CONTROLS register */
	unsigned char bytes[MAX_I2C_LENGTH];
	int burst_size; /* number of words per burst size */
	int bytes_per_word = 3;
	int num_bytes;
	int* p;
	int error;
	/* first set DMEM and AIF, leaving other bits intact */
	error = tfa9887_read_reg(tfa9887, TFA9887_CF_CONTROLS, &cf_ctrl);
	if (error != Tfa9887_Error_Ok) {
		return error;
	}
	cf_ctrl &= ~0x000E; /* clear AIF & DMEM */
	cf_ctrl |= (Tfa9887_DMEM_XMEM << 1); /* set DMEM, leave AIF cleared for autoincrement */
	error = tfa9887_write_reg(tfa9887, TFA9887_CF_CONTROLS, cf_ctrl);
	if (error != Tfa9887_Error_Ok) {
		return error;
	}
	error = tfa9887_write_reg(tfa9887, TFA9887_CF_MAD, start_offset);
	if (error != Tfa9887_Error_Ok) {
		return error;
	}
	num_bytes = num_words*bytes_per_word;
	p = pValues;
	for (; num_bytes > 0; ) {
		burst_size = ROUND_DOWN(MAX_I2C_LENGTH, bytes_per_word);
		if (num_bytes < burst_size) {
			burst_size = num_bytes;
		}

		error = regmap_raw_read(tfa9887->regmap_byte, TFA9887_CF_MEM, bytes,burst_size);
		if (error != Tfa9887_Error_Ok) {
			return error;
		}
		convert_bytes2data(bytes, burst_size,  p);
		num_bytes -= burst_size;
		p += burst_size/bytes_per_word;
	}
	return Tfa9887_Error_Ok;
}

static int load_binary_data(struct tfa9887_t *tfa9887,
		const unsigned char* bytes, int length)
{
	unsigned int size;
	int index = 0;
	unsigned char buffer[MAX_I2C_LENGTH];
	int error;
	int value = 0;
	unsigned int status;
	error = tfa9887_read_reg(tfa9887, TFA9887_STATUS, &status);
	if (error == Tfa9887_Error_Ok) {
		if ( (status & 0x0043) != 0x0043) {
			/* one of Vddd, PLL and clocks not ok */
			error = -1;
		}
	}
	TFA_LOGI("tfa status %u\n", status);
	error = dsp_read_mem(tfa9887, 0x2210, 1, &value);
	TFA_LOGI("tfa version %x\n", value);
	while (index < length) {
		/* extract little endian length */
		size = bytes[index] + bytes[index+1] * 256;
		index += 2;
		if ( (index + size) > length) {
			/* outside the buffer, error in the input data */
			return -1;
		}
		memcpy(buffer, bytes + index, size);
		error = regmap_raw_write(tfa9887->regmap_byte, buffer[0],
				&buffer[1], (size -1));
		TFA_LOGI("%d %d\n", buffer[0], size - 1);
		if (error != Tfa9887_Error_Ok) {
			TFA_LOGE("error\n");
			break;
		}
		index += size;
	}
	return error;
}

static int dsp_set_param(struct tfa9887_t *tfa9887, unsigned char module_id,
		unsigned char param_id, const unsigned char *data, int num_bytes)
{
	int error;
	unsigned int cf_ctrl = 0x0002; /* the value to be sent to the CF_CONTROLS register: cf_req=00000000, cf_int=0, cf_aif=0, cf_dmem=XMEM=01, cf_rst_dsp=0 */
	unsigned int cf_mad = 0x0001; /* memory address to be accessed (0 : Status, 1 : ID, 2 : parameters) */
	unsigned int cf_status; /* the contents of the CF_STATUS register */
	unsigned char mem[3];
	int rpcStatus = STATUS_OK;
	int tries = 0;

	error = tfa9887_write_reg(tfa9887, TFA9887_CF_CONTROLS, cf_ctrl);
	if (error == Tfa9887_Error_Ok) {
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_MAD, cf_mad);
	}
	if (error == Tfa9887_Error_Ok) {
		unsigned char id[3];
		id[0] = 0;
		id[1] = module_id+128;
		id[2] = param_id;
		error = regmap_raw_write(tfa9887->regmap_byte, TFA9887_CF_MEM,&id, 3);
	}

	error = regmap_raw_write(tfa9887->regmap_byte, TFA9887_CF_MEM, data, num_bytes);

	if (error == Tfa9887_Error_Ok) {
		cf_ctrl |= (1<<8) | (1<<4); /* set the cf_req1 and cf_int bit */
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_CONTROLS, cf_ctrl);

		do {
			error = tfa9887_read_reg(tfa9887, TFA9887_CF_STATUS, &cf_status);
			tries++;
			usleep_range(100, 200);
		} while ((error == Tfa9887_Error_Ok) && ((cf_status & 0x0100) == 0) && (tries < 100)); /* don't wait forever, DSP is pretty quick to respond (< 1ms) */

		if (tries >= 100 && tfa9887->powered) {
			/* something wrong with communication with DSP */
			TFA_LOGE("Timed out waiting for status, powered: %d\n",
					tfa9887->powered);
			error = -1;
		}
	}
	cf_ctrl = 0x0002;
	cf_mad = 0x0000;
	if (error == Tfa9887_Error_Ok) {
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_CONTROLS,cf_ctrl);
	}

	if (error == Tfa9887_Error_Ok) {
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_MAD, cf_mad);
	}
	if (error == Tfa9887_Error_Ok) {
		error = regmap_raw_read(tfa9887->regmap_byte, TFA9887_CF_MEM,&mem,3);
		rpcStatus = (int)((mem[0] << 16) | (mem[1] << 8) | mem[2]);
	}
	if (error == Tfa9887_Error_Ok) {
		if (rpcStatus != STATUS_OK) {
			error = rpcStatus+100;
			TFA_LOGE("RPC rpcStatus =%d\n", error);
		}
	}
	return error;
}

#if 0
/* Unused */
static int dsp_get_param(struct tfa9887_t *tfa9887, unsigned char module_id,
		unsigned char param_id, unsigned char *data, int num_bytes)
{
	int error;
	/* the value to be sent to the CF_CONTROLS register: cf_req=00000000,
	 * cf_int=0, cf_aif=0, cf_dmem=XMEM=01, cf_rst_dsp=0 */
	unsigned int cf_ctrl = 0x0002;
	/* memory address to be access (0:Status, 1:ID, 2:parameters) */
	unsigned int cf_mad = 0x0001;
	/* the contents of the CF_STATUS register */
	unsigned int cf_status;
	unsigned char mem[3];
	unsigned char id[3];
	int tries = 0;
	/* 1) write the id and data to the DSP XMEM */
	error = tfa9887_write_reg(tfa9887, TFA9887_CF_CONTROLS,
				cf_ctrl);
	error = tfa9887_write_reg(tfa9887, TFA9887_CF_MAD, cf_mad);
	id[0] = 0;
	id[1] = module_id + 128;
	id[2] = param_id;
	/* only try MEM once, if error, need to resend mad as well */
	error = regmap_raw_write(tfa9887->regmap_byte, TFA9887_CF_MEM, id, 3);
	/* 2) wake up the DSP and let it process the data */
	if (error == 0) {
		cf_ctrl |= (1 << 8) | (1 << 4); /* set the cf_req1 and cf_int bit */
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_CONTROLS,
					cf_ctrl);
	}
	/* 3) wait for the ack */
	if (error == Tfa9887_Error_Ok) {
		do {
			error = tfa9887_read_reg(tfa9887, TFA9887_CF_STATUS,
						&cf_status);
			msleep(1);
			tries++;
		} while (error == 0 && ((cf_status & 0x0100) == 0) && tries < 100);
		/* don't wait forever, DSP is pretty quick to respond (< 1ms) */

		if (tries >= 100) {
			/* something wrong with communication with DSP */
			TFA_LOGE("GetParam failed\n");
			return -1;
		}
	}
	/* 4) check the RPC return value */
	cf_ctrl = 0x0002;
	cf_mad = 0x0000;
	if (error == Tfa9887_Error_Ok) {
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_CONTROLS,cf_ctrl);
	}
	if (error == Tfa9887_Error_Ok) {
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_MAD, cf_mad);
	}
	if (error == Tfa9887_Error_Ok) {
		regmap_raw_read(tfa9887->regmap_byte, TFA9887_CF_MEM,&mem,3);
		error = (mem[0] << 16) | (mem[1] << 8) | mem[2];
	}
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("RPC error\n");
	}

	/* 5) read the resulting data */
	if (error == 0) {
		/* memory address to access (0:Status, 1:ID, 2:parameters) */
		cf_mad = 0x0002;
		error = tfa9887_write_reg(tfa9887, TFA9887_CF_MAD, cf_mad);
	}
	if (error == 0) {
		error =
			regmap_raw_read(tfa9887->regmap_byte, TFA9887_CF_MEM, data, num_bytes);
	}
	return error;
}
#endif

/* Module functions */

static int tfa9887_power(struct tfa9887_t *tfa9887, int on)
{
	int error;
	unsigned int value;

	mutex_lock(&tfa9887->lock);
	tfa9887->powered = on;

	error = tfa9887_read_reg(tfa9887, TFA9887_SYSTEM_CONTROL, &value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to read from TFA9887_SYSTEM_CONTROL\n");
		goto power_err;
	}

	// get powerdown bit
	value &= ~(TFA9887_SYSCTRL_POWERDOWN);
	if (!on) {
		value |= TFA9887_SYSCTRL_POWERDOWN;
	}

	error = tfa9887_write_reg(tfa9887, TFA9887_SYSTEM_CONTROL, value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to write TFA9887_SYSTEM_CONTROL\n");
	}

	if (!on) {
		msleep(10);
	}

power_err:
	mutex_unlock(&tfa9887->lock);
	return error;
}

static int tfa9887_set_volume(struct tfa9887_t *tfa9887, int volume)
{
	int error;
	unsigned int value;

	error = tfa9887_read_reg(tfa9887, TFA9887_AUDIO_CONTROL, &value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to read from TFA9887_AUDIO_CONTROL\n");
		goto set_vol_err;
	}

	value = ((value & 0x00FF) | (unsigned int)(volume << 8));
	error = tfa9887_write_reg(tfa9887, TFA9887_AUDIO_CONTROL, value);

set_vol_err:
	return error;
}

static int tfa9887_mute(struct tfa9887_t *tfa9887, Tfa9887_Mute_t mute)
{
	int error;
	unsigned int aud_value, sys_value;

	error = tfa9887_read_reg(tfa9887, TFA9887_AUDIO_CONTROL, &aud_value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to read from TFA9887_AUDIO_CONTROL\n");
		goto mute_err;
	}
	error = tfa9887_read_reg(tfa9887, TFA9887_SYSTEM_CONTROL, &sys_value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to read from TFA9887_SYSTEM_CONTROL\n");
		goto mute_err;
	}

	switch (mute) {
		case Tfa9887_Mute_Off:
			/* clear CTRL_MUTE, set ENBL_AMP, mute none */
			aud_value &= ~(TFA9887_AUDIOCTRL_MUTE);
			sys_value |= TFA9887_SYSCTRL_ENBL_AMP;
			break;
		case Tfa9887_Mute_Digital:
			/* set CTRL_MUTE, set ENBL_AMP, mute ctrl */
			aud_value |= TFA9887_AUDIOCTRL_MUTE;
			sys_value |= TFA9887_SYSCTRL_ENBL_AMP;
			break;
		case Tfa9887_Mute_Amplifier:
			/* clear CTRL_MUTE, clear ENBL_AMP, only mute amp */
			aud_value &= ~(TFA9887_AUDIOCTRL_MUTE);
			sys_value &= ~(TFA9887_SYSCTRL_ENBL_AMP);
			break;
		default:
			error = -1;
			TFA_LOGW("Unknown mute type: %d\n", mute);
			goto mute_err;
	}

	error = tfa9887_write_reg(tfa9887, TFA9887_AUDIO_CONTROL, aud_value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to write TFA9887_AUDIO_CONTROL\n");
		goto mute_err;
	}
	error = tfa9887_write_reg(tfa9887, TFA9887_SYSTEM_CONTROL, sys_value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to write TFA9887_SYSTEM_CONTROL\n");
		goto mute_err;
	}

mute_err:
	return error;
}

static int tfa9887_enable_dsp(struct tfa9887_t *tfa9887, bool enable)
{
	TFA_LOGI("enable: %d\n", enable);
	mutex_lock(&tfa9887->lock);
	tfa9887->dsp_enabled = enable;
	mutex_unlock(&tfa9887->lock);
	return Tfa9887_Error_Ok;
};

static int tfa9887_select_input(struct tfa9887_t *tfa9887, int input)
{
	int error;
	unsigned int value;

	error = tfa9887_read_reg(tfa9887, TFA9887_I2S_CONTROL, &value);
	if (error != Tfa9887_Error_Ok) {
		goto select_amp_err;
	}

	// clear 2 bits
	value &= ~(0x3 << TFA9887_I2SCTRL_INPUT_SEL_SHIFT);

	switch (input) {
		case 1:
			value |= 0x40;
			break;
		case 2:
			value |= 0x80;
			break;
		default:
			TFA_LOGW("Invalid input selected: %d\n",
					input);
			error = -1;
			goto select_amp_err;
	}
	error = tfa9887_write_reg(tfa9887, TFA9887_I2S_CONTROL, value);

select_amp_err:
	return error;
}

static int tfa9887_select_channel(struct tfa9887_t *tfa9887, int channels)
{
	int error;
	unsigned int value;

	error = tfa9887_read_reg(tfa9887, TFA9887_I2S_CONTROL, &value);
	if (error != Tfa9887_Error_Ok) {
		goto select_channel_err;
	}

	// clear the 2 bits first
	value &= ~(0x3 << TFA9887_I2SCTRL_CHANSEL_SHIFT);

	switch (channels) {
		case 0:
			value |= 0x8;
			break;
		case 1:
			value |= 0x10;
			break;
		case 2:
			value |= 0x18;
			break;
		default:
			TFA_LOGW("Too many channels requested: %d\n",
					channels);
			error = -1;
			goto select_channel_err;
	}
	error = tfa9887_write_reg(tfa9887, TFA9887_I2S_CONTROL, value);

select_channel_err:
	return error;
}

static int tfa9887_set_sample_rate(struct tfa9887_t *tfa9887, int sample_rate)
{
	int error;
	unsigned int value;

	error = tfa9887_read_reg(tfa9887, TFA9887_I2S_CONTROL, &value);
	if (error == Tfa9887_Error_Ok) {
		// clear the 4 bits first
		value &= (~(0xF << TFA9887_I2SCTRL_RATE_SHIFT));
		switch (sample_rate) {
			case 48000:
				value |= TFA9887_I2SCTRL_RATE_48000;
				break;
			case 44100:
				value |= TFA9887_I2SCTRL_RATE_44100;
				break;
			case 32000:
				value |= TFA9887_I2SCTRL_RATE_32000;
				break;
			case 24000:
				value |= TFA9887_I2SCTRL_RATE_24000;
				break;
			case 22050:
				value |= TFA9887_I2SCTRL_RATE_22050;
				break;
			case 16000:
				value |= TFA9887_I2SCTRL_RATE_16000;
				break;
			case 12000:
				value |= TFA9887_I2SCTRL_RATE_12000;
				break;
			case 11025:
				value |= TFA9887_I2SCTRL_RATE_11025;
				break;
			case 8000:
				value |= TFA9887_I2SCTRL_RATE_08000;
				break;
			default:
				TFA_LOGE("Unsupported sample rate %d\n", sample_rate);
				error = -1;
				return error;
		}
		error = tfa9887_write_reg(tfa9887, TFA9887_I2S_CONTROL,
				value);
	}

	return error;
}

static int tfa9887_wait_ready(struct tfa9887_t *tfa9887,
		unsigned int ready_bits, unsigned int ready_state)
{
	int error;
	unsigned int value;
	int tries;
	bool ready;

	tries = 0;
	do {
		error = tfa9887_read_reg(tfa9887, TFA9887_STATUS, &value);
		ready = (error == Tfa9887_Error_Ok &&
				(value & ready_bits) == ready_state);
		TFA_LOGI("Waiting for 0x%04x, current state: 0x%04x\n",
				ready_state, value);
		tries++;
		msleep(10);
	} while (!ready && tries < 10);

	if (tries >= 10) {
		TFA_LOGE("Timed out waiting for tfa9887 to become ready\n");
		error = -1;
	}

	return error;
}

static int tfa9887_set_configured(struct tfa9887_t *tfa9887)
{
	int error;
	unsigned int value;

	error = tfa9887_read_reg(tfa9887, TFA9887_SYSTEM_CONTROL, &value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to read from TFA9887_SYSTEM_CONTROL\n");
		goto set_conf_err;
	}
	value |= TFA9887_SYSCTRL_CONFIGURED;
	tfa9887_write_reg(tfa9887, TFA9887_SYSTEM_CONTROL, value);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to write TFA9887_SYSTEM_CONTROL\n");
	}

set_conf_err:
	return error;
}

static int tfa9887_startup(struct tfa9887_t *tfa9887)
{
	int error;
	unsigned int value;

	error = tfa9887_write_reg(tfa9887, 0x09, 0x0002);
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_read_reg(tfa9887, 0x09, &value);
	}
	if (Tfa9887_Error_Ok == error) {
		/* DSP must be in control of the amplifier to avoid plops */
		value |= TFA9887_SYSCTRL_SEL_ENBL_AMP;
		error = tfa9887_write_reg(tfa9887, 0x09, value);
	}

	/* some other registers must be set for optimal amplifier behaviour */
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x40, 0x5A6B);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x05, 0x13AB);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x06, 0x001F);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, TFA9887_SPKR_CALIBRATION,
				0x0C4E);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x09, 0x025D);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x0A, 0x3EC3);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x41, 0x0308);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x48, 0x0180);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x49, 0x0E82);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x52, 0x0000);
	}
	if (Tfa9887_Error_Ok == error) {
		error = tfa9887_write_reg(tfa9887, 0x40, 0x0000);
	}

	return error;
}

/* helper functions to reduce code duplication */

static int tfa9887_do_init(struct tfa9887_t *tfa9887, int sample_rate,
		bool is_right)
{
	int error;
	char *patch_data, *config_data, *preset_data, *speaker_data, *eq_data;
	int patch_size, config_size, preset_size, speaker_size, eq_size;
	int channel;
	unsigned int pll_lock_bits = (TFA9887_STATUS_CLKS | TFA9887_STATUS_PLLS);

	if (is_right) {
		channel = 1;
		patch_data = patch_data_right;
		patch_size = PATCH_RIGHT_SIZE;
		config_data = config_playback_right;
		config_size = CONFIG_PLAYBACK_RIGHT_SIZE;
		preset_data = preset_playback_right;
		preset_size = PRESET_PLAYBACK_RIGHT_SIZE;
		speaker_data = speaker_data_right;
		speaker_size = SPEAKER_RIGHT_SIZE;
		eq_data = eq_playback_right;
		eq_size = EQ_PLAYBACK_RIGHT_SIZE;
	} else {
		channel = 0;
		patch_data = patch_data_left;
		patch_size = PATCH_LEFT_SIZE;
		config_data = config_playback_left;
		config_size = CONFIG_PLAYBACK_LEFT_SIZE;
		preset_data = preset_playback_left;
		preset_size = PRESET_PLAYBACK_LEFT_SIZE;
		speaker_data = speaker_data_left;
		speaker_size = SPEAKER_LEFT_SIZE;
		eq_data = eq_playback_left;
		eq_size = EQ_PLAYBACK_LEFT_SIZE;
	}

	/* do cold boot init */
	error = tfa9887_startup(tfa9887);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to initialize\n");
		goto priv_init_err;
	}
	error = tfa9887_set_sample_rate(tfa9887, sample_rate);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to set sample rate\n");
		goto priv_init_err;
	}
	error = tfa9887_select_channel(tfa9887, channel);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to select channel\n");
		goto priv_init_err;
	}
	error = tfa9887_select_input(tfa9887, 2);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to select input\n");
		goto priv_init_err;
	}
	error = tfa9887_set_volume(tfa9887, 0);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to set volume\n");
		goto priv_init_err;
	}
	error = tfa9887_power(tfa9887, true);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to power up\n");
		goto priv_init_err;
	}

	/* wait for ready */
	error = tfa9887_wait_ready(tfa9887, pll_lock_bits, pll_lock_bits);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Failed to lock PLLs\n");
		goto priv_init_err;
	}

	/* load firmware */
	error = load_binary_data(tfa9887, patch_data, patch_size);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to load patch data\n");
		goto priv_init_err;
	}
	error = dsp_set_param(tfa9887, MODULE_SPEAKERBOOST, PARAM_SET_CONFIG,
			config_data, config_size);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to load config data\n");
		goto priv_init_err;
	}
	error = dsp_set_param(tfa9887, MODULE_SPEAKERBOOST, PARAM_SET_PRESET,
			preset_data, preset_size);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to load preset data\n");
		goto priv_init_err;
	}
	error = dsp_set_param(tfa9887, MODULE_SPEAKERBOOST, PARAM_SET_LSMODEL,
			speaker_data, speaker_size);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to load speaker data\n");
		goto priv_init_err;
	}
	error = dsp_set_param(tfa9887, MODULE_BIQUADFILTERBANK, PARAM_SET_EQ,
			eq_data, eq_size);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to load EQ data\n");
		goto priv_init_err;
	}
	error = tfa9887_set_configured(tfa9887);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Unable to set configured\n");
		goto priv_init_err;
	}

	/* wait for ready */
	error = tfa9887_wait_ready(tfa9887, TFA9887_STATUS_MTPB, 0);
	if (error != Tfa9887_Error_Ok) {
		TFA_LOGE("Failed to become ready\n");
		goto priv_init_err;
	}

	/* enable DSP */
	tfa9887_enable_dsp(tfa9887, true);

priv_init_err:
	return error;
}

static void tfa9887_enable_spk_amp(struct tfa9887_t *tfa9887, bool enable,
		bool is_right)
{
	int error;
	unsigned int value;
	unsigned int dsp_bypass;

	if (is_right) {
		dsp_bypass = 0x8853;
	} else {
		dsp_bypass = 0x880B;
	}

	mutex_lock(&tfa9887->lock);
	if (enable && !tfa9887->spkamp_enabled) {
		/* off->on transition */
		tfa9887->spkamp_enabled = true;
		if (tfa9887->dsp_enabled) {
			error = tfa9887_power(tfa9887, true);
			error = tfa9887_mute(tfa9887, Tfa9887_Mute_Off);
		} else {
			error = tfa9887_write_reg(tfa9887, 0x04, dsp_bypass);
			error = tfa9887_write_reg(tfa9887, 0x09, 0x0619);
			error = tfa9887_write_reg(tfa9887, 0x09, 0x0618);

			/* set up speakers */
			error = tfa9887_read_reg(tfa9887, 0x08, &value);
			value |= (0x4 << 8);
			error = tfa9887_write_reg(tfa9887, 0x08, value);
		}
	} else if (!enable && tfa9887->spkamp_enabled) {
		/* on->off transition */
		tfa9887->spkamp_enabled = false;
		if (tfa9887->dsp_enabled) {
			error = tfa9887_power(tfa9887, true);
			error = tfa9887_mute(tfa9887, Tfa9887_Mute_Digital);
		} else {
			/* power down amp */
			tfa9887_write_reg(tfa9887, 0x09, 0x0619);
		}
	}
	mutex_unlock(&tfa9887->lock);
}

/* Public functions */
static bool tfa9887_initialized = false;
int Tfa9887_Init(int sample_rate)
{
	int error;
	bool left_init = false;
	bool right_init = false;

	if (tfa9887R) {
		error = tfa9887_do_init(tfa9887R, sample_rate, true);
		if (error != Tfa9887_Error_Ok) {
			TFA_LOGW("Error %d occurred initializing tfa9887R!",
					error);
			right_init = false;
		} else {
			right_init = true;
		}
		TFA_LOGI("Completed TFA9887R initialization");
	}
	if (tfa9887L) {
		error = tfa9887_do_init(tfa9887L, sample_rate, false);
		if (error != Tfa9887_Error_Ok) {
			TFA_LOGW("Error %d occurred initializing tfa9887L!",
					error);
			left_init = false;
		} else {
			left_init = true;
		}
		TFA_LOGI("Completed TFA9887L initialization");
	}

	tfa9887_initialized = right_init && left_init;

	return Tfa9887_Error_Ok;
}
EXPORT_SYMBOL(Tfa9887_Init);

void Tfa9887_EnableSpkAmp(bool enable)
{
	TFA_LOGI("enable: %d", enable);
	if (tfa9887R) {
		tfa9887_enable_spk_amp(tfa9887R, enable, true);
	}
	if (tfa9887L) {
		tfa9887_enable_spk_amp(tfa9887L, enable, false);
	}
}
EXPORT_SYMBOL(Tfa9887_EnableSpkAmp);

/* Kernel module initialization functions */

static ssize_t tfa9887_dsp_state_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	bool left_dsp = false;
	bool right_dsp = false;

	if (tfa9887R) {
		mutex_lock(&tfa9887R->lock);
		right_dsp = tfa9887R->dsp_enabled;
		mutex_unlock(&tfa9887R->lock);
	}
	if (tfa9887L) {
		mutex_lock(&tfa9887L->lock);
		left_dsp = tfa9887L->dsp_enabled;
		mutex_unlock(&tfa9887L->lock);
	}

	if (left_dsp && right_dsp) {
		*buf = '1';
	} else {
		*buf = '0';
	}

	return 1;
}

static ssize_t tfa9887_dsp_state_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = count;

	if (!buf || count < 1) {
		ret = -EINVAL;
		goto dsp_state_fail;
	}

	if (buf[0] == '0') {
		if (tfa9887R) {
			tfa9887_enable_dsp(tfa9887R, false);
		}
		if (tfa9887L) {
			tfa9887_enable_dsp(tfa9887L, false);
		}
	} else if (buf[0] == '1') {
		if (tfa9887R) {
			tfa9887_enable_dsp(tfa9887R, true);
		}
		if (tfa9887L) {
			tfa9887_enable_dsp(tfa9887R, true);
		}
	} else {
		ret = -EINVAL;
		goto dsp_state_fail;
	}

dsp_state_fail:
	return ret;
}

static struct kobject *tfa9887_kobj;
static struct kobj_attribute tfa9887_dsp_state =
		__ATTR(dsp_state, 0660, tfa9887_dsp_state_show,
				tfa9887_dsp_state_store);

static bool tfa9887_readable_register(struct device *dev, unsigned int reg)
{
	return true;
}

static bool tfa9887_volatile_register(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config tfa9887_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.volatile_reg = tfa9887_volatile_register,
	.readable_reg = tfa9887_readable_register,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config tfa9887_regmap_byte = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = tfa9887_volatile_register,
	.readable_reg = tfa9887_readable_register,
	.cache_type = REGCACHE_NONE,
};

static __devinit int tfa9887R_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	unsigned int val;
	int ret;

	TFA_LOGI("E\n");
	tfa9887R = devm_kzalloc(&i2c->dev, sizeof(struct tfa9887_t),
			GFP_KERNEL);

	if (tfa9887R == NULL)
		return -ENOMEM;
	tfa9887R->regmap = regmap_init_i2c(i2c, &tfa9887_regmap);
	tfa9887R->regmap_byte = regmap_init_i2c(i2c, &tfa9887_regmap_byte);
	if (IS_ERR(tfa9887R->regmap)) {
		ret = PTR_ERR(tfa9887R->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
				ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tfa9887R);
	mutex_init(&tfa9887R->lock);
	tfa9887R->irq = i2c->irq;
	ret = regmap_read(tfa9887R->regmap, TFA9887_REVISIONNUMBER, &val);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to read chip revision: %d\n", ret);
		goto err;
	}
	dev_info(&i2c->dev, "TFA9887 revision %d\n",val);
	tfa9887_kobj = kobject_create_and_add("tfa9887", kernel_kobj);
	ret = sysfs_create_file(tfa9887_kobj, (const struct attribute *) &tfa9887_dsp_state);

	if (tfa9887R) {
		mutex_lock(&tfa9887R->lock);
		tfa9887R->initialized = true;
		tfa9887R->dsp_enabled = false;
		tfa9887R->spkamp_enabled = false;
		tfa9887R->powered = false;
		mutex_unlock(&tfa9887R->lock);
	}
	return 0;
err:
	regmap_exit(tfa9887R->regmap);
	return ret;
}

static __devexit int tfa9887R_i2c_remove(struct i2c_client *client)
{
	struct tfa9887_t *tfa9887R = i2c_get_clientdata(client);
	regmap_exit(tfa9887R->regmap);
	regmap_exit(tfa9887R->regmap_byte);
	sysfs_remove_file(tfa9887_kobj, (const struct attribute *) &tfa9887_dsp_state);
	kobject_del(tfa9887_kobj);
	return 0;
}

static void tfa9887R_i2c_shutdown(struct i2c_client *i2c)
{
	if (tfa9887R) {
		mutex_lock(&tfa9887R->lock);
		if (i2c->irq)
			disable_irq(i2c->irq);
		tfa9887R->initialized = false;
		mutex_unlock(&tfa9887R->lock);
	}
}

static const struct i2c_device_id tfa9887R_i2c_id[] = {
	{ "tfa9887R", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa9887R_i2c_id);

static struct i2c_driver tfa9887R_i2c_driver = {
	.driver = {
		.name = "tfa9887R",
		.owner = THIS_MODULE,
	},
	.probe =    tfa9887R_i2c_probe,
	.remove =   __devexit_p(tfa9887R_i2c_remove),
	.id_table = tfa9887R_i2c_id,
	.shutdown = tfa9887R_i2c_shutdown,
};

static __devinit int tfa9887L_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	unsigned int val;
	int ret;

	TFA_LOGI("E\n");
	tfa9887L = devm_kzalloc(&i2c->dev,  sizeof(struct tfa9887_t),
			GFP_KERNEL);

	if (tfa9887L == NULL)
		return -ENOMEM;
	tfa9887L->regmap = regmap_init_i2c(i2c, &tfa9887_regmap);
	tfa9887L->regmap_byte = regmap_init_i2c(i2c, &tfa9887_regmap_byte);
	if (IS_ERR(tfa9887L->regmap)) {
		ret = PTR_ERR(tfa9887L->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
				ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tfa9887L);
	mutex_init(&tfa9887L->lock);
	tfa9887L->irq = i2c->irq;
	ret = regmap_read(tfa9887L->regmap, TFA9887_REVISIONNUMBER, &val);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to read chip revision: %d\n", ret);
		goto err;
	}
	dev_info(&i2c->dev, "TFA9887 revision %d\n",val);
	if (tfa9887L) {
		mutex_lock(&tfa9887L->lock);
		tfa9887L->initialized = true;
		tfa9887L->dsp_enabled = false;
		tfa9887L->spkamp_enabled = false;
		tfa9887L->powered = false;
		mutex_unlock(&tfa9887L->lock);
	}
	return 0;
err:
	regmap_exit(tfa9887L->regmap);
	return ret;
}

static __devexit int tfa9887L_i2c_remove(struct i2c_client *client)
{
	struct tfa9887_t *tfa9887L = i2c_get_clientdata(client);
	regmap_exit(tfa9887L->regmap);
	regmap_exit(tfa9887L->regmap_byte);
	return 0;
}

static void tfa9887L_i2c_shutdown(struct i2c_client *i2c)
{
	if (tfa9887L) {
		mutex_lock(&tfa9887L->lock);
		if (i2c->irq)
			disable_irq(i2c->irq);
		tfa9887L->initialized = false;
		mutex_unlock(&tfa9887L->lock);
	}
}

static const struct i2c_device_id tfa9887L_i2c_id[] = {
	{ "tfa9887L", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa9887L_i2c_id);

static struct i2c_driver tfa9887L_i2c_driver = {
	.driver = {
		.name = "tfa9887L",
		.owner = THIS_MODULE,
	},
	.probe = tfa9887L_i2c_probe,
	.remove = __devexit_p(tfa9887L_i2c_remove),
	.id_table = tfa9887L_i2c_id,
	.shutdown = tfa9887L_i2c_shutdown,
};

static int __init tfa9887_modinit(void)
{
	int ret = 0;
	ret = i2c_add_driver(&tfa9887R_i2c_driver);
	if (ret != 0) {
		TFA_LOGE("Failed to register tfa9887 I2C driver: %d\n", ret);
	}
	ret = i2c_add_driver(&tfa9887L_i2c_driver);
	if (ret != 0) {
		TFA_LOGE("Failed to register tfa9887 I2C driver: %d\n", ret);
	}
	return ret;
}
module_init(tfa9887_modinit);

static void __exit tfa9887_exit(void)
{
	i2c_del_driver(&tfa9887R_i2c_driver);
	i2c_del_driver(&tfa9887L_i2c_driver);
}
module_exit(tfa9887_exit);
