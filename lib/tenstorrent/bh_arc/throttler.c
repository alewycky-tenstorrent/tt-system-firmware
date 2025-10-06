/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "throttler.h"
#include "aiclk_ppm.h"
#include "cm2dm_msg.h"
#include <zephyr/drivers/misc/bh_fwtable.h>
#include "telemetry_internal.h"
#include "telemetry.h"
#include "noc2axi.h"

static uint32_t power_limit;

#define kThrottlerAiclkScaleFactor 500.0F
#define DEFAULT_BOARD_POWER_LIMIT  150

LOG_MODULE_REGISTER(throttler);

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

typedef enum {
	kThrottlerThm,
	kThrottlerGDDRThm,
	kThrottlerDopplerSlow,
	kThrottlerDopplerFast,
	kThrottlerCount,
} ThrottlerId;

typedef struct {
	float min;
	float max;
} ThrottlerLimitRange;

/* This table is used to restrict the throttler limits to reasonable ranges. */
/* They are passed in from the FW table in SPI */
/* clang-format off */
static const ThrottlerLimitRange throttler_limit_ranges[kThrottlerCount] = {
	[kThrottlerThm]		= { .min = 50, .max = 100, },
	[kThrottlerGDDRThm]	= { .min = 50, .max = 100, },
	[kThrottlerDopplerSlow]	= { .min = 50, .max = 1200, },
	[kThrottlerDopplerFast]	= { .min = 50, .max = 1200, },
};
/* clang-format on */

typedef struct {
	float alpha_filter;
	float p_gain;
	float d_gain;
} ThrottlerParams;

typedef struct {
	const AiclkArbMax arb_max; /* The arbiter associated with this throttler */

	const ThrottlerParams params;
	float limit;
	float value;
	float error;
	float prev_error;
	float output;
} Throttler;

static Throttler throttler[kThrottlerCount] = {
	[kThrottlerThm] =
		{
			.arb_max = kAiclkArbMaxThm,
			.params =
				{
					.alpha_filter = 1.0,
					.p_gain = 0.2,
					.d_gain = 0,
				},
		},
	[kThrottlerGDDRThm] =
		{
			.arb_max = kAiclkArbMaxGDDRThm,
			.params =
				{
					.alpha_filter = 1.0,
					.p_gain = 0.2,
					.d_gain = 0,
				},
		},
	[kThrottlerDopplerSlow] =
		{
			.arb_max = kAiclkArbMaxDopplerSlow,
			.params =
				{
					.alpha_filter = 1.0,
					.p_gain = 0.0025,
					.d_gain = 0.3,
				},
		},
	[kThrottlerDopplerFast] =
		{
			.arb_max = kAiclkArbMaxDopplerFast,
			.params =
				{
					.alpha_filter = 1.0,
					.p_gain = 0.015,
					.d_gain = 0.1,
				},
		},
};

static void SetThrottlerLimit(ThrottlerId id, float limit)
{
	float clamped_limit =
		CLAMP(limit, throttler_limit_ranges[id].min, throttler_limit_ranges[id].max);

	LOG_INF("Throttler %d limit set to %d", id, (uint32_t)clamped_limit);
	throttler[id].limit = clamped_limit;
}

static uint32_t throttle_counter;
static const uint32_t kKernelThrottleAddress = 0x10;

static void InitKernelThrottling()
{
	const uint8_t kNocRing = 0;
	const uint8_t kNocTlb = 1; /* should reserve and pre-program a TLB for this */

	throttle_counter = 0;

	/* should reserve a TLB for this */
	NOC2AXITensixBroadcastTlbSetup(kNocRing, kNocTlb, kKernelThrottleAddress,
				       kNoc2AxiOrderingStrict);
	NOC2AXIWrite32(kNocRing, kNocTlb, kKernelThrottleAddress, throttle_counter);
}

/* must only be called when throttle state changes */
static void SendKernelThrottlingMessage(bool throttle)
{
	const uint8_t kNocRing = 0;
	const uint8_t kNocTlb = 1; /* should reserve and pre-program a TLB for this */

	/* The LLK uses fast = even, slow = odd, but for debug purposes, they'd like to
	 * know how many times throttling has happened. Just in case CMFW somehow gets
	 * out of sync internally, double-check the parity. */
	throttle_counter++;
	if ((throttle_counter & 1) != throttle) {
		throttle_counter++;
	}

	/* should reserve a TLB for this */
	NOC2AXITensixBroadcastTlbSetup(kNocRing, kNocTlb, kKernelThrottleAddress,
				       kNoc2AxiOrderingStrict);
	NOC2AXIWrite32(kNocRing, kNocTlb, kKernelThrottleAddress, throttle_counter);
}

void InitThrottlers(void)
{
	SetThrottlerLimit(kThrottlerThm,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.thm_limit);
	SetThrottlerLimit(kThrottlerGDDRThm,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.gddr_thm_limit);

	SetThrottlerLimit(kThrottlerDopplerSlow, DEFAULT_BOARD_POWER_LIMIT);
	SetThrottlerLimit(kThrottlerDopplerFast, DEFAULT_BOARD_POWER_LIMIT);

	InitKernelThrottling();

	EnableArbMax(throttler[kThrottlerThm].arb_max, true);
	EnableArbMax(throttler[kThrottlerGDDRThm].arb_max, true);

	EnableArbMax(throttler[kThrottlerDopplerSlow].arb_max, true);
	EnableArbMax(throttler[kThrottlerDopplerFast].arb_max, true);

	SetAiclkArbMax(kAiclkArbMaxDopplerCritical, GetAiclkFmin());
	EnableArbMax(kAiclkArbMaxDopplerCritical, false); /* enabled when limit triggered */
}

static void UpdateThrottler(ThrottlerId id, float value)
{
	Throttler *t = &throttler[id];

	t->value = t->params.alpha_filter * value + (1 - t->params.alpha_filter) * t->value;
	t->error = (t->limit - t->value) / t->limit;
	t->output = t->params.p_gain * t->error + t->params.d_gain * (t->error - t->prev_error);
	t->prev_error = t->error;
}

static void UpdateThrottlerArb(ThrottlerId id)
{
	Throttler *t = &throttler[id];

	float arb_val = GetThrottlerArbMax(t->arb_max);

	arb_val += t->output * kThrottlerAiclkScaleFactor;

	SetAiclkArbMax(t->arb_max, arb_val);
}

static uint16_t board_power_history[1000];
static uint16_t *board_power_history_cursor = board_power_history;
static uint32_t board_power_sum = 0;
static bool critical_throttling = false;
static bool kernel_nops_enabled;

static uint32_t samples_above_tdp;            /* 1 bit per sample, LSB is most recent */
static const uint8_t overdrive_threshold = 8; /* 8 of 10 > TDP triggers overdrive */
static const uint8_t overdrive_lookback = 10;
static bool overdrive = false;

#define ADVANCE_CIRCULAR_POINTER(pointer, array)                                                   \
	do {                                                                                       \
		if (++(pointer) == (array) + ARRAY_SIZE(array))                                    \
			(pointer) = (array);                                                       \
	} while (false)

static uint16_t UpdateMovingAveragePower(uint16_t current_power)
{
	board_power_sum += current_power - *board_power_history_cursor;
	*board_power_history_cursor = current_power;

	ADVANCE_CIRCULAR_POINTER(board_power_history_cursor, board_power_history);

	return board_power_sum / ARRAY_SIZE(board_power_history);
}

static void UpdateDopplerOverdrive(const TelemetryInternalData *telemetry)
{
	uint32_t tdp_limit = tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdp_limit;

	samples_above_tdp <<= 1;
	samples_above_tdp |= (telemetry->vcore_power > tdp_limit);

	uint8_t recent_samples_above_tdp =
		POPCOUNT(samples_above_tdp & BIT_MASK(overdrive_lookback));
	if (recent_samples_above_tdp >= overdrive_threshold) {
		overdrive = true;
	} else if (recent_samples_above_tdp == 0) {
		overdrive = false;
	}
}

static void UpdateDoppler(const TelemetryInternalData *telemetry)
{
	UpdateDopplerOverdrive(telemetry);
	EnableArbMax(throttler[kThrottlerThm].arb_max, !overdrive);

	uint16_t current_power = GetInputPower();
	uint16_t average_power = UpdateMovingAveragePower(current_power);

	UpdateThrottler(kThrottlerDopplerFast, current_power);
	UpdateThrottler(kThrottlerDopplerSlow, average_power);

	/* AICLK=Fmin isn't always enough to get below the board power limit. */
	bool start_nops = GetAiclkTarg() == GetAiclkFmin() && current_power > power_limit;
	bool stop_nops = GetAiclkTarg() == GetAiclkFmax() && current_power < power_limit;

	bool overdrive_temp_limit = (telemetry->asic_temperature > throttler[kThrottlerThm].limit);

	bool new_critical_throttling = overdrive_temp_limit;

	bool new_kernel_nops_enabled =
		((kernel_nops_enabled || start_nops) && !stop_nops) || overdrive_temp_limit;

	if (new_kernel_nops_enabled != kernel_nops_enabled) {
		kernel_nops_enabled = new_kernel_nops_enabled;
		SendKernelThrottlingMessage(kernel_nops_enabled);
	}

	if (new_critical_throttling != critical_throttling) {
		critical_throttling = new_critical_throttling;
		EnableArbMax(kAiclkArbMaxDopplerCritical, critical_throttling);
	}
}

void CalculateThrottlers(void)
{
	TelemetryInternalData telemetry_internal_data;

	ReadTelemetryInternal(1, &telemetry_internal_data);

	UpdateDoppler(&telemetry_internal_data);

	UpdateThrottler(kThrottlerThm, telemetry_internal_data.asic_temperature);
	UpdateThrottler(kThrottlerGDDRThm, GetMaxGDDRTemp());

	for (ThrottlerId i = 0; i < kThrottlerCount; i++) {
		UpdateThrottlerArb(i);
	}
}

int32_t Dm2CmSetBoardPowerLimit(const uint8_t *data, uint8_t size)
{
	if (size != 2) {
		return -1;
	}

	power_limit = sys_get_le16(data);

	LOG_INF("Cable Power Limit: %u", power_limit);
	power_limit = MIN(power_limit,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.board_power_limit);

	SetThrottlerLimit(kThrottlerDopplerSlow, power_limit);
	SetThrottlerLimit(kThrottlerDopplerFast, power_limit);

	UpdateTelemetryBoardPowerLimit(power_limit);

	return 0;
}
