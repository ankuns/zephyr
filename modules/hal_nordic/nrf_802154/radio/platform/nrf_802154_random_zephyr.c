/*
 * Copyright (c) 2019 - 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <platform/nrf_802154_temperature.h>
#include <zephyr/drivers/entropy.h>

static uint32_t state;

static uint32_t next(void)
{
	state ^= (state<<13);
	state ^= (state>>17);
	return state ^= (state<<5);
}

void nrf_802154_random_init(void)
{
#if DT_HAS_CHOSEN(zephyr_entropy)
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
	int err;

	__ASSERT_NO_MSG(device_is_ready(dev));

	do {
		err = entropy_get_entropy(dev, (uint8_t *)&state, sizeof(state));
		__ASSERT_NO_MSG(err == 0);
	} while (state == 0);
#else
	/* As a fallback, to get compiled let's pretend we have entropy source which gave us this value.
	 * It is technically incorrect because all nodes will have the same pseud-random sequence, but
	 * at least allows to compile.
	 */
	state = 1;
#endif
}

void nrf_802154_random_deinit(void)
{
	/* Intentionally empty */
}

uint32_t nrf_802154_random_get(void)
{
	return next();
}
