/*
 * Copyright (c) 2022 Chromium OS Authors
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT ti_tca9554a

#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(gpio_tca9554a, CONFIG_GPIO_LOG_LEVEL);

/* TCA9554A register addresses */
#define TCA9554A_REG_INPUT				0x0
#define TCA9554A_REG_OUTPUT				0x1
#define TCA9554A_REG_POLARITY_INVERSION	0x2
#define TCA9554A_REG_CONFIGURATION		0x3

/** Cache of the output configuration and data of the pins. */
struct tca9554a_pins_state {
	uint8_t input;
	uint8_t output;
	uint8_t config;
};

struct tca9554a_irq_state {
	uint8_t rising;
	uint8_t falling;
};

/** Configuration data */
struct tca9554a_drv_cfg {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;

	struct i2c_dt_spec i2c_spec;
	struct gpio_dt_spec int_gpio;
};

/** Runtime driver data */
struct tca9554a_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	sys_slist_t callbacks;
	struct k_sem lock;
	struct k_work work;
	const struct device *dev;
	struct gpio_callback int_gpio_cb;
	struct tca9554a_pins_state pins_state;
	struct tca9554a_irq_state irq_state;
};

/**
 * @brief Gets the state of a specified register from the TCA9554A and stores it
 * in a buffer.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reg Address of the register to be read.
 * @param buf Pointer to the buffer to output the register.
 *
 * @retval 0 if successful.
 * @retval Negative value for error code.
 */
static int read_port_reg(const struct device *dev, uint8_t reg, uint8_t *buf)
{
	const struct tca9554a_drv_cfg *const config = dev->config;
	uint8_t value;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c_spec, reg, &value);
	if (ret != 0) {
		LOG_ERR("%s: error reading register 0x%X (%d)", dev->name,
			reg, ret);
		return ret;
	}

	LOG_DBG("%s: Read: REG[0x%X] = 0x%X,", dev->name, reg, (value & 0xFF));

	return 0;
}

/**
 * @brief writes to a specified register from the TCA9554A.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reg Address of the register to be written.
 * @param value The pin value to be written into the register.
 *
 * @retval 0 if successful.
 * @retval Negative value for error code.
 */
static int write_port_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct tca9554a_drv_cfg *const config = dev->config;
	int ret;

	LOG_DBG("%s: Write: REG[0x%X] = 0x%X", dev->name, reg, (value & 0xFF));

	ret = i2c_reg_write_byte_dt(&config->i2c_spec, reg, value);
	if (ret != 0) {
		LOG_ERR("%s: error writing to register 0x%X "
			"(%d)",
			dev->name, reg, ret);
	}

	return ret;
}

static inline int update_input_reg(const struct device *dev, uint8_t *buf)
{
	int ret;
	struct tca9554a_drv_data *const drv_data = dev->data;

	ret = read_port_reg(dev, TCA9554A_REG_INPUT, buf);
	if (ret == 0) {
		drv_data->pins_state.input = *buf;
	}

	return ret;
}

static inline int update_output_reg(const struct device *dev, uint8_t value)
{
	int ret;
	struct tca9554a_drv_data *const drv_data = dev->data;

	ret = write_port_reg(dev, TCA9554A_REG_OUTPUT, value);
	if (ret == 0) {
		drv_data->pins_state.output = value;
	}

	return ret;
}

static inline int update_invers_regs(const struct device *dev, uint8_t value)
{
	return write_port_reg(dev, TCA9554A_REG_POLARITY_INVERSION, value);
}

static inline int update_config_reg(const struct device *dev, uint8_t value)
{
	int ret;
	struct tca9554a_drv_data *const drv_data = dev->data;

	ret = write_port_reg(dev, TCA9554A_REG_CONFIGURATION, value);
	if (ret == 0) {
		drv_data->pins_state.config = value;
	}

	return ret;
}

/**
 * @brief Handles interrupt triggered by the interrupt pin of TCA9554A.
 *
 * If int_gpios is configured in device tree then this will be triggered each
 * time a gpio configured as an input changes state. The gpio input states are
 * read in this function which clears the interrupt.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
static void tca9554a_handle_interrupt(const struct device *dev)
{
	struct tca9554a_drv_data *drv_data = dev->data;
	struct tca9554a_irq_state *irq_state = &drv_data->irq_state;
	int ret;
	uint8_t previous_state;
	uint8_t current_state;
	uint8_t transitioned_pins;
	uint8_t interrupt_status;

	k_sem_take(&drv_data->lock, K_FOREVER);

	/* Any interrupts enabled? */
	if (!irq_state->rising && !irq_state->falling) {
		k_sem_give(&drv_data->lock);
		return;
	}

	/* Store previous input state then read new value */
	previous_state = drv_data->pins_state.input;
	ret = update_input_reg(dev, &current_state);
	if (ret != 0) {
		k_sem_give(&drv_data->lock);
		return;
	}

	/* Find out which input pins have changed state */
	transitioned_pins = previous_state ^ current_state;

	/* Mask gpio transactions with rising/falling edge interrupt config */
	interrupt_status = (irq_state->rising & transitioned_pins & current_state);
	interrupt_status |= (irq_state->falling & transitioned_pins & previous_state);
	k_sem_give(&drv_data->lock);

	if (interrupt_status) {
		gpio_fire_callbacks(&drv_data->callbacks, dev, interrupt_status);
	}
}

/**
 * @brief Work handler for TCA9554A interrupt
 *
 * @param work Work struct that contains pointer to interrupt handler function
 */
static void tca9554a_work_handler(struct k_work *work)
{
	struct tca9554a_drv_data *drv_data = CONTAINER_OF(work, struct tca9554a_drv_data, work);

	tca9554a_handle_interrupt(drv_data->dev);
}

/**
 * @brief ISR for interrupt pin of TCA9554A
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param gpio_cb Pointer to callback function struct
 * @param pins Bitmask of pins that triggered interrupt
 */
static void tca9554a_int_gpio_handler(const struct device *dev, struct gpio_callback *gpio_cb,
				      uint8_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct tca9554a_drv_data *drv_data =
		CONTAINER_OF(gpio_cb, struct tca9554a_drv_data, int_gpio_cb);

	k_work_submit(&drv_data->work);
}

static int tca9554a_setup_pin(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct tca9554a_drv_data *const drv_data = dev->data;
	uint8_t reg_cfg = drv_data->pins_state.config;
	uint8_t reg_out = drv_data->pins_state.output;
	int ret;

	/* For each pin, 0 == output, 1 == input */
	if ((flags & GPIO_OUTPUT) != 0) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			reg_out |= BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			reg_out &= ~BIT(pin);
		}
		ret = update_output_reg(dev, reg_out);
		if (ret != 0) {
			return ret;
		}
		reg_cfg &= ~BIT(pin);
	} else {
		reg_cfg |= BIT(pin);
	}

	ret = update_config_reg(dev, reg_cfg);

	return ret;
}

static int tca9554a_pin_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	int ret;
	struct tca9554a_drv_data *const drv_data = dev->data;

	/* Does not support disconnected pin */
	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		return -ENOTSUP;
	}

	/* Single Ended lines (Open drain and open source) not supported */
	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	/* The TCA9554A has no internal pull up support */
	if (((flags & GPIO_PULL_UP) != 0) || ((flags & GPIO_PULL_DOWN) != 0)) {
		return -ENOTSUP;
	}

	/* Simultaneous input & output mode not supported */
	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret = tca9554a_setup_pin(dev, pin, flags);
	if (ret != 0) {
		LOG_ERR("%s: error setting pin direction (%d)", dev->name, ret);
	}

	k_sem_give(&drv_data->lock);
	return ret;
}

static int tca9554a_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct tca9554a_drv_data *const drv_data = dev->data;
	uint8_t buf;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret = update_input_reg(dev, &buf);
	if (ret == 0) {
		*value = buf;
	}

	k_sem_give(&drv_data->lock);
	return ret;
}

static int tca9554a_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	struct tca9554a_drv_data *const drv_data = dev->data;
	uint8_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	reg_out = drv_data->pins_state.output;
	reg_out = (reg_out & ~mask) | (mask & value);

	ret = update_output_reg(dev, reg_out);

	k_sem_give(&drv_data->lock);

	return ret;
}

static int tca9554a_port_set_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return tca9554a_port_set_masked_raw(dev, mask, mask);
}

static int tca9554a_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return tca9554a_port_set_masked_raw(dev, mask, 0);
}

static int tca9554a_port_toggle_bits(const struct device *dev, gpio_port_pins_t mask)
{
	struct tca9554a_drv_data *const drv_data = dev->data;
	uint32_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	reg_out = drv_data->pins_state.output;
	reg_out ^= mask;

	ret = update_output_reg(dev, reg_out);

	k_sem_give(&drv_data->lock);

	return ret;
}

static int tca9554a_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					    enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	struct tca9554a_drv_data *drv_data = dev->data;
	struct tca9554a_irq_state *irq = &drv_data->irq_state;

	/* Device does not support level-triggered interrupts. */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	if (mode == GPIO_INT_MODE_DISABLED) {
		irq->falling &= ~BIT(pin);
		irq->rising &= ~BIT(pin);
	} else { /* GPIO_INT_MODE_EDGE */
		if (trig == GPIO_INT_TRIG_BOTH) {
			irq->falling |= BIT(pin);
			irq->rising |= BIT(pin);
		} else if (trig == GPIO_INT_TRIG_LOW) {
			irq->falling |= BIT(pin);
			irq->rising &= ~BIT(pin);
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			irq->falling &= ~BIT(pin);
			irq->rising |= BIT(pin);
		}
	}

	k_sem_give(&drv_data->lock);

	return 0;
}

static int tca9554a_manage_callback(const struct device *dev, struct gpio_callback *callback,
				    bool set)
{
	struct tca9554a_drv_data *drv_data = dev->data;

	return gpio_manage_callback(&drv_data->callbacks, callback, set);
}

static DEVICE_API(gpio, tca9554a_drv_api) = {
	.pin_configure = tca9554a_pin_config,
	.port_get_raw = tca9554a_port_get_raw,
	.port_set_masked_raw = tca9554a_port_set_masked_raw,
	.port_set_bits_raw = tca9554a_port_set_bits_raw,
	.port_clear_bits_raw = tca9554a_port_clear_bits_raw,
	.port_toggle_bits = tca9554a_port_toggle_bits,
	.pin_interrupt_configure = tca9554a_pin_interrupt_configure,
	.manage_callback = tca9554a_manage_callback,
};

/**
 * @brief Initialization function of TCA9554A
 *
 * This sets initial input/output configuration and output states.
 * The interrupt is configured if it is enabled.
 *
 * @param dev Device struct
 *
 * @retval 0 if successful.
 * @retval Negative value for error code.
 */
static int tca9554a_init(const struct device *dev)
{
	const struct tca9554a_drv_cfg *drv_cfg = dev->config;
	struct tca9554a_drv_data *drv_data = dev->data;
	int ret;

	if (!device_is_ready(drv_cfg->i2c_spec.bus)) {
		LOG_ERR("I2C device not found");
		return -ENODEV;
	}

	/* Reset polarity inversion register */
	ret = update_invers_regs(dev, 0x0);
	if (ret != 0) {
		LOG_ERR("%s: failed to reset inversion register: %d", dev->name, ret);
		return ret;
	}

	/* Set initial configuration of the pins. */
	ret = update_config_reg(dev, 0xFF);
	if (ret != 0) {
		return ret;
	}

	ret = update_output_reg(dev, 0x0);
	if (ret != 0) {
		return ret;
	}

	/* Read initial state of the input port register. */
	ret = update_input_reg(dev, &drv_data->pins_state.input);
	if (ret != 0) {
		LOG_ERR("%s: failed to initially read input port: %d", dev->name, ret);
		return ret;
	}

	/* If the INT line is available, configure the callback for it. */
	if (drv_cfg->int_gpio.port) {
		if (!gpio_is_ready_dt(&drv_cfg->int_gpio)) {
			LOG_ERR("Cannot get pointer to gpio interrupt device "
				"%s init failed", dev->name);
			return -EINVAL;
		}

		drv_data->dev = dev;

		k_work_init(&drv_data->work, tca9554a_work_handler);

		ret = gpio_pin_configure_dt(&drv_cfg->int_gpio, GPIO_INPUT);
		if (ret != 0) {
			LOG_ERR("%s init failed: %d", dev->name, ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&drv_cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			LOG_ERR("%s init failed: %d", dev->name, ret);
			return ret;
		}

		gpio_init_callback(&drv_data->int_gpio_cb, tca9554a_int_gpio_handler,
				   BIT(drv_cfg->int_gpio.pin));

		ret = gpio_add_callback(drv_cfg->int_gpio.port, &drv_data->int_gpio_cb);
		if (ret != 0) {
			LOG_ERR("%s init failed: %d", dev->name, ret);
			return ret;
		}
	}

	LOG_DBG("%s init ok", dev->name);
	return ret;
}

#define TCA9554A_INST(idx)                                                                         \
	static const struct tca9554a_drv_cfg tca9554a_cfg##idx = {                                 \
		.common = {                                                                        \
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(idx),                     \
		},                                                                                 \
		.i2c_spec = I2C_DT_SPEC_INST_GET(idx),                                             \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(idx, int_gpios, {0}),                         \
	};                                                                                         \
	static struct tca9554a_drv_data tca9554a_data##idx = {                                     \
		.lock = Z_SEM_INITIALIZER(tca9554a_data##idx.lock, 1, 1),                          \
		.work = Z_WORK_INITIALIZER(tca9554a_work_handler),                                 \
		.dev = DEVICE_DT_INST_GET(idx),                                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, tca9554a_init, NULL, &tca9554a_data##idx, &tca9554a_cfg##idx,   \
			      POST_KERNEL, CONFIG_GPIO_TCA9554A_INIT_PRIORITY, &tca9554a_drv_api);

DT_INST_FOREACH_STATUS_OKAY(TCA9554A_INST)
