/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2023 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ESP32 rotary encoder driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ESP32 */
#include <driver/gpio.h>
#include "hal/gpio_ll.h"

#include <stdint.h>
#include <limits>
#include <errno.h>

#define ENC_GPIO_READ(x)		gpio_ll_get_level(&GPIO, (x))

/**
 * @brief Encoder reading class.
 *
 * @tparam T Data type.
 */
template <typename T> class Encoder
{
public:
	enum PullUp { NONE, PULL_UP, PULL_DOWN };

	/**
	 * @brief Default construct a new Encoder object.
	 *
	 * @note Init call is required.
	 */
	Encoder() {}

	/**
	 * @brief Construct a new Encoder object.
	 *
	 * @param pin_a		GPIO number of encoder A pin.
	 * @param pin_b		GPIO number of encoder B pin.
	 * @param pull_up	Enable or disable the pull up or pull down load.
	 */
	Encoder(int pin_a, int pin_b, enum PullUp pull_up = NONE) {
		init(pin_a, pin_b, pull_up);
	}

	/**
	 * @brief Destroy the Encoder object.
	 */
	~Encoder() {
		gpio_isr_handler_remove((gpio_num_t)enc_a);
		gpio_reset_pin((gpio_num_t)enc_a);
		gpio_reset_pin((gpio_num_t)enc_b);
		if (waiter)
			vSemaphoreDelete(waiter);
	}

	/**
	 * @brief Initialize GPIO and install ISRs.
	 *
	 * @param pin_a		GPIO number of encoder A pin.
	 * @param pin_b		GPIO number of encoder B pin.
	 * @param pull_up	Enable or disable the pull up or pull down load.
	 */
	void init(int pin_a, int pin_b, enum PullUp pull_up = NONE) {
		enc_a = (gpio_num_t)pin_a;
		enc_b = (gpio_num_t)pin_b;

		gpio_install_isr_service(0);
		gpio_config(enc_a, pull_up);
		gpio_config(enc_b, pull_up);
		ESP_ERROR_CHECK(gpio_set_intr_type(enc_a,
			pull_up == PULL_DOWN ? \
				GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE));
		ESP_ERROR_CHECK(gpio_isr_handler_add(enc_a, isr_a, this));
		waiter = xSemaphoreCreateBinary();
		ESP_ERROR_CHECK(waiter == nullptr);
	}

	/**
	 * @brief Get the encoder value.
	 *
	 * @return T		Encoder value.
	 */
	T get_value() {
		return value;
	}

	/**
	 * @brief Set the step value.
	 *
	 * @param new_step	New step value.
	 */
	void set_step(T new_step = 1) {
		step = new_step;
	}

	/**
	 * @brief Invert encoder direction.
	 */
	void invert() {
		step = -step;
	}

	/**
	 * @brief Set the new encoder value.
	 *
	 * @param new_value	New value.
	 */
	void set_value(T new_value) {
		value = new_value;
	}

	/**
	 * @brief Set the limits of the encoder value.
	 *
	 * @param new_min	New minimum value.
	 * @param new_max	New minimum value.
	 */
	void set_limits(T new_min, T new_max) {
		min = new_min;
		max = new_max;
	}

	/**
	 * @brief Wait for trigger.
	 *
	 * @param timeout_ms	Timeout in [ms].
	 *
	 * @return int		0 on success, error code if failed.
	 */
	int wait(uint32_t timeout_ms = 0) {
		if (!timeout_ms)
			timeout_ms = portMAX_DELAY;

		if (xSemaphoreTake(waiter, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
			return 0;

		return -ETIMEDOUT;
	}

private:
	static void gpio_config(int g, enum PullUp pull_up) {
		ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t)g));
		ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)g,
			GPIO_MODE_INPUT));
		switch (pull_up) {
		case NONE:
			break;
		case PULL_UP:
			ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)g,
				GPIO_PULLUP_ONLY));
			ESP_ERROR_CHECK(gpio_pullup_en((gpio_num_t)g));
			break;
		case PULL_DOWN:
			ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)g,
				GPIO_PULLDOWN_ONLY));
			ESP_ERROR_CHECK(gpio_pullup_en((gpio_num_t)g));
			break;
		}
	}

	static inline void wakeup(Encoder *e) {
		xSemaphoreGiveFromISR(e->waiter, NULL);
	}

	static inline void check_limit(Encoder *e) {
		if (e->value > e->max)
			e->value = e->max;
		if (e->value < e->min)
			e->value = e->min;
	}

	static void isr_a(void *params) {
		Encoder *e = static_cast<Encoder *>(params);
		if (ENC_GPIO_READ(e->enc_b))
			e->value++;
		else
			e->value--;

		check_limit(e);
		wakeup(e);
	}

	gpio_num_t enc_a;
	gpio_num_t enc_b;
	T value = 0;
	T step = 1;
	T min = std::numeric_limits<T>::min();
	T max = std::numeric_limits<T>::max();
	SemaphoreHandle_t waiter = nullptr;
};

#endif /* __ENCODER_H__ */
