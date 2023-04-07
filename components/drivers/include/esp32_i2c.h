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
 * ESP32 i2c driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

/* FreeRTOS */
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>

/* IDF Drivers */
#include <driver/gpio.h>
#include "driver/i2c.h"

/**
 * @brief I2c driver for ESP32.
 */
template <typename R = uint8_t> class i2c
{
public:
	/**
	 * @brief Default constructor.
	 *
	 * Call i2c::init() to use I2c.
	 */
	i2c() { }

	/**
	 * @brief Initialize the I2c bus.
	 *
	 * @param[in] sda_pin		Number of SDA pin.
	 * @param[in] scl_pin		Number of SCL pin.
	 * @param[in] freq		Bus frequency. Typically 100..400kHz.
	 * @param[in] pullup		Internal pullup enable. Default = yes.
	 */
	i2c(int sda_pin,
	    int scl_pin,
	    unsigned long freq = 100000UL,
	    bool pullup = false,
	    int bus_num = 0)
	{
		init_done =
			init(sda_pin, scl_pin, freq, pullup, bus_num) == ESP_OK;
	}

	/**
	 * @brief Destroy the i2c::i2c object
	 */
	~i2c()
	{
		if (init_done) {
			vSemaphoreDelete(lock);
			i2c_driver_delete(used_bus_num);
		}
	}

	/**
	 * @brief  Write data from the buffer to device register reg.
	 *
	 * @param[in] addr		I2c device address.
	 * @param[in] reg		Pointer to register value.
	 * @param[in] reg_size		Size of register value.
	 * @param[in] buf		Pointer to data buffer.
	 * @param[in] size		Buffer size.
	 *
	 * @retval			Zero if success, error if failed.
	*/
	esp_err_t write(uint8_t addr,
			uint8_t *reg,
			uint16_t reg_size,
			uint8_t *buf,
			uint16_t size)
	{
		esp_err_t res;
		i2c_cmd_handle_t handle;

		if (!init_done)
			return no_init_error(); 

		if (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE)
			return ESP_FAIL;

		handle = i2c_cmd_link_create();
		if (!handle) {
			xSemaphoreGive(lock);
			return ESP_ERR_NO_MEM;
		}

		res = i2c_master_start(handle);
		if (res) {
			ESP_LOGE(__FILE__, "start error: %s",
				esp_err_to_name(res));
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		res = i2c_master_write_byte(handle,
			(addr << 1) | I2C_MASTER_WRITE, true);
		if (res) {
			ESP_LOGE(__FILE__, "I2C write address error: %s",
				esp_err_to_name(res));
			i2c_master_stop(handle);
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		res = i2c_master_write(handle, reg, reg_size, I2C_MASTER_NACK);
		if (res) {
			ESP_LOGE(__FILE__, "I2C write error: %s",
				esp_err_to_name(res));
			i2c_master_stop(handle);
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		if (buf && size) {
			res = i2c_master_write(handle, buf, size, false);
			if (res) {
				ESP_LOGE(__FILE__, "I2C write error: %s",
					esp_err_to_name(res));
				i2c_master_stop(handle);
				i2c_cmd_link_delete(handle);
				xSemaphoreGive(lock);
				return res;
			}
		}

		res = i2c_master_stop(handle);
		if (res) {
			ESP_LOGE(__FILE__, "I2C stop error: %s",
				esp_err_to_name(res));
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		res = i2c_master_cmd_begin(used_bus_num, handle, portMAX_DELAY);
		if (res)
			ESP_LOGE(__FILE__, "I2C reading failed: %s",
				esp_err_to_name(res));

		i2c_cmd_link_delete(handle);
		xSemaphoreGive(lock);
		return ESP_OK;
	}

	/**
	 * @brief  Read data from the device reg to buffer.
	 *
	 * @param[in] addr		I2c device address.
	 * @param[in] reg		Pointer to register value.
	 * @param[in] reg_size		Size of register value.
	 * @param[out] buf		Pointer to destanation buffer.
	 * @param[in] size		Buffer size.
	 *
	 * @retval			Zero if success, error if failed.
	*/
	esp_err_t read(uint8_t addr,
		       uint8_t *reg,
		       uint16_t reg_size,
		       uint8_t *buf,
		       uint16_t size)
	{
		esp_err_t res;
		i2c_cmd_handle_t handle;

		if (!init_done)
			return no_init_error(); 

		xSemaphoreTake(lock, portMAX_DELAY);

		handle = i2c_cmd_link_create();
		if (!handle) {
			ESP_LOGE(__FILE__, "driver install failed: no memory");
			xSemaphoreGive(lock);
			return ESP_ERR_NO_MEM;
		}

		res = i2c_master_start(handle);
		if (res) {
			ESP_LOGE(__FILE__, "I2C start error: %s",
				esp_err_to_name(res));
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		/* write address */
		res = i2c_master_write_byte(handle,
			(addr << 1) | I2C_MASTER_WRITE, true);
		if (res) {
			ESP_LOGE(__FILE__, "I2C write address error: %s",
				esp_err_to_name(res));
			i2c_master_stop(handle);
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		/* write destanation */
		res = i2c_master_write(handle, reg, reg_size, I2C_MASTER_NACK);
		if (res) {
			ESP_LOGE(__FILE__, "I2C write byte error: %s",
				esp_err_to_name(res));
			i2c_master_stop(handle);
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		/* repeated start */
		res = i2c_master_start(handle);
		if (res) {
			ESP_LOGE(__FILE__, "I2C repeated start error: %s",
				esp_err_to_name(res));
			i2c_master_stop(handle);
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		/* write address for reading */
		res = i2c_master_write_byte(handle,
			(addr << 1) | I2C_MASTER_READ, true);
		if (res) {
			ESP_LOGE(__FILE__, "I2C address write error: %s",
				esp_err_to_name(res));
			i2c_master_stop(handle);
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}
		
		if (buf && size > 1) {
			res = i2c_master_read(handle, buf, size - 1,
				I2C_MASTER_ACK);
			if (res) {
				ESP_LOGE(__FILE__, "I2C reading error: %s",
					esp_err_to_name(res));
				i2c_master_stop(handle);
				i2c_cmd_link_delete(handle);
				xSemaphoreGive(lock);
				return res;
			}
			buf += size - 1;
			size = 1;
		}

		/* last byte read with NACK */
		if (buf && size) {
			res = i2c_master_read(handle, buf, 1, I2C_MASTER_NACK);
			if (res) {
				ESP_LOGE(__FILE__, "I2C reading error: %s",
					esp_err_to_name(res));
				i2c_master_stop(handle);
				i2c_cmd_link_delete(handle);
				xSemaphoreGive(lock);
				return res;
			}
		}

		res = i2c_master_stop(handle);
		if (res) {
			ESP_LOGE(__FILE__, "I2C stop error: %s",
				esp_err_to_name(res));
			i2c_cmd_link_delete(handle);
			xSemaphoreGive(lock);
			return res;
		}

		res = i2c_master_cmd_begin(used_bus_num, handle, portMAX_DELAY);
		if (res)
			ESP_LOGE(__FILE__, "I2C reading failed: %s",
				esp_err_to_name(res));
		i2c_cmd_link_delete(handle);
		xSemaphoreGive(lock);
		return res;
	}

	esp_err_t init(int sda_pin,
	    	       int scl_pin,
		       unsigned long freq = 100000UL,
		       bool pullup = false,
		       int bus_num = 0)
	{
		esp_err_t res;

		if (init_done) {
			ESP_LOGE(__FILE__, "Already initialized");
			return ESP_ERR_INVALID_STATE;
		}

		if (bus_num >= I2C_NUM_MAX) {
			ESP_LOGE(__FILE__, "No more I2c busses available");
			return ESP_ERR_INVALID_ARG;
		}

		i2c_config_t cfg;
		cfg.mode = I2C_MODE_MASTER;
		cfg.sda_io_num = sda_pin;
		cfg.sda_pullup_en = pullup;
		cfg.scl_io_num = scl_pin;
		cfg.scl_pullup_en = pullup;
		cfg.master.clk_speed = freq;
		cfg.clk_flags = 0;

		gpio_reset_pin((gpio_num_t)sda_pin);
		gpio_reset_pin((gpio_num_t)scl_pin);

		res = i2c_param_config(bus_num, &cfg);
		if (res) {
			ESP_LOGE(__FILE__, "configuration failed: %s",
				esp_err_to_name(res));
			return res;
		}

		res = i2c_driver_install(bus_num, cfg.mode, 0, 0, 0);
		if (res) {
			ESP_LOGE(__FILE__, "driver install failed: %s",
				esp_err_to_name(res));
			return res;
		}

		lock = xSemaphoreCreateMutex();
		used_bus_num = bus_num;
		init_done = lock ? true : false;

		return init_done ? ESP_OK : ESP_ERR_NO_MEM;
	}

	/**
	 * @brief  Write data to the device reg from buffer.
	 *
	 * @param[in] addr		I2c device address.
	 * @param[in] reg		Register address value.
	 * @param[in] buf		Pointer to data buffer.
	 * @param[in] size		Buffer size.
	 *
	 * @retval			Zero if success, error if failed.
	*/
	esp_err_t write_reg(uint8_t addr,
			    R reg,
			    uint8_t *buf,
			    uint16_t size)
	{
		return write(addr, &reg, sizeof(reg), buf, size);
	}

	/**
	 * @brief  Read data from the device reg to buffer.
	 *
	 * @param[in] addr		I2c device address.
	 * @param[in] reg		Register address value.
	 * @param[in] buf		Pointer to data buffer.
	 * @param[in] size		Buffer size.
	 *
	 * @retval			Zero if success, error if failed.
	*/
	esp_err_t read_reg(uint8_t addr,
			   R reg,
			   uint8_t *buf,
			   uint16_t size)
	{
		return read(addr, &reg, sizeof(reg), buf, size);
	}

private:
	SemaphoreHandle_t lock;
	bool init_done = false;
	int used_bus_num;

	esp_err_t no_init_error()
	{
		ESP_LOGE(__FILE__, "I2c not initialized");
		return ESP_ERR_INVALID_STATE;
	}
};
