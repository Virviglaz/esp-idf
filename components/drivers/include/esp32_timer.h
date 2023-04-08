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
 * ESP32 timer driver function
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef __ESP32_TIMER_H__
#define __ESP32_TIMER_H__

/* IDF Drivers */
#include "esp_timer.h"

/**
 * @brief Calls the callback after timeout expired.
 * 
 */
class delayed_action
{
public:
	/**
	 * @brief Default constructor.
	 * 
	 */
	delayed_action() {}

	/**
	 * @brief Construct a new delayed action object.
	 * 
	 * @param[in] timeout_us	Timeout in [us].
	 * @param[in] action		Callback function.
	 * @param[in] user_data		User data pointer.
	 * @param[in] call_from_isr	True to call from ISR, false from task.
	 */
	delayed_action(uint32_t timeout_us,
		       void (*action)(void *args),
		       void *user_data = nullptr,
		       bool call_from_isr = false)
	{
		init(timeout_us, action, user_data, call_from_isr);
	}

	/**
	 * @brief Destroy the delayed action object.
	 * 
	 */
	~delayed_action()
	{
		ESP_ERROR_CHECK(esp_timer_delete(handle));
	}

	/**
	 * @brief Start the delayed action.
	 * 
	 */
	void IRAM_ATTR run()
	{
		esp_timer_start_once(handle, _timeout_us);
	}

	/**
	 * @brief Set new timeout and start the delayed action.
	 * 
	 * @param timeout_us 
	 */
	void IRAM_ATTR run(uint32_t timeout_us)
	{
		esp_timer_start_once(handle, timeout_us);
	}

	/**
	 * @brief Initialise the default constructor.
	 * 
	 * @param[in] timeout_us	Timeout in [us].
	 * @param[in] action		Callback function.
	 * @param[in] user_data		User data pointer.
	 * @param[in] call_from_isr	True to call from ISR, false from task.
	 */
	void init(uint32_t timeout_us,
		  void (*action)(void *args),
		  void *user_data = nullptr,
		  bool call_from_isr = false)
	{
		const esp_timer_create_args_t config = {
			.callback = callback,
			.arg = this,
			.dispatch_method =
				call_from_isr ? ESP_TIMER_ISR : ESP_TIMER_TASK,
			.name = __func__,
			.skip_unhandled_events = true,
		};
		ESP_ERROR_CHECK(esp_timer_create(&config, &handle));
		_action = action;
		_user_data = user_data;
		_timeout_us = timeout_us;
	}

private:
	void (*_action)(void *args);
	esp_timer_handle_t handle;
	uint32_t _timeout_us;
	void *_user_data;

	static void IRAM_ATTR callback(void *args)
	{
		delayed_action *m = (delayed_action *)args;
		m->_action(m->_user_data);
	}
};

#endif /* __ESP32_TIMER_H__ */
