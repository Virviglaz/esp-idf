#ifndef __STEP_MOTOR_H__
#define __STEP_MOTOR_H__

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <errno.h>

#ifndef SIMULATION
#include "driver/gpio.h"
#include "esp_timer.h"
#include "log.h"
#define STP_GPIO_RESET(p)	gpio_reset_pin((gpio_num_t)(p))
#define STP_GPIO_INIT(p)	gpio_set_direction((gpio_num_t)(p), \
					GPIO_MODE_OUTPUT)
#define STP_GPIO_SET(p, s)	gpio_set_level((gpio_num_t)p, s)
#define DBG			DEBUG
#else
#define STP_GPIO_RESET(p)		0
#define STP_GPIO_INIT(p)		0
#define STP_GPIO_SET(p, s)		0
#define DBG(format, ...)		0
#endif /* SIMULATION*/

#define MIN(x, y)	(x) < (y) ? (x) : (y)
#define MAX(x, y)	(x) > (y) ? (x) : (y)

class Step_motor
{
public:
	Step_motor(int ena_pin,
		   int clk_pin,
		   int dir_pin,
		   float acc,
		   uint32_t pos_clk_us = 50,
		   bool (*check_stop)(bool CW) = nullptr) :
			_ena_pin(ena_pin),
			_clk_pin(clk_pin),
			_dir_pin(dir_pin),
			_acc(acc),
			_pos_clk_us(pos_clk_us),
			_check_stop(check_stop) {}
	~Step_motor() {
		clear();
		esp_timer_stop(handle);
		esp_timer_delete(handle);
		if (waiter)
			vSemaphoreDelete(waiter);
	}

	int init() {
		const esp_timer_create_args_t config = {
			.callback = callback,
			.arg = this,
			.dispatch_method = ESP_TIMER_TASK,
			.name = __func__,
			.skip_unhandled_events = true,
		};
		
		ESP_ERROR_CHECK(STP_GPIO_RESET(_ena_pin));
		ESP_ERROR_CHECK(STP_GPIO_RESET(_clk_pin));
		ESP_ERROR_CHECK(STP_GPIO_RESET(_dir_pin));

		ESP_ERROR_CHECK(STP_GPIO_INIT(_ena_pin));
		ESP_ERROR_CHECK(STP_GPIO_INIT(_clk_pin));
		ESP_ERROR_CHECK(STP_GPIO_INIT(_dir_pin));
		
		ESP_ERROR_CHECK(esp_timer_create(&config, &handle));

		waiter = xSemaphoreCreateBinary();
		ESP_ERROR_CHECK(waiter == nullptr);

		/* acceleration */
		acc_step_per_us = _acc / 1E12;

		per_slowest_step = 1000000.0 / sqrt(2.0 * _acc);

		return 0;
	}

	void add_segment(int steps, float speed) {
		bool reverse = false;
		int32_t steps_to_run;
		if (steps < 0) {
			steps = -steps;
			reverse = true;
		}

		float cur_per_sqrt = speed * speed;
		int32_t acc_steps = (int32_t)round(5E11 / (_acc * cur_per_sqrt));

		if (steps <= 2 * acc_steps) {
			while (2 * acc_steps > steps)
				acc_steps--;

			steps = 0;
			steps_to_run = 2 * acc_steps;
		} else
			steps_to_run = steps - 2 * acc_steps;
		float cur_step_per_in_us = per_slowest_step;
		int32_t period_us_per_step = (int32_t)(1000000.0 / speed);
		DBG("Acc steps: %ld", acc_steps);
		DBG("Total steps: %ld", steps_to_run);
		DBG("Run speed: %ld [us]", period_us_per_step);
		DBG("Clock pulse: %ld [us]", _pos_clk_us);

		Segment *s = new Segment(acc_steps, period_us_per_step, steps);

		for (int32_t i = 0; i != acc_steps; i++) {
			cur_per_sqrt = cur_step_per_in_us * cur_step_per_in_us;
			s->_acc_steps[i] = MAX(_pos_clk_us + 1, cur_step_per_in_us -
				acc_step_per_us * cur_per_sqrt * cur_step_per_in_us);
			if (s->_acc_steps[i] < period_us_per_step)
				s->_acc_steps[i] = period_us_per_step;
			cur_step_per_in_us = s->_acc_steps[i];
			DBG("ACC step %ld: %lu [us]", i, s->_acc_steps[i]);
		}

		if (reverse)
			s->dir = Segment::CCW;

		job.push_back(s);
	}

	void enable() {
		STP_GPIO_SET(_ena_pin, 1);
	}

	void disable() {
		STP_GPIO_SET(_ena_pin, 0);
	}

	void run() {
		taskDISABLE_INTERRUPTS();
		force_stop = false;
		int i = 0;
		for (const auto& segment : job) {
			if (segment->is_done()) {
				delete(segment);
				job.erase(job.begin() + i);
			}
			i++;
		}

		taskENABLE_INTERRUPTS();
		enable();
		esp_timer_start_once(handle, _pos_clk_us);
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

	bool is_running() {
		bool done = true;
		taskDISABLE_INTERRUPTS();
		for (const auto& segment : job)
			if (!segment->is_done())
				done = false;
		taskENABLE_INTERRUPTS();
		return done;
	}

	void clear() {
		for (const auto& segment : job)
			delete(segment);
		job.clear();
	}

	void stop() {
		force_stop = true;
	}

private:
	static void callback(void *args) {
		Step_motor *m = (Step_motor *)args;
		for (const auto& segment : m->job) {
			uint32_t start_in_us = segment->get_period_us(m);
			if (start_in_us) {
				esp_timer_start_once(m->handle, start_in_us);
				return;
			}

			if (m->_check_stop &&
				m->_check_stop(segment->dir == Segment::CW))
				m->force_stop = true;
		}

		xSemaphoreGiveFromISR(m->waiter, NULL);
	}

	class Segment {
	public:
		Segment(int32_t acc_steps,
			float run_period,
			uint32_t run_steps) {
			_nof_acc_steps = acc_steps;
			_run_period = run_period;
			_run_steps = run_steps;
			_acc_steps = (uint32_t *)malloc(acc_steps *
				sizeof(acc_steps));
		}

		~Segment() {
			free(_acc_steps);
		}

		uint32_t get_period_us(Step_motor *m) {
			uint32_t start_in_us = 0;

			if (m->force_stop) {
				state = DONE;
				return 0;
			}

			if (state == DONE)
				return 0;

			if (m->clk_edge == NEG) {
				STP_GPIO_SET(m->_clk_pin, 0);
				m->clk_edge = POS;
				return m->_pos_clk_us;
			}

			STP_GPIO_SET(m->_clk_pin, 1);
			STP_GPIO_SET(m->_dir_pin, dir == CCW);
			m->clk_edge = NEG;
			switch (state) {
			case IDLE:
			case ACC:
				start_in_us = _acc_steps[step_num];
				step_num++;
				if (step_num >= _nof_acc_steps) {
					step_num = 0;
					state = _run_steps ? RUN : DEC;
				}
				break;
			case RUN:
				start_in_us = _run_period;
				step_num++;
				if (step_num >= _run_steps) {
					step_num = 0;
					state = DEC;
				}
				break;
			case DEC:
				start_in_us = _acc_steps[_nof_acc_steps -
					step_num - 1];
				step_num++;
				if (step_num >= _nof_acc_steps) {
					step_num = 0;
					state = DONE;
				}
				break;
			case DONE:
				return 0;
			}

			return start_in_us;
		}

		bool is_done() {
			return state == DONE;
		}

		uint32_t *_acc_steps = nullptr;
		float _run_period;
		uint32_t _run_steps;
		uint32_t _nof_acc_steps;
		uint32_t step_num = 0;
		enum { IDLE, ACC, RUN, DEC, DONE } state = IDLE;
		enum { CW, CCW } dir = CW;
	};

	int _ena_pin;
	int _clk_pin;
	int _dir_pin;
	float _acc;
	uint32_t _pos_clk_us;

	float per_slowest_step = 0;
	float acc_step_per_us = 0;

	enum { POS, NEG } clk_edge = POS;

	esp_timer_handle_t handle;
	std::vector<Segment *> job;
	SemaphoreHandle_t waiter = nullptr;
	bool force_stop = false;
	bool (*_check_stop)(bool CW);
};

#endif /* __STEP_MOTOR_H__ */
