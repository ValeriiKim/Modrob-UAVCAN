#define NUNAVUT_ASSERT(x) assert(x) // объявление макроса для работы serialization.h для DSDL
// Прошивка когнитивного субмодуля транспортного модуля

// Драйверы микроконтроллера
#include "main.hpp"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "timer2.h"
#include "bxcan.h"

// Библиотеки протокола, o1heap, cQueue
#include "canard.h"
#include "o1heap.h"
#include "cQueue.h"

#include "math.h"
#include "algorithm"

// DSDL Headers (uavcan namespace)
#include "uavcan/node/Heartbeat_1_0.h"
#include "modrob/transport_module/cognition_setpoint_0_1.h"
// #include "modrob/sensor_module/sensor_data_0_1.h"
#include "modrob/sensor_module/sensor_data_0_2.h"

void SystemClock_Config(void);
void CAN_clk_gpio_init();
void bxCAN_interrupts_enable();
void bxCAN_interrupts_disable();

constexpr uint32_t USEC_IN_SEC = 1000000; // секунда выраженная в мкс
float Vx = 0, Vy = 0;
bool get_new_sensor_data = false;

O1HeapInstance *o1heap_ins;
static constexpr size_t HEAP_SIZE = 51200; // 51200
uint8_t _base[HEAP_SIZE] __attribute__((aligned(O1HEAP_ALIGNMENT))); // Create pointer to the memory arena for the HEAP

void *memAllocate(CanardInstance *const ins, const size_t amount)
{
	(void)ins;
	return o1heapAllocate(o1heap_ins, amount);
}

void memFree(CanardInstance *const ins, void *const pointer)
{
	(void)ins;
	o1heapFree(o1heap_ins, pointer);
}

Queue_t can1rx_queue, can2rx_queue;

struct CAN_message
{
	uint64_t microseconds;
	uint32_t extended_can_id;
	size_t payload_size = CANARD_MTU_CAN_CLASSIC;
	uint8_t payload[CANARD_MTU_CAN_CLASSIC] = {0};

	CAN_message()
	{
		microseconds = 0;
		extended_can_id = 0;
		payload_size = CANARD_MTU_CAN_CLASSIC;
		memset(payload, 0, 8);
	}
};

/** Функция, которая для каждой точки статического препятствия проверяет, приведёт ли эта
 * точка к столкновению с роботом на следующем временном шаге, на котором положение робота
 * будет иметь координаты x и y
 * @param points_x массив координат X точек
 * @param points_y массив координат Y точек
 * @param count количество точек скана
 * @param x координата X робота на следующем временном шаге
 * @param y координата Y робота на следующем временном шаге
 * @return true, если хотя бы одна точка вызовет столкновение, false - ни одна точка не вызовет столкновение
 **/
inline bool static_collision(float points_x[], float points_y[], size_t count, float x, float y)
{
	bool result;
	for (size_t i = 0; i < count; i++)
	{
		if (sqrtf(powf((points_x[i] - x), 2) + powf((points_y[i] - y), 2)) <= 0.45)
		{
			result = true;
			break;
		}
		else
		{
			result = false;
		}
	}
	return result;
}

/** Функция, которая для каждого отрезка прямой статического препятствия проверяет, приведёт ли этот
 * сегмент к столкновению с роботом на следующем временном шаге, на котором положение робота
 * будет иметь координаты x и y
 * @param points_x массив координат X точек
 * @param points_y массив координат Y точек
 * @param count количество точек скана
 * @param x координата X робота на следующем временном шаге
 * @param y координата Y робота на следующем временном шаге
 * @return true, если хотя бы один сегмент вызовет столкновение, false - ни один сегмент не вызовет столкновение
 **/
inline bool static_segment_collision(modrob_sensor_module_segment_params_0_1 segments[], size_t count, float x, float y, float safe_dist_squared)
{
	bool result;
	float segment_x, segment_y;
	float px_seg, py_seg;
	float dot, seg_len_squared;
	float param = -1;
	float xx, yy;
	for (size_t i = 0; i < count; i++)
	{
		segment_x = segments[i].x2 - segments[i].x1;
		segment_y = segments[i].y2 - segments[i].y1;
		px_seg = x - segments[i].x1;
		py_seg = y - segments[i].y1;
		dot = px_seg*segment_x + py_seg*segment_y;
		seg_len_squared = powf(segment_x, 2) + powf(segment_y, 2);
		param = dot/seg_len_squared;
		if (param < 0)
		{
			xx = segments[i].x1;
			yy = segments[i].y1;
		}
		else if (param > 1)
		{
			xx = segments[i].x2;
			yy = segments[i].y2;
		}
		else
		{
			xx = segments[i].x1 + param*segment_x;
			yy = segments[i].y1 + param*segment_y;
		}

		float min_dist_squared = powf(xx-x, 2) + powf(yy-y, 2);
		if (min_dist_squared <= safe_dist_squared)
		{
			result = true;
			// break;
		}
		else
		{
			result = false;
		}
	}
	return result;
}

/** Функция навигации транспортного модуля, которая копирует содержание функции Navigation
 * в блоке TM local planner блока Transport module модели Simulink с названием navigation_robot3_5. 
 * Для движения к целевой точке используется линейный регулятор, для объезда подвижных препятствий 
 * применяется алгоритм Velocity Obstacles (VO).
 * @param sdata указатель на сенсорные данные, получаемые от сенсорного модуля
 * @param Vx проекция текущей скорости робота на ось X
 * @param Vy проекция текущей скорости робота на ось Y
 **/
void Navigation(modrob_sensor_module_sensor_data_0_2 *sdata, float &Vx, float &Vy)
{
	float goal_x = 3;
	float goal_y = 0;
	float radi_r = 0.15;
	float clearance = 0.05;
	float ax = 4.65, ay = 4.65;
	float dt = 0.1;
	float Xa = sdata->cur_pos[0];
	float Ya = sdata->cur_pos[1];
	// float Vx = 0, Vy = 0;
	float X_next, Y_next;
	float safe_dist = 0.45; // robot_radius (0.3 m) + 0.15 m
	float safe_dist_squared = powf(safe_dist, 2);

	float Vdesx;
	float Vdesy;
	float Vrx = 0, Vry = 0;
	float Vxmin;
	float Vxmax;
	float Vymin;
	float Vymax;

	float *theta_left = new float[sdata->obstacles.count];
	float *theta_right = new float[sdata->obstacles.count];
	float dist_AB;
	float dist_to_obst;
	float dist_t;
	float theta_AB;
	float alpha;

	// LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_3);
	for (size_t i = 0; i < sdata->obstacles.count; i++)
	{
		dist_AB = sqrtf(powf(sdata->obstacles.elements[i].obst_pos[0] - Xa, 2) +
						powf(sdata->obstacles.elements[i].obst_pos[1] - Ya, 2));
		dist_to_obst = dist_AB - radi_r - sdata->obstacles.elements[i].radius;
		if (dist_to_obst <= clearance)
		{
			break;
		}
		else if (dist_AB <= 3)
		{
			dist_t = sqrtf(powf(dist_AB, 2) + powf(radi_r + sdata->obstacles.elements[i].radius + clearance, 2));
			theta_AB = atan2f(sdata->obstacles.elements[i].obst_pos[1] - Ya, sdata->obstacles.elements[i].obst_pos[0] - Xa);
			alpha = atan2f(radi_r + sdata->obstacles.elements[i].radius, dist_t);
			theta_right[i] = theta_AB - alpha;
			theta_left[i] = theta_AB + alpha;
		}
		else
		{
			continue;
		}
	}

	float L = sqrtf(powf(goal_x - Xa, 2) + powf(goal_y - Ya, 2));
	float gamma = atan2f(goal_y - Ya, goal_x - Xa);
	Vdesx = 0.7 * L * cosf(gamma);
	Vdesy = 0.7 * L * sinf(gamma);
	Vxmin = Vrx - ax * dt;
	Vxmax = Vrx + ax * dt;
	Vymin = Vry - ay * dt;
	Vymax = Vry + ay * dt;
	// LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
	float theta_VAB = 0;
	float numV = 10;
	float Vx_RV[10], Vy_RV[10];
	float step_x = (Vxmax - Vxmin) / (numV - 1);
	float step_y = (Vymax - Vymin) / (numV - 1);
	for (size_t n = 0; n < numV; n++)
	{
		Vx_RV[n] = Vxmin + n * step_x;
	}
	for (size_t n = 0; n < numV; n++)
	{
		Vy_RV[n] = Vymin + n * step_y;
	}
	uint16_t num_collision_vels = 0;
	// LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_3);
	for (size_t i = 0; i < numV; i++)
	{
		for (size_t j = 0; j < numV; j++)
		{
			for (size_t k = 0; k < sdata->obstacles.count; k++)
			{
				theta_VAB = atan2f(Vy_RV[j] - sdata->obstacles.elements[k].obst_vel[1],
								   Vx_RV[i] - sdata->obstacles.elements[k].obst_vel[0]);
				if ((theta_VAB >= theta_right[i]) && (theta_VAB <= theta_left[i]))
				{
					Vx_RV[i] = 999;
					Vy_RV[j] = 999;
					num_collision_vels++;
				}
			}
			// if (Vx_RV[i] != 999)
			// {
			X_next = Xa + Vx_RV[i] * 0.2;
			Y_next = Ya + Vy_RV[j] * 0.2;
			bool check = static_segment_collision(sdata->segments.elements, sdata->segments.count, X_next, Y_next, safe_dist_squared);
			if (check)
			{
				Vx_RV[i] = 999;
				Vy_RV[j] = 999;
				num_collision_vels++;
			}
			// }
		}
	}
	delete[] theta_left;
	delete[] theta_right;
	// LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
	float dV;
	if (num_collision_vels == numV*numV)
	{
		Vx = 0;
		Vy = 0;
	}
	else
	{
		size_t k = 0;
		size_t min_ix = 0;
		size_t min_iy = 0;
		float min_dV = sqrtf(powf(Vx_RV[0] - Vdesx, 2) + powf(Vy_RV[0] - Vdesy, 2));

		for (size_t i = 0; i < numV; i++)
		{
			for (size_t j = 0; j < numV; j++)
			{
				dV = sqrtf(powf(Vx_RV[i] - Vdesx, 2) + powf(Vy_RV[j] - Vdesy, 2));
				if (dV <= min_dV)
				{
					min_dV = dV;
					min_ix = i;
					min_iy = j;
				}
			}
		}
		Vx = Vx_RV[min_ix];
		Vy = Vy_RV[min_iy];
	}
}

bool publish_heartbeat(CanardInstance *const canard_ins, CanardTxQueue *tx_queue, const uavcan_node_Heartbeat_1_0 *const hb, uint64_t micros)
{
	static CanardTransferID transfer_id = 0;
	uint8_t payload[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
	size_t inout_buffer_size_bytes = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
	if (uavcan_node_Heartbeat_1_0_serialize_(hb, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS)
	{
		return false; // произошла ошибка сериализации
	}
	const CanardTransferMetadata transfer_metadata =
		{
			/* .priority       = */ CanardPriorityNominal,
			/* .transfer_kind  = */ CanardTransferKindMessage,
			/* .port_id        = */ uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
			/* .remote_node_id = */ CANARD_NODE_ID_UNSET,
			/* .transfer_id    = */ transfer_id,
		};
	int32_t result = canardTxPush(tx_queue,
								  canard_ins,
								  micros + 2 * USEC_IN_SEC,
								  &transfer_metadata,
								  inout_buffer_size_bytes,
								  payload);
	bool const success = (result >= 0);
	++transfer_id;

	if (result < 0)
	{
		// usart2::UART_send_float(0.23, true);
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
	}
	return result;
}

// Функция для отправки задающих воздействий на вторую шину CAN для субмодулей
bool publish_setpoint(CanardInstance *const canard_ins,
					  CanardTxQueue *tx_queue,
					  const modrob_transport_module_cognition_setpoint_0_1 *const setp,
					  uint64_t micros)
{
	static CanardTransferID transfer_id = 0;
	uint8_t payload[modrob_transport_module_cognition_setpoint_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
	size_t inout_buffer_size_bytes = modrob_transport_module_cognition_setpoint_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
	if (modrob_transport_module_cognition_setpoint_0_1_serialize_(setp, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS)
	{
		return false;
	}
	const CanardTransferMetadata transfer_metadata =
		{
			/* .priority       = */ CanardPriorityNominal,
			/* .transfer_kind  = */ CanardTransferKindMessage,
			/* .port_id        = */ 10,
			/* .remote_node_id = */ CANARD_NODE_ID_UNSET,
			/* .transfer_id    = */ transfer_id,
		};
	int32_t result = canardTxPush(tx_queue,
								  canard_ins,
								  micros + 100000,
								  &transfer_metadata,
								  inout_buffer_size_bytes,
								  payload);
	bool const success = (result >= 0);
	++transfer_id;

	if (result < 0)
	{
		// usart2::UART_send_float(0.23, true);
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
	}
	return result;
}

// Общая функция обработки входящих canard сообщений 
void process_received_transfer(const CanardRxTransfer *transfer)
{

	if ((transfer->metadata.transfer_kind == CanardTransferKindMessage) &&
		(transfer->metadata.port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_))
	{
		uavcan_node_Heartbeat_1_0 hb_remote;
		size_t inout_buffer_size_bytes = transfer->payload_size;
		int res = uavcan_node_Heartbeat_1_0_deserialize_(&hb_remote, (uint8_t *)(transfer->payload), &inout_buffer_size_bytes);

		UART_send_float(hb_remote.vendor_specific_status_code, true);
		if (res < 0)
		{
			// usart2::usart_send_string("HERE!\n");
		}
	}

	if ((transfer->metadata.transfer_kind == CanardTransferKindMessage) &&
		(transfer->metadata.port_id == 150))
	{
		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_3);
		modrob_sensor_module_sensor_data_0_2 sensor_data;
		size_t inout_buffer_size_bytes = transfer->payload_size;
		int res = modrob_sensor_module_sensor_data_0_2_deserialize_(&sensor_data, (uint8_t *)(transfer->payload), &inout_buffer_size_bytes);
		Navigation(&sensor_data, Vx, Vy);
		get_new_sensor_data = true;
		if (res < 0)
		{
			UART_send_string("HERE!\n");
		}
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
	}
}

void send_transmission_queue(CanardInstance *const canard_ins, CanardTxQueue *tx_queue, const uint8_t iface_index, uint64_t micros)
{
	for (const CanardTxQueueItem *item = NULL; (item = canardTxPeek(tx_queue)) != NULL;) // Look at the top of the TX queue.
	{
		if ((0U == item->tx_deadline_usec) || (item->tx_deadline_usec > micros)) // Check the deadline.
		{
			if (not bxCANPush(iface_index,
							  micros,
							  item->tx_deadline_usec,
							  item->frame.extended_can_id,
							  item->frame.payload_size,
							  item->frame.payload))
			{
				break; // If the driver is busy, break and retry later.
			}
		}
		canard_ins->memory_free(canard_ins, canardTxPop(tx_queue, item));
	}
}

// Функция обработки входящих CAN фреймов, поступающих по первой шине
bool receive_transfers_can1(CanardInstance *const canard_ins)
{
	CanardRxTransfer transfer;
	const uint8_t iface_index = 0;
	while (not q_isEmpty(&can1rx_queue))
	{
		CAN_message can_msg;
		CanardFrame rxf;
		bxCAN_interrupts_disable();
		q_pop(&can1rx_queue, &can_msg);
		bxCAN_interrupts_enable();
		rxf.extended_can_id = can_msg.extended_can_id;
		rxf.payload = can_msg.payload;
		rxf.payload_size = can_msg.payload_size;
		const int8_t res = canardRxAccept(canard_ins, can_msg.microseconds, &rxf, iface_index, &transfer, NULL);
		if (res == 1)
		{
			process_received_transfer(&transfer);
			canard_ins->memory_free(canard_ins, transfer.payload);
		}
		else if (res < 0)
		{
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			return false;
		}
		else
		{
		}
	}
	return true;
}

// Функция обработки входящих CAN фреймов, поступающих по второй шине
bool receive_transfers_can2(CanardInstance *const canard_ins)
{
	CanardRxTransfer transfer;
	const uint8_t iface_index = 1;
	while (not q_isEmpty(&can2rx_queue))
	{
		CAN_message can_msg;
		CanardFrame rxf;
		bxCAN_interrupts_disable();
		q_pop(&can2rx_queue, &can_msg);
		bxCAN_interrupts_enable();
		rxf.extended_can_id = can_msg.extended_can_id;
		rxf.payload = can_msg.payload;
		rxf.payload_size = can_msg.payload_size;
		const int8_t res = canardRxAccept(canard_ins, can_msg.microseconds, &rxf, iface_index, &transfer, NULL);
		if (res == 1)
		{
			process_received_transfer(&transfer);
			canard_ins->memory_free(canard_ins, transfer.payload);
		}
		else if (res < 0)
		{
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			return false;
		}
		else
		{
		}
	}
	return true;
}

int main()
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	timer2::tim2_setup();
	timer2::tim2_start();
	uint8_t str[] = "USART Transmit DMA\r\n";
	CAN_clk_gpio_init();

	BxCANTimings can_timings;
	bool res = bxCANComputeTimings(HAL_RCC_GetPCLK1Freq(), 1000000, &can_timings);
	if (!res)
	{
		// The requested bit rate cannot be set up!
	}
	res = bxCANConfigure(0, can_timings, false);
	if (!res)
	{
		// Can't configure bxCAN1!
	}
	res = bxCANConfigure(1, can_timings, false);
	if (!res)
	{
		// Can't configure bxCAN2!
	}
	bxCAN_interrupts_enable();
	o1heap_ins = o1heapInit(_base, HEAP_SIZE);
	CanardInstance canard_ins = canardInit(&memAllocate, &memFree);
	CanardTxQueue tx_queue_can1 = canardTxInit(700, CANARD_MTU_CAN_CLASSIC);
	CanardTxQueue tx_queue_can2 = canardTxInit(700, CANARD_MTU_CAN_CLASSIC);
	canard_ins.node_id = MODULE_ID; // Set equal to MODULE_ID which is edited in platformio.ini
	q_init(&can1rx_queue, sizeof(CAN_message), 512, FIFO, true);
	q_init(&can2rx_queue, sizeof(CAN_message), 256, FIFO, true);

	// usart2::UART_send_string("bxCAN controller initialized and started\n");

	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
	uint64_t prev_time = timer2::get_micros();
	uint64_t next_1_hz_timestep = USEC_IN_SEC;
	uint64_t next_10_hz_timestep = 100000;

	uavcan_node_Heartbeat_1_0 hb =
		{
			timer2::get_micros() / USEC_IN_SEC,
			uavcan_node_Health_1_0_NOMINAL,
			uavcan_node_Mode_1_0_OPERATIONAL,
			0xA1, // max FF (255)
		};

	modrob_transport_module_cognition_setpoint_0_1 setpoint = {};
	setpoint.angle.count = 4;
	setpoint.angle.elements[0] = 0.0;
	setpoint.angle.elements[1] = 0.0;
	setpoint.angle.elements[2] = 0.0;
	setpoint.angle.elements[3] = 0.0;
	setpoint.angular_velocity.count = 4;
	setpoint.angular_velocity.elements[0] = 0.0;
	setpoint.angular_velocity.elements[1] = 0.0;
	setpoint.angular_velocity.elements[2] = 0.0;
	setpoint.angular_velocity.elements[3] = 0.0;

	static CanardRxSubscription heartbeat_subscription;
	canardRxSubscribe(&canard_ins, CanardTransferKindMessage,
					  uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
					  12,
					  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					  &heartbeat_subscription);

	static CanardRxSubscription sensor_data_subscription;
	canardRxSubscribe(&canard_ins, CanardTransferKindMessage,
					  150,
					  1200,
					  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					  &sensor_data_subscription);

	while (1)
	{

		hb.uptime = timer2::get_micros() / USEC_IN_SEC;

		if (timer2::get_micros() >= next_1_hz_timestep)
		{
			next_1_hz_timestep += USEC_IN_SEC;
			publish_heartbeat(&canard_ins, &tx_queue_can1, &hb, timer2::get_micros());
		}
		// if (timer2::get_micros() >= next_10_hz_timestep)
		// {
		// 	next_10_hz_timestep += 100000;
		// 	publish_setpoint(&canard_ins, &tx_queue_can2, &setpoint, timer2::get_micros());
		// }
		if (get_new_sensor_data)
		{
			setpoint.angular_velocity.elements[0] = 0.1*(Vx - Vy);
			setpoint.angular_velocity.elements[1] = 0.1*(Vx + Vy);
			setpoint.angular_velocity.elements[2] = 0.1*(Vx + Vy);
			setpoint.angular_velocity.elements[3] = 0.1*(Vx - Vy);
			publish_setpoint(&canard_ins, &tx_queue_can2, &setpoint, timer2::get_micros());
			get_new_sensor_data = false;
		}

		send_transmission_queue(&canard_ins, &tx_queue_can1, 0, timer2::get_micros());
		send_transmission_queue(&canard_ins, &tx_queue_can2, 1, timer2::get_micros());
		receive_transfers_can1(&canard_ins);
		receive_transfers_can2(&canard_ins);
	}
	return 0;
}

/* Нужно добавлять для обработчиков прерываний обёртку, которая позволяет воспринимать
участок кода как код языка С (НЕ С++). Без этого функции обработчиков
прерываний не хотят работать  */

extern "C"
{
	/**
  * @brief This function handles System tick timer.
  */
	void SysTick_Handler(void)
	{
		HAL_IncTick();
	}
}

// обработчик прерывания от памяти к периферии (USART2-TX) для дебага
extern "C"
{
	void DMA1_Stream6_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(&hdma_usart2_tx);
	}
}

extern "C"
{
	/* С каждым прерыванием увеличиваем счётчик переполнений */
	void TIM2_IRQHandler()
	{
		timer2::tim2_upcount();
		// node.upcount_realtime();
	}
}

extern "C"
{
	/* Обработка прерывания по переносу данных из сдвигового регистра в USART_DR при приёме данных по RX */
	void USART2_IRQHandler(void)
	{
		HAL_UART_IRQHandler(&huart2);
		// commander.serial_commander_IRQHandler();
	}
}

/** Обработчик прерывания по приёму фреймов по CAN1, прерывание происходит, когда
 * в FIFO0 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void CAN1_RX0_IRQHandler(void)
	{
		if (CAN1->RF0R & CAN_RF0R_FMP0) // FMP0 > 0?
		{
			CAN_message can_msg;
			bxCANPop(0, &can_msg.extended_can_id, &can_msg.payload_size, can_msg.payload);
			can_msg.microseconds = timer2::get_micros();
			q_push(&can1rx_queue, &can_msg);
		}
	}
}

/** Обработчик прерывания по приёму фреймов по CAN1, прерывание происходит, когда
 * в FIFO1 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void CAN1_RX1_IRQHandler(void)
	{
		if (CAN1->RF1R & CAN_RF1R_FMP1) // FMP1 > 0?
		{
			CAN_message can_msg;
			bxCANPop(0, &can_msg.extended_can_id, &can_msg.payload_size, can_msg.payload);
			can_msg.microseconds = timer2::get_micros();
			q_push(&can1rx_queue, &can_msg);
		}
	}
}

/** Обработчик прерывания по приёму фреймов по CAN2, прерывание происходит, когда
 * в FIFO0 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void CAN2_RX0_IRQHandler(void)
	{
		if (CAN2->RF0R & CAN_RF0R_FMP0) // FMP0 > 0?
		{
			CAN_message can_msg;
			bxCANPop(1, &can_msg.extended_can_id, &can_msg.payload_size, can_msg.payload);
			can_msg.microseconds = timer2::get_micros();
			q_push(&can2rx_queue, &can_msg);
		}
	}
}

/** Обработчик прерывания по приёму фреймов по CAN2, прерывание происходит, когда
 * в FIFO1 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void CAN2_RX1_IRQHandler(void)
	{
		if (CAN2->RF1R & CAN_RF1R_FMP1) // FMP1 > 0?
		{
			CAN_message can_msg;
			bxCANPop(1, &can_msg.extended_can_id, &can_msg.payload_size, can_msg.payload);
			can_msg.microseconds = timer2::get_micros();
			q_push(&can2rx_queue, &can_msg);
		}
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
	{
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1)
	{
	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1)
	{
	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{
	}
	LL_SetSystemCoreClock(168000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
	{
		// Error_Handler();
	}
}

void CAN_clk_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	__HAL_RCC_CAN2_CLK_ENABLE();
	/**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* CAN1 interrupt Init */
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

	/**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* CAN2 interrupt Init */
	HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
}

void bxCAN_interrupts_enable()
{
	//  Interrupt generated when get new frame in FIFO0 or FIFO1
	SET_BIT(CAN1->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);

	SET_BIT(CAN2->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
}

void bxCAN_interrupts_disable()
{
	CLEAR_BIT(CAN1->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);

	CLEAR_BIT(CAN2->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
}