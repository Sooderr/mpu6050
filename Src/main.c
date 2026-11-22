#include <stdint.h>

#define RCC_AHB1ENR   (*(volatile uint32_t*)0x40023830U)
#define RCC_APB1ENR   (*(volatile uint32_t*)0x40023840U)

#define GPIOB_MODER   (*(volatile uint32_t*)0x40020400U)
#define GPIOB_OTYPER  (*(volatile uint32_t*)0x40020404U)
#define GPIOB_PUPDR   (*(volatile uint32_t*)0x4002040CU)
#define GPIOB_IDR     (*(volatile uint32_t*)0x40020410U)   // НОВОЕ
#define GPIOB_ODR     (*(volatile uint32_t*)0x40020414U)   // НОВОЕ
#define GPIOB_AFRH    (*(volatile uint32_t*)0x40020424U)

#define I2C1_CR1      (*(volatile uint32_t*)0x40005400U)
#define I2C1_CR2      (*(volatile uint32_t*)0x40005404U)
#define I2C1_DR       (*(volatile uint32_t*)0x40005410U)
#define I2C1_SR1      (*(volatile uint32_t*)0x40005414U)
#define I2C1_SR2      (*(volatile uint32_t*)0x40005418U)
#define I2C1_CCR      (*(volatile uint32_t*)0x4000541CU)
#define I2C1_TRISE    (*(volatile uint32_t*)0x40005420U)

#define MPU_ADDR        0x68U   // адрес датчика
#define WHO_AM_I_REG    0x75U   // номер регистра «кто ты»
#define PWR_MGMT_1_REG  0x6BU   // управление питанием

#define ACCEL_XOUT_H    0x3BU   // X, старший байт
#define ACCEL_XOUT_L    0x3CU   // X, младший байт
#define ACCEL_YOUT_H    0x3DU
#define ACCEL_YOUT_L    0x3EU
#define ACCEL_ZOUT_H    0x3FU
#define ACCEL_ZOUT_L    0x40U

#define I2C_TIMEOUT     100000U // сколько раз проверять флаг, прежде чем сдаться

volatile int16_t  ax = 0;
volatile int16_t  ay = 0;
volatile int16_t  az = 0;
volatile uint32_t i2c_errors = 0;   // сколько раз I2C сбоил

/* ---------- ожидания с тайм-аутом ---------- */

// ждать, пока бит в SR1 станет 1
int i2c_wait(uint32_t bit) {
	uint32_t t = I2C_TIMEOUT;
	while ((I2C1_SR1 & (1U << bit)) == 0) {
		--t;
		if (t == 0) {
			return -1;
		}
	}
	return 0;
}

// НОВОЕ: ждать, пока STOP (CR1 бит 9) сбросится в 0
int i2c_wait_stop(void) {
	uint32_t t = I2C_TIMEOUT;
	while ((I2C1_CR1 & (1U << 9)) != 0) {
		--t;
		if (t == 0) {
			return -1;
		}
	}
	return 0;
}

/* ---------- чтение одного регистра ---------- */
int i2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *out) {
	// 1. START
	I2C1_CR1 |= (1U << 8);
	if (i2c_wait(0) != 0) {        // ждать SB
		return -1;
	}

	// 2. адрес + запись
	I2C1_DR = (dev << 1);
	if (i2c_wait(1) != 0) {        // ждать ADDR
		return -1;
	}
	(void) I2C1_SR1;               // сброс ADDR
	(void) I2C1_SR2;

	// 3. номер регистра
	if (i2c_wait(7) != 0) {        // ждать TxE
		return -1;
	}
	I2C1_DR = reg;
	if (i2c_wait(2) != 0) {        // ждать BTF
		return -1;
	}

	// 4. повторный START
	I2C1_CR1 |= (1U << 8);
	if (i2c_wait(0) != 0) {        // ждать SB
		return -1;
	}

	// 5. адрес + чтение
	I2C1_DR = (dev << 1) | 1U;
	if (i2c_wait(1) != 0) {        // ждать ADDR
		return -1;
	}

	// 6. принять 1 байт
	I2C1_CR1 &= ~(1U << 10);       // ACK = 0 (NACK)
	(void) I2C1_SR1;               // сброс ADDR
	(void) I2C1_SR2;
	I2C1_CR1 |= (1U << 9);         // STOP
	if (i2c_wait(6) != 0) {        // ждать RxNE
		return -1;
	}

	*out = (uint8_t) I2C1_DR;      // забрать байт

	if (i2c_wait_stop() != 0) {    // НОВОЕ: ждать STOP с тайм-аутом
		return -1;
	}
	return 0;                      // успех
}

/* ---------- запись одного регистра ---------- */
int i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val) {
	// 1. START
	I2C1_CR1 |= (1U << 8);
	if (i2c_wait(0) != 0) {        // SB
		return -1;
	}

	// 2. адрес + запись
	I2C1_DR = (dev << 1);
	if (i2c_wait(1) != 0) {        // ADDR
		return -1;
	}
	(void) I2C1_SR1;
	(void) I2C1_SR2;

	// 3. номер регистра
	if (i2c_wait(7) != 0) {        // TxE
		return -1;
	}
	I2C1_DR = reg;

	// 4. значение
	if (i2c_wait(7) != 0) {        // TxE
		return -1;
	}
	I2C1_DR = val;
	if (i2c_wait(2) != 0) {        // BTF: всё дошло
		return -1;
	}

	// 5. STOP
	I2C1_CR1 |= (1U << 9);
	if (i2c_wait_stop() != 0) {    // НОВОЕ: ждать STOP с тайм-аутом
		return -1;
	}
	return 0;                      // успех
}

/* ---------- настройка I2C1 ---------- */
void i2c_init(void) {
	I2C1_CR2 &= ~(0x3FU << 0);
	I2C1_CR2 |=  (16U << 0);       // FREQ = 16 МГц

	I2C1_CCR &= ~(0xFFFU << 0);
	I2C1_CCR |=  (80U << 0);       // 100 кГц

	I2C1_TRISE &= ~(0x3FU << 0);
	I2C1_TRISE |=  (17U << 0);     // TRISE = 17

	I2C1_CR1 |= (1U << 0);         // PE: включить I2C1
}

// НОВОЕ: короткая пауза для ручного «дёргания» SCL
void delay_short(void) {
	for (volatile uint32_t d = 0; d < 100U; d++) {
	}
}

// НОВОЕ: освободить шину, если датчик «держит» SDA
void i2c_bus_clear(void) {
	// PB8, PB9 → обычный выход (open-drain уже включён)
	GPIOB_ODR |= (1U << 8) | (1U << 9);       // отпустить обе линии
	GPIOB_MODER &= ~((3U << 16) | (3U << 18));
	GPIOB_MODER |=  ((1U << 16) | (1U << 18)); // 01 = выход

	// 9 тактов SCL: датчик дотолкает свой байт и отпустит SDA
	for (uint32_t i = 0; i < 9U; i++) {
		GPIOB_ODR &= ~(1U << 8);   // SCL = 0
		delay_short();
		GPIOB_ODR |=  (1U << 8);   // SCL = 1
		delay_short();
	}

	// STOP вручную: SDA 0 → 1, пока SCL = 1
	GPIOB_ODR &= ~(1U << 9);
	delay_short();
	GPIOB_ODR |=  (1U << 9);
	delay_short();

	// вернуть ножки блоку I2C1 (AF)
	GPIOB_MODER &= ~((3U << 16) | (3U << 18));
	GPIOB_MODER |=  ((2U << 16) | (2U << 18));
}

// восстановление после сбоя
void i2c_recover(void) {
	i2c_bus_clear();                 // НОВОЕ: освободить линии
	I2C1_CR1 |=  (1U << 15);         // SWRST = 1: сбросить I2C1
	I2C1_CR1 &= ~(1U << 15);         // SWRST = 0: отпустить сброс
	i2c_init();                      // настройки стёрлись, записать заново
	(void) i2c_write_reg(MPU_ADDR, PWR_MGMT_1_REG, 0x00U); // НОВОЕ: снова разбудить датчик
}

int main(void) {
	// такты
	RCC_AHB1ENR |= (1U << 1);        // GPIOB
	RCC_APB1ENR |= (1U << 21);       // I2C1

	// PB8, PB9 → AF
	GPIOB_MODER &= ~(3U << 16);
	GPIOB_MODER |=  (2U << 16);
	GPIOB_MODER &= ~(3U << 18);
	GPIOB_MODER |=  (2U << 18);

	GPIOB_OTYPER |= (1U << 8);       // PB8 open-drain
	GPIOB_OTYPER |= (1U << 9);       // PB9 open-drain

	GPIOB_PUPDR &= ~(3U << 16);
	GPIOB_PUPDR |=  (1U << 16);      // PB8 pull-up
	GPIOB_PUPDR &= ~(3U << 18);
	GPIOB_PUPDR |=  (1U << 18);      // PB9 pull-up

	GPIOB_AFRH &= ~(0xFU << 0);
	GPIOB_AFRH |=  (4U << 0);        // PB8 = I2C1_SCL
	GPIOB_AFRH &= ~(0xFU << 4);
	GPIOB_AFRH |=  (4U << 4);        // PB9 = I2C1_SDA

	i2c_init();

	uint8_t who = 0;
	int who_status = i2c_read_reg(MPU_ADDR, WHO_AM_I_REG, &who);

	uint8_t pwr_before = 0;
	int pwr_before_status = i2c_read_reg(MPU_ADDR, PWR_MGMT_1_REG, &pwr_before);

	int wake_status = i2c_write_reg(MPU_ADDR, PWR_MGMT_1_REG, 0x00U);   // разбудить

	uint8_t pwr_after = 0;
	int pwr_after_status = i2c_read_reg(MPU_ADDR, PWR_MGMT_1_REG, &pwr_after);

	(void) who;
	(void) who_status;
	(void) pwr_before;
	(void) pwr_before_status;
	(void) wake_status;
	(void) pwr_after;
	(void) pwr_after_status;

	while (1) {
		// ---- X ----
		uint8_t xh = 0;
		uint8_t xl = 0;
		if (i2c_read_reg(MPU_ADDR, ACCEL_XOUT_H, &xh) == 0 &&
		    i2c_read_reg(MPU_ADDR, ACCEL_XOUT_L, &xl) == 0) {
			ax = (int16_t) ((xh << 8) | xl);
		} else {
			i2c_errors++;            // сбой → считаем, ax не трогаем
			i2c_recover();
		}

		// ---- Y ----
		uint8_t yh = 0;
		uint8_t yl = 0;
		if (i2c_read_reg(MPU_ADDR, ACCEL_YOUT_H, &yh) == 0 &&
		    i2c_read_reg(MPU_ADDR, ACCEL_YOUT_L, &yl) == 0) {
			ay = (int16_t) ((yh << 8) | yl);
		} else {
			i2c_errors++;            // сбой → считаем, ay не трогаем
			i2c_recover();
		}

		// ---- Z ----
		uint8_t zh = 0;
		uint8_t zl = 0;
		if (i2c_read_reg(MPU_ADDR, ACCEL_ZOUT_H, &zh) == 0 &&
		    i2c_read_reg(MPU_ADDR, ACCEL_ZOUT_L, &zl) == 0) {
			az = (int16_t) ((zh << 8) | zl);
		} else {
			i2c_errors++;            // сбой → считаем, az не трогаем
			i2c_recover();
		}
	}
}
