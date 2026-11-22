#include <stdint.h>

#define RCC_AHB1ENR   (*(volatile uint32_t*)0x40023830U)
#define RCC_APB1ENR   (*(volatile uint32_t*)0x40023840U)

#define GPIOB_MODER (*(volatile uint32_t*)0x40020400U)
#define GPIOB_OTYPER (*(volatile uint32_t*)0x40020404U)

#define GPIOB_PUPDR (*(volatile uint32_t*)0x4002040CU)
#define GPIOB_AFRH (*(volatile uint32_t*)0x40020424U)

#define I2C1_CR2 (*(volatile uint32_t*)0x40005404U)
#define I2C1_CCR   (*(volatile uint32_t*)0x4000541CU)
#define I2C1_TRISE (*(volatile uint32_t*)0x40005420U)

#define I2C1_CR1 (*(volatile uint32_t*)0x40005400U)
#define I2C1_SR1 (*(volatile uint32_t*)0x40005414U)

#define I2C1_DR  (*(volatile uint32_t*)0x40005410U)
#define I2C1_SR2 (*(volatile uint32_t*)0x40005418U)

#define MPU_ADDR      0x68U   // адрес датчика
#define WHO_AM_I_REG  0x75U   // номер регистра «кто ты»
#define PWR_MGMT_1_REG  0x6BU   // управление питанием

uint8_t i2c_read_reg(uint8_t dev, uint8_t reg) {
	// 1. START
	I2C1_CR1 |= (1U << 8);                 // крикнуть START
	while ((I2C1_SR1 & (1U << 0)) == 0) {
	}   // ждать флаг SB

	// 2. адрес + запись
	I2C1_DR = (dev << 1);                   // отправить адрес
	while ((I2C1_SR1 & (1U << 1)) == 0) {
	}    // ждать ADDR (датчик ответил)
	(void) I2C1_SR1;                          // сброс ADDR: читаем SR1...
	(void) I2C1_SR2;

	// 3. номер регистра
	while ((I2C1_SR1 & (1U << 7)) == 0) {
	}   // ждать TxE
	I2C1_DR = reg;                          // отправить номер регистра
	while ((I2C1_SR1 & (1U << 2)) == 0) {
	}   // ждать BTF

	// 4. повторный START
	I2C1_CR1 |= (1U << 8);                 // START
	while ((I2C1_SR1 & (1U << 0)) == 0) {
	}   // ждать SB

	// 5. адрес + чтение
	I2C1_DR = (dev << 1) | 1U;              // адрес + бит чтения
	while ((I2C1_SR1 & (1U << 1)) == 0) {
	}   // ждать ADDR

	// 6. принять 1 байт
	I2C1_CR1 &= ~(1U << 10);                // ACK = 0 (NACK)
	(void) I2C1_SR1;                         // сброс ADDR
	(void) I2C1_SR2;
	I2C1_CR1 |= (1U << 9);                 // STOP
	while ((I2C1_SR1 & (1U << 6)) == 0) {
	}   // ждать RxNE

	uint8_t data = (uint8_t) I2C1_DR;        // забрать байт
	while((I2C1_CR1 & (1U << 9)) != 0){}     // ждать, пока STOP отправится
	return data;                             // вернуть байт
}


void i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val)
{
    // 1. START
    I2C1_CR1 |= (1U << 8);
    while((I2C1_SR1 & (1U << 0)) == 0){}   // SB

    // 2. адрес + запись
    I2C1_DR = (dev << 1);
    while((I2C1_SR1 & (1U << 1)) == 0){}   // ADDR
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    // 3. номер регистра
    while((I2C1_SR1 & (1U << 7)) == 0){}   // TxE
    I2C1_DR = reg;

    // 4. значение
    while((I2C1_SR1 & (1U << 7)) == 0){}   // TxE
    I2C1_DR = val;
    while((I2C1_SR1 & (1U << 2)) == 0){}   // BTF: всё дошло

    // 5. STOP
    I2C1_CR1 |= (1U << 9);

    while((I2C1_CR1 & (1U << 9)) != 0){}   // ждать, пока STOP отправится
}


int main(void) {

	RCC_AHB1ENR |= (1U << 1);
	RCC_APB1ENR |= (1U << 21);

	GPIOB_MODER &= ~(3U << 16);
	GPIOB_MODER |= (2U << 16);

	GPIOB_MODER &= ~(3U << 18);
	GPIOB_MODER |= (2U << 18);

	GPIOB_OTYPER |= (1U << 8);   // PB8 open-drain
	GPIOB_OTYPER |= (1U << 9);   // PB9 open-drain

	GPIOB_PUPDR &= ~(3U << 16);
	GPIOB_PUPDR |= (1U << 16);   // PB8 pull-up

	GPIOB_PUPDR &= ~(3U << 18);
	GPIOB_PUPDR |= (1U << 18);   // PB9 pull-up

	GPIOB_AFRH &= ~(0xFU << 0);
	GPIOB_AFRH |= (4U << 0);     // PB8 = I2C1_SCL

	GPIOB_AFRH &= ~(0xFU << 4);
	GPIOB_AFRH |= (4U << 4);     // PB9 = I2C1_SDA

	I2C1_CR2 &= ~(0x3FU << 0);
	I2C1_CR2 |= (16U << 0);    // FREQ = ... МГц

	I2C1_CCR &= ~(0xFFFU << 0);
	I2C1_CCR |= (80U << 0);       // 100 кГц

	I2C1_TRISE &= ~(0x3FU << 0);
	I2C1_TRISE |= (17U << 0);     // TRISE = ?

	I2C1_CR1 |= (1U << 0);

	volatile uint8_t who = i2c_read_reg(MPU_ADDR, WHO_AM_I_REG);
	(void)who;

	volatile uint8_t pwr_before = i2c_read_reg(MPU_ADDR, PWR_MGMT_1_REG);
	i2c_write_reg(MPU_ADDR, PWR_MGMT_1_REG, 0x00U);   // разбудить
	volatile uint8_t pwr_after  = i2c_read_reg(MPU_ADDR, PWR_MGMT_1_REG);
	(void)pwr_before;
	(void)pwr_after;

	while (1) {
	}
}
