#ifndef INCLUDE_MPU6050_IRQ_H
#define INCLUDE_MPU6050_IRQ_H

int mpu6050_irq_init(void);
void mpu6050_irq_free(void);

extern unsigned int mpu6050_irq_cnt; // how many time IRQ happened


#endif
