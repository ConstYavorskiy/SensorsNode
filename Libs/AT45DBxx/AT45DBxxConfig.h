#ifndef _AT45DBXXCONFIG_H
#define _AT45DBXXCONFIG_H

#include "main.h"

extern SPI_HandleTypeDef hspi1;

#define	_AT45DBXX_SPI		hspi1
#define	_AT45DBXX_CS_GPIO	GPIOA
#define	_AT45DBXX_CS_PIN	GPIO_PIN_4

#endif
