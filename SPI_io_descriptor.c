/*
 * SPI_io_descriptor.c
 *
 * Created: 26/09/2022 23:25:43
 *  Author: Codrutza Dragu
 */ 


#include "SPI_io_descriptor.h"

void create_spi_descriptor(){
	// creating SPI instance
	spi_m_sync_get_io_descriptor(&SPI_0, &io);
	spi_m_sync_enable(&SPI_0);

}