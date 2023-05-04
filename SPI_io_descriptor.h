/*
 * SPI_io_descriptor.h
 *
 * Created: 26/09/2022 23:14:23
 *  Author: Codrutza Dragu and Dimitar Ruzhev 
 */ 


#ifndef SPI_IO_DESCRIPTOR_H_
#define SPI_IO_DESCRIPTOR_H_

#include "hal_io.h"
#include "hal_spi_m_sync.h"
#include "hal_delay.h"

struct io_descriptor *io;
struct spi_m_sync_descriptor SPI_0;


void create_spi_descriptor(); // create an SPI instance


#endif /* SPI_IO_DESCRIPTOR_H_ */