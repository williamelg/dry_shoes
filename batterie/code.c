/*
 * batterie.c
 *
 *  Created on: 13 avr. 2023
 *      Author: willi
 */

#include "batterie.h"

float* ADC_DMA_BAT( uint16_t *adc_dma_buffer, float *table){

	// Print ADC conversion results
	my_printf("| Ch11=%04d | Ch12=%04d | Ch13=%04d | Ch14=%04d\r\n ", adc_dma_buffer[0], adc_dma_buffer[1], adc_dma_buffer[2], adc_dma_buffer[3]);
	for (int i=0;i<4;i++){
		table[i]=(float)adc_dma_buffer[i] / 4096 * 3.3;
	}
	my_printf("| bloc1 = %d\r\n | bloc2 = %d\r\n | bloc3 = %d\r\n | bloc4 = %d\r\n",(int)(double)round((((float)adc_dma_buffer[0] / 4096 * 3.3)/3.3)*100), (int)(double)round((((float)adc_dma_buffer[1] / 4096 * 3.3)/3.3)*100), (int)(double)round((((float)adc_dma_buffer[2] / 4096 * 3.3)/3.3)*100), (int)(double)round((((float)adc_dma_buffer[3] / 4096 * 3.3)/3.3)*100));
	return table;
}

int BLOC_CHANGE( float *voltage){
	int probleme = 0;
	float som = voltage[0] + voltage[1] + voltage [2] + voltage[3] ;
	float moy = som / 4;
	for (int i = 0 ; i<4; i++ ) {
		if (voltage[i] <= 0.01 ) {
			my_printf (" Problème numero 1 bloc I = %d",i+1);
			probleme = 1;
			return probleme;
		}
		if (voltage [i] < moy - 0.2 || voltage [i] > moy + 0.2 ) {
			my_printf (" Problème numero 2 bloc I = %d",i+1);
			probleme = 1;
		}
	}

	return probleme;
}

