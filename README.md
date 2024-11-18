#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>



#include "ff.h"
#include "diskio.h"

#ifndef AUDIO_SEQUENCER_H
#define AUDIO_SEQUENCER_H

// #define SAMPLE_RATE 44000
#define AUDIOBUFFER 512 // Samples
#define HEADER_SIZE 44 // Bytes
#define AUDIOMAX 8

typedef struct audioTrack{
    char * fileName;
    uint16_t data[AUDIOBUFFER];
    bool is_pressed;
} audioTrack;


void nano_wait(unsigned int);
void internal_clock();

void preLoadAudios(audioTrack*);
void mix_audio(audioTrack *, uint16_t*);
void init_tim6(void);
void setup_dac(void);
void setup_dma(uint16_t*);
void enable_dma(void);
void init_sdcard_spi_or_init_spi2(void);
void disable_sdcard(void);
void enable_sdcard(void);
void sdcard_high_speed(void);
void init_tim15(void);
void TIM6_DAC_IRQHandler(void);
// void setup_adc(void);




// void nano_wait(int);




int main(void){
    FATFS fs;
    FRESULT fres;

    internal_clock();
    init_sdcard_spi_or_init_spi2();
    sdcard_high_speed();

    fres = f_mount(&fs, "", 1);
    if (fres != FR_OK) {
        // Handle error
        while(1);
    }
    

    char * trackNames[AUDIOMAX] = {
        "BClassic1_SNARE3_snare_BANDLAB.wav",
        "BobbyKritical_Kick12_kick_BANDLAB.wav",
        "BobbyKritical_Snare9_snare_BANDLAB.wav",
        "Clap-3.wav",
        "Headed_Tambo_Hit_percussion_BANDLAB.wav",
        "Kawai-K1r-Closed-Hi-Hat.wav",
        "minecraft-villager-sound-effect.wav",
        "taco-bell-bong-sfx.wav"
    };
    // Initializes audio tracks array
    audioTrack * audioTracks = (audioTrack*)malloc(AUDIOMAX*sizeof(audioTrack));
    if (audioTracks == NULL) {
        // Handle memory allocation failure
        while(1); // Or implement an appropriate error handling mechanism
    }
    for(int i = 0; i < AUDIOMAX; i++){
        audioTracks[i].fileName = trackNames[i]; 
        memset(audioTracks[i].data, 0, sizeof(audioTracks[i].data));
        audioTracks[i].is_pressed = false;
    }

    audioTracks[7].is_pressed = true;
    
    preLoadAudios(audioTracks);

    uint16_t mixed_audio[AUDIOBUFFER];
    mix_audio(audioTracks, mixed_audio);

    // create mixed wave file and play 8 in each step of 8 step loop
    // for(int i = 0; i < AUDIOMAX; i++){
    //     mix_audio(audioTracks, buffer[i]);
    // }
    for (int i = 0; i < AUDIOBUFFER; i++) {
    mixed_audio[i] = (mixed_audio[i] * 4095) / UINT16_MAX;
}


    setup_dac();
    init_tim6();

    
    setup_dma(mixed_audio);
    enable_dma();
    init_tim15();


    
    

    // Use below for implementing rest of sequencer code.
    // // Main loop
    // while(1) {
    //     nano_wait(1000000000);
    // }

    // free(audioTracks);
    return 0;
}


// Needs error checking and buffer overflow check
void preLoadAudios(audioTrack * audioTracks){
    FIL file;
    FRESULT fres;
    UINT br;

    for(int i = 0; i < AUDIOMAX; i++){
        audioTrack *track = &audioTracks[i];

        fres = f_open(&file, track->fileName, FA_READ); // not sure if r or rb(binary mode)
        if (fres != FR_OK) {
            // Handle file open error
            continue;  // Skip to next file
        }
        // SKIP 44 bytes(header)
        f_lseek(&file, HEADER_SIZE);


        // Change: Add check for file size
        FSIZE_t file_size = f_size(&file) - HEADER_SIZE;
        if (file_size > AUDIOBUFFER * sizeof(uint16_t)) {
            file_size = AUDIOBUFFER * sizeof(uint16_t);  // Limit to buffer size
        }
        
        // Get data
        fres = f_read(&file, track->data, file_size, &br);
        if (fres != FR_OK || br != file_size) {
            // Handle error or end of file
        }

        // If file is smaller than buffer, zero-pad the rest
        if (file_size < AUDIOBUFFER * sizeof(uint16_t)) {
            memset((uint8_t*)track->data + file_size, 0, AUDIOBUFFER * sizeof(uint16_t) - file_size);
        }


        f_close(&file);
    }
}


// Needs error checking and buffer overflow check
void mix_audio(audioTrack * audioTracks, uint16_t * buffer){
    float temp;
    memset(buffer, 0, AUDIOBUFFER * sizeof(uint16_t));
    
    int active_tracks = 0;
    for(int i = 0; i < AUDIOMAX; i++){
        if(audioTracks[i].is_pressed){
            active_tracks++;
        }
    }

    if (active_tracks == 0) return;

    float scale = 1.0f / active_tracks;

    for(int i = 0; i < AUDIOMAX; i++){
        audioTrack track = audioTracks[i];

        if(track.is_pressed){
            for(int j = 0; j < AUDIOBUFFER; j++){
                temp = buffer[j] + (track.data[j] * scale);
                buffer[j] = (temp > UINT16_MAX) ? UINT16_MAX : 
                            (temp < 0) ? 0 : (uint16_t)temp;
            }
        }
    }
}

// uint32_t volume = 2048;

// void setup_adc(void) {
//     RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

//     GPIOA->MODER &= ~(0x3 << 2);
//     GPIOA->MODER |= (0x3 << 2);

//     RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

//     RCC->CR2 |= RCC_CR2_HSI14ON;
//     while (!(RCC->CR2 & RCC_CR2_HSI14RDY));

//     ADC1->CR |= ADC_CR_ADEN; 
//     while (!(ADC1->ISR & ADC_ISR_ADRDY));
    
    
//     ADC1->CHSELR |= (1 << 1);
//     while (!(ADC1->ISR & ADC_ISR_ADRDY)) ;


// }



void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

    TIM6->PSC =  2 - 1; // This is supposed to be the wav file sample rate, gets close, but exactly is impossible.
    TIM6->ARR = 544 - 1;

    TIM15->DIER |= TIM_DIER_UDE;

    TIM15->DIER &= ~TIM_DIER_UIE;

    TIM15->CR1 |= TIM_CR1_CEN;
}





// init_tim6()
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM15->PSC =  2 - 1; // This is supposed to be the wav file sample rate, gets close, but exactly is impossible.
    TIM15->ARR = 544 - 1; //CHANGED Sahil

    TIM6->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= 1 << TIM6_DAC_IRQn;
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
}

// (DAC configuration)
void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    
    GPIOA->MODER &= ~0x00000300; 
    GPIOA->MODER |= 0x00000300;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    DAC->CR &= ~DAC_CR_TSEL1;
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_EN1;
    
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF; // Clear interrupt flag
        TIM6->DIER &= ~TIM_DIER_UIE;
    }
}


// (DMA configuration)
void setup_dma(uint16_t * audio) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CPAR = (uint16_t)(&(DAC->DHR12R1)); 
    for (int i = 0; i < AUDIOBUFFER; i++) {
    audio[i] += 2048; // Adjust each sample
}
    DMA1_Channel5->CMAR = (uint16_t)(audio);    
    DMA1_Channel5->CNDTR = AUDIOBUFFER;
    DMA1_Channel5->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_CIRC;


}







// Enables DMA and DAC
void enable_dma(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}


void init_sdcard_spi_or_init_spi2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOB->MODER &= ~(0x3 << 20);
    GPIOB->MODER |= (0x2 << 20);

    GPIOC->MODER &= ~(0xF << 4);
    GPIOC->MODER |= (0xA << 4);

    GPIOB->AFR[1] &= ~(0xF << (8));
    GPIOB->AFR[1] |= (0x5 << (8));


    GPIOC->AFR[0] &= ~(0xFF << (8));
    GPIOC->AFR[0] |= (0x11 << (8));

    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 |= 7 << SPI_CR1_BR_Pos;
    SPI2->CR1 |= SPI_CR1_MSTR;
    SPI2->CR2 = (15 << SPI_CR2_DS_Pos); // MIGHT NEED TO BE EITHER 8 (7) or 16 (15)
    SPI2->CR2 |= SPI_CR2_FRXTH;

    SPI2->CR1 |= SPI_CR1_SSM;
    SPI2->CR1 |= SPI_CR1_SSI;

    SPI2->CR1 |= SPI_CR1_SPE;

    GPIOC->MODER &= ~(0x3 << 22);
    GPIOC -> MODER |= (0x1 << 22);


    disable_sdcard();
}
void enable_sdcard(void) {
    // Assuming CS pin is connected to PB12
    GPIOC->BSRR = (1 << 27);  // Set PBC11 low

}

void disable_sdcard(void) {
    // Assuming CS pin is connected to PB12
    GPIOC->BSRR = (1 << 10);  // Set PC11 high
}


void sdcard_high_speed(void) {
    SPI2->CR1 &= ~SPI_CR1_SPE;  // Disable SPI2
    SPI2->CR1 &= ~(SPI_CR1_BR);  // Clear BR bits
    SPI2->CR1 |= (1 << 3);  // Set BR to achieve ~12MHz (adjust if needed)
    SPI2->CR1 |= SPI_CR1_SPE;  // Re-enable SPI2
}



#endif 

