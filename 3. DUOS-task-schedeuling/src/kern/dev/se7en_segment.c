/* 
* Copyright information is at the end of this file.
*/

#include "./include/se7en_segment.h"
#include <stm32_peps.h>


uint8_t top = 15;
uint8_t top_left = 14;
uint8_t top_right = 13;

uint8_t middle = 6;

uint8_t bottom = 1;
uint8_t bottom_left = 2;
uint8_t bottom_right = 12;


/**
* Take an array of 8 bit integer, of size 9 and set the corresponding pins of the LEDs as output
* [0] [1] [2]
* [3] [4] [5]
* [6] [7] [8]
* @param led_array: an array of 8 bit integer
**/
void se7en_seg_init()
{
    RCC->AHB1ENR |= (1<<1); //for portB
    GPIOB->MODER |= (1<<(top*2));
    GPIOB->MODER |= (1<<(top_left*2));
    GPIOB->MODER |= (1<<(top_right*2));
    GPIOB->MODER |= (1<<(middle*2));
    GPIOB->MODER |= (1<<(bottom*2));
    GPIOB->MODER |= (1<<(bottom_left*2));
    GPIOB->MODER |= (1<<(bottom_right*2));
}
    


void se7en_seg_print(uint8_t num){
    switch (num)
    {
    case 0:
        GPIOB->BSRR |= (1<<top);
        GPIOB->BSRR |= (1<<top_left);
        GPIOB->BSRR |= (1<<top_right);

        GPIOB->BSRR |= (1<<middle) << 16;

        GPIOB->BSRR |= (1<<bottom);
        GPIOB->BSRR |= (1<<bottom_left);
        GPIOB->BSRR |= (1<<bottom_right);
        break;
    
    case 1:
        GPIOB->BSRR |= (1<<top)<< 16;
        GPIOB->BSRR |= (1<<top_left)<< 16;
        GPIOB->BSRR |= (1<<top_right);

        GPIOB->BSRR |= (1<<middle) << 16;

        GPIOB->BSRR |= (1<<bottom)<< 16;
        GPIOB->BSRR |= (1<<bottom_left)<< 16;
        GPIOB->BSRR |= (1<<bottom_right);
        break;

    case 2:
        GPIOB->BSRR |= (1<<top);
        GPIOB->BSRR |= (1<<top_left) << 16;
        GPIOB->BSRR |= (1<<top_right);

        GPIOB->BSRR |= (1<<middle);

        GPIOB->BSRR |= (1<<bottom);
        GPIOB->BSRR |= (1<<bottom_left);
        GPIOB->BSRR |= (1<<bottom_right) << 16;
        break;
    
    case 3:
        GPIOB->BSRR |= (1<<top);
        GPIOB->BSRR |= (1<<top_left) << 16;
        GPIOB->BSRR |= (1<<top_right);

        GPIOB->BSRR |= (1<<middle);

        GPIOB->BSRR |= (1<<bottom);
        GPIOB->BSRR |= (1<<bottom_left) << 16;
        GPIOB->BSRR |= (1<<bottom_right) ;
        break;
    
    case 4:
        GPIOB->BSRR |= (1<<top) << 16;
        GPIOB->BSRR |= (1<<top_left) ;
        GPIOB->BSRR |= (1<<top_right);

        GPIOB->BSRR |= (1<<middle);

        GPIOB->BSRR |= (1<<bottom) << 16;
        GPIOB->BSRR |= (1<<bottom_left) << 16;
        GPIOB->BSRR |= (1<<bottom_right) ;
        break;
    
    case 5:
        GPIOB->BSRR |= (1<<top) ;
        GPIOB->BSRR |= (1<<top_left) ;
        GPIOB->BSRR |= (1<<top_right) << 16;

        GPIOB->BSRR |= (1<<middle);

        GPIOB->BSRR |= (1<<bottom);
        GPIOB->BSRR |= (1<<bottom_left) << 16;
        GPIOB->BSRR |= (1<<bottom_right) ;
        break;

    case 6:
        GPIOB->BSRR |= (1<<top) ;
        GPIOB->BSRR |= (1<<top_left) ;
        GPIOB->BSRR |= (1<<top_right) << 16;

        GPIOB->BSRR |= (1<<middle);

        GPIOB->BSRR |= (1<<bottom);
        GPIOB->BSRR |= (1<<bottom_left) ;
        GPIOB->BSRR |= (1<<bottom_right) ;
        break;
    
    case 7:
        GPIOB->BSRR |= (1<<top) ;
        GPIOB->BSRR |= (1<<top_left) << 16;
        GPIOB->BSRR |= (1<<top_right) ;

        GPIOB->BSRR |= (1<<middle) << 16;

        GPIOB->BSRR |= (1<<bottom) << 16;
        GPIOB->BSRR |= (1<<bottom_left) << 16;
        GPIOB->BSRR |= (1<<bottom_right) ;
        break;
    
    case 8:
        GPIOB->BSRR |= (1<<top) ;
        GPIOB->BSRR |= (1<<top_left) ;
        GPIOB->BSRR |= (1<<top_right) ;

        GPIOB->BSRR |= (1<<middle) ;

        GPIOB->BSRR |= (1<<bottom) ;
        GPIOB->BSRR |= (1<<bottom_left) ;
        GPIOB->BSRR |= (1<<bottom_right) ;
        break;
    
    case 9:
        GPIOB->BSRR |= (1<<top) ;
        GPIOB->BSRR |= (1<<top_left) ;
        GPIOB->BSRR |= (1<<top_right) ;

        GPIOB->BSRR |= (1<<middle) ;

        GPIOB->BSRR |= (1<<bottom) ;
        GPIOB->BSRR |= (1<<bottom_left) << 16;
        GPIOB->BSRR |= (1<<bottom_right) ;
        break;
    default:
        break;
    }
}



















/*
 * Copyright (c) 2022 
 * Computer Science and Engineering, University of Dhaka
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */