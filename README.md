## P600EncoderEmulation
Code is written for STM32F407VG.
## Configuration

### LEDs
- Green - **PD12**
- Orange - **PD13**
- Red - **PD14**
- Blue - **PD15**

### ADC
Used ADC1 to measure volatge on IN1 and IN2. Beetwen measurement is a little delay (~14us). Input frequency is 84MHz and is delivered by PCLK2. Prescaller divide input frequency 8 times, so frequency for ADC is 10.5MHz. 
- ADC1_IN1 - **PA1** - connect positive voltage of motor (black wire)
- ADC1_IN2 - **PA2** - connect negative voltage of motor (red wire)

Sampling time is 144 cycles for both channels. It gives sampling rate 72 916Hz
DMA 2 Stream 0 transfer data from ADC to memory in circular mode and half-word. 

### UART
Baudrate 115200, 8bits, 1 stop, parirty 0.
- TX - **PD5**
- RX - **PD6** 	

### TIM7
Used for generate A/B encoder signal. TIM7 is clocked by APB1 bus, so input frequency is 84MHz. Prescaler is set to 7 its give 10 500 000Hz. ARR register could be from 1 to 65535. Maximum encoder frequency should be 40kHz at ARR = 262.

Pinout:
- Signal A - **PD12**
- Signal B - **PD13**

### TIM6
Used to check motor input volatage on ADC1 and correct encoder output frequency. Loop frequency is 16kHz.
PSC 1311 and ARR 3 gives 16 006Hz

### TIM3
Taktowany z lini APB1 o częstotliwości 84MHz. Częstotliwość TIM określa się wzorem: FREQ = TIM_CLK/(ARR+1)(PSC+1)(CKD+1)
Jeśli chcemy uzyskać od 2Hz do 40kHz na wyjściu:

Rememeber to set MMS Master mode selection
TIMx_CR2->MMS = 100: Compare - OC1REF signal is used as trigger output (TRGO)

Dla 40kHz
40 000 = 84 000 000 / ((1049 + 1)(PSC + 1)(1))
40 000 = 84 000 000 / 1050(PSC+1)
42 000 000 = 84 000 000/(PSC + 1)
PSC = (84 000 000 / 42 000 000) - 1
PSC = 2 - 1
PSC = 1

Dla 2Hz:
2 = 84 000 000 / ((1049 + 1)(PSC + 1)(1))
2 = 84 000 000 / 1050(PSC+1)
2100 = 84 000 000/(PSC + 1)
PSC = (84 000 000 / 2100) - 1
PSC = 40 000 - 1
PSC = 39 999

Dla 50Hz:
PSC = 1599
Interconnection (s. 632): 
TIM3_TRGO -> TIM1 (ITR2)

### TIM4
Generate PWM signal (STEP)
TIM4_TRGO -> TIM8 (ITR2)

Rememeber to set MMS Master mode selection
TIMx_CR2->MMS = 100: Compare - OC1REF signal is used as trigger output (TRGO

TIM1 and TIM8 are 16bit timer so its need software increase number of counts use interrupt when overflow