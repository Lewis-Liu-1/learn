## git hub connect hangs
https://stackoverflow.com/questions/8750930/git-clone-hangs-forever-on-github
## nano ~/.ssh/config

## 
Host github.com
  Hostname ssh.github.com
  Port 443
 
# Knowledge:
**Push-pull** outputs can pull a pin to either **1** or **0** while **open-drain** outputs can only pull the pin to **0**

Open-drain outputs are useful if you have multiple devices set to **output** on the same wire, because it prevents different devices from trying to pull the signal in two different directions at once and damaging each other. But we want push-pull, for simply turning an LED on or off


## button and led
https://vivonomicon.com/2018/04/22/bare-metal-stm32-programming-part-3-leds-and-buttons/

# Knowledge:
The F0 line of chips boot to an 8MHz ‘HSI’ (‘High-Speed Internal’) oscillator, but the L0 line is optimized for low power usage and boots to a 2.1MHz ‘MSI’ (‘Multi-Speed Internal’) oscillator instead

# interrupt
https://vivonomicon.com/2018/04/28/bare-metal-stm32-programming-part-4-intro-to-hardware-interrupts/

# NVIC
“Nested Vector Interrupt Controller”
# EXTI
‘EXTI’ stands for ‘EXTended Interrupt controller’

