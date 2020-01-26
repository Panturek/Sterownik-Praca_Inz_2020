# Sterownik-Praca_Inz_2020
Sterownik robota na stm32f4_disco na zephyrze

Działać nie ma prawa od ręki. Trza CANa ogarnąć na płytce po stronie zephyra. Myśmy to zrobili tak:

1) PRIMO
~/zephyrproject/zephyr/boards/arm/stm32f4_disco/pinmux.c 
dodać/zmienić piny do CANa:

#ifdef CONFIG_CAN1_1
	{STM32_PIN_PB8, STM32F4_PINMUX_FUNC_PB8_CAN_RX},
	{STM32_PIN_PB9, STM32F4_PINMUX_FUNC_PB9_CAN_TX},
#endif	/* CONFIG_CAN_1*/

ostatecznie na płytce jest RX-PA11 a TX-PA12
jakby kto robił swoją płytkę to niech lepiej wybierze inne piny, bo te są od usb otg.
od CANa jest kilka a od otg tylko te

2) SECONDO
/home/panturek/zephyrproject/zephyr/dts/arm/st/f4/stm32f405.dtsi
tam pod can1 zmienić status na okay: 

can1: can@40006400 {
      .
      .
      status = "okay";
			label = "CAN_1";
			bus-speed = <125000>;
			.
      .
};

prędkość też można ustawiać, w bitach na sekundę jest.
w wersji PRO jakiś overlay powinien do tego być w folderze z projektem.
u jest zastosowane podejście żółtodziobiane, bo też działa.
