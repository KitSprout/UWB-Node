UWB-Node
========
[![GitHub version](https://img.shields.io/badge/version-v1.6-brightgreen.svg)](https://github.com/KitSprout/UWB-Node)
[![GitHub old version](https://img.shields.io/badge/old%20version-%20v1.2-green.svg)](https://github.com/KitSprout/UWB-Node/releases/tag/v1.2)
[![GitHub license](https://img.shields.io/badge/license-%20MIT%20%2F%20CC%20BY--SA%204.0-blue.svg)](https://github.com/KitSprout/UWB-Node/blob/master/LICENSE)
[![GitHub pcb library](https://img.shields.io/badge/pcb%20library-%20v3.2-yellow.svg)](https://github.com/KitSprout/AltiumDesigner_PcbLibrary/releases/tag/v3.2)

UWB Node 是一款體積小的 UWB 室內定位開發模組，採用 STM32F411CE 芯片以及 DWM1000 模組，板上有九軸慣性感測器、氣壓計，並規劃電池電壓檢測功能，預留電源接口與開關，方便獨立供電安裝，未來預計與 [UWB Adapter](https://github.com/KitSprout/UWB-Adapter) 配合，以完善室內定位之開發。

Hardware
========
* 控制器 : [STM32F411CE](http://www.st.com/en/microcontrollers/stm32f411ce.html) 48Pin 100MHz DSP FPU
* 感測器 : [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) + [LPS22HB](http://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps22hb.html)
* UWB : DecaWave [DWM1000](https://www.decawave.com/products/dwm1000-module)
* 設計軟體 [Altium Designer 17](http://www.altium.com/altium-designer/overview) ( PcbLib use AD [PcbLib v3.2](https://github.com/KitSprout/AltiumDesigner_PcbLibrary/releases/tag/v3.2) )

View
========
<img src="https://lh3.googleusercontent.com/mCzzQTiW9ZMtXypp2jceR8vzO2CTvdLcJt4vv0sfbgvvQVbfNUZv35zs0yerX58ldhprV0ztE4bfRyO5jWjDpGF3qjqo8jkT5JgLvTKUlz2kQx3TYhIt5vg9BREE7fCKhFhAOnarzA=w1449-h951-no"/>
<br />
