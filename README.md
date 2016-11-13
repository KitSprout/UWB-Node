UWB-Node
========
[![GitHub version](https://img.shields.io/badge/version-v1.2-brightgreen.svg)](https://github.com/KitSprout/UWB-Node)
[![GitHub old version](https://img.shields.io/badge/old%20version-%20v1.0-green.svg)](https://github.com/KitSprout/UWB-Node/releases/tag/v1.0)
[![GitHub license](https://img.shields.io/badge/license-%20MIT%20%2F%20CC%20BY--SA%204.0-blue.svg)](https://github.com/KitSprout/UWB-Node/blob/master/LICENSE)
[![GitHub pcb library](https://img.shields.io/badge/pcb%20library-%20v2.12-yellow.svg)](https://github.com/KitSprout/AltiumDesigner_PcbLibrary/releases/tag/v2.12)

UWB Node 是一款 UWB 室內定位開發模組，採用 STM32F411CE 芯片以及 DWM1000 模組，板上有九軸慣性感測器、氣壓計，並規劃電池電壓檢測功能，預留部分擴充腳位，藉以連接藍牙、WIFI 模組以及 OLED 螢幕。

Hardware
========
* 控制器　 : [STM32F411CE](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1877/PF260148) 48Pin 100MHz DSP FPU
* 感測器　 : [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) + [LPS22HB](http://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps22hb.html)
* UWB　　 : DecaWave [DWM1000](http://www.decawave.com/products/dwm1000-module)
* 設計軟體 [Altium Designer 16](http://www.altium.com/en/products/altium-designer) ( PcbLib use AD [PcbLib v2.12](https://github.com/KitSprout/AltiumDesigner_PcbLibrary/releases/tag/v2.12) )

<img src="https://lh3.googleusercontent.com/pQ7BopdjN-gVYOG8wm8I_R0amw77cDrUzewzARG8QNo_p4aVc2wkJQYsrMeRPm0QME2RE9v7uvN7XYaHzIq5KpJ5QTPE6zgiab9SlENyY6HeMo6gvfN_VN8rlICWM1hCoh-hBtQDuU4a1mNY2GB_48JGpFoviRlp3c4zyV5JxC1_cg5ltCCD-C71ARulXYUSfAZPmx2rK98cbVpwSIGSJugkxpqMxMQnjBkcSp-C2XwHUq6wZAo76RbzJnHx9ZTqQblGVyCjoXMIdWlc9CcuwS9BBVyAbBPJ16gmKn-HdPZvG2w13C6g0TO30680J1QOouhlGbqHKRIPfvnjzG736Niny6bORYA0CfWV2fqMH_KSNbM6Rc1c2OB85N8buGm1uvxfOMDr8E6fC796tKJsUtV0RZTZXKW9SRj-JbhN2GwggpjN0gKts2xCjquweDuIiomtDytirDATeg41_eJorwWWdLU9w9m-j47RyrfpvJE9fbBBkcJkNu9IssG5qUq0GxmDJK_rl0d_rtf_l3eQF4_6UBxoyjp21zvVTEWZQXEkMaeQhgiliP0eSdRir3dXMkxXbN8UW3oZN_e6FRJ-j-Rg8puhYWdpPvTASk2LtPLUJX1n=w381-h276-no"/>

View
========
<img src="https://lh3.googleusercontent.com/253C_XjY5QrzkHj9jU5nW3GMJcE52MzS_0MwQulyRT63yu781R0tdCL0rWfQaiV8sXaUT0I6d5OgwsMB9My000SVux2-vEmrsWV5tgWydhWRkmgSoEXkY0SYKR8qYvExFk4hTj0quaUUnFTtCpERlNZoz_tkWPgwP-zAzX3fqZv7mIMz6UQO7LlWbBCS3IfKJa1B9ntHs0Xvi_hGc-ISfVA7eelV2LgXCVztlQ-I2cWI9V54Wn4Nr56NP7YbvvyeUh3wdHxPXEpZX_sF7_--D-8PmWsNjrD0HgDEFhLvf1pM7PtLoxNElu-I5Mq2TX7L2utC7p6Zr096hl5CNhfXCMA95lqz-B-7O9F_ekBXBKo4Tz84rnwTeXk5JCLiVZILCrG4kxx7ts1SiblQ59_py_vzDciMiC5gQ9HhB-aNfGUzrEjjMQadlMLjEvrIQU87K7pShsMYe8a7JnPwKLAalBVVc91Ykyeq_pCtY7Gn6bAXCMt4CKCD48XaEpkTCK1qwmUXRez3NHHnjRyRVfuYJVcC-zmqZJQj6SmAyMRvTNQhdhgHatE6LjM_y5-aO_zihuQpMivVdncazjrAmNaBW2NIw8TmAxLCkOHt_z8PktN4p6XE=w1027-h770-no"/>

<br />
[more photo...](https://goo.gl/photos/W93huj9bQzaHpJsv7)

Schematic
========
<img src="https://lh3.googleusercontent.com/e1kFH0VYdga2kUgn1RYbRyb9w9bVBomTIqkBuT80-z-KbchTS7NkgXQrkw4rjkJdK-hA0vE8ZF4Fb2LzJ5zKidpfvaGNhR4ZDVAFzh4sCjOxRpTShdKrjCMupd21vXxKEQGyOrQ9JZCX0thS1MsKS6_Hb__LlaSjvffHqkmsvx8Os1Qoe0TiNFQTQphq5nilW9-_wcKPnHuqEFAer18CPccYBHzlv5VNDcHCGAUqQR7ojt5UZn1KgYEOAIBWZf31n6wXaeD7vJQI8btL9d16YDID9CwC-ujo90QFheJMCt3_gLY40534U5QSv1y1PP8hETZIuWd_Te4qc17PqbgnKJf1muKMrXpQjToZvNTWytkQIwCyCsBbhEaUJUfk3g13OtQ6GV4xbc7EPZeqNi3OzrVdzwt4SquvpXX4eeptnb1euZO75GbXTThOmvAThX61HHdZ2Q_vLYtFD5pffRLHHbxOCOVD6cWu1ILH1SqQ6VcPv1WBFuN2i6fMnfgQhGE5TpWXE7sGZUKCClgHEyFdZtzKOAZLawXVCH_cH_roktU-DYgg1zIMFU6KdVhEuuYDZcz5PdspoSNT4Qg8-bSsuOaAR9K06fsweQqc0hgdOPNsCnlV=w990-h770-no"/>
