UWB-Node
========
[![GitHub version](https://img.shields.io/badge/version-v1.0-brightgreen.svg)](https://github.com/KitSprout/UWB-Node)
[![GitHub license](https://img.shields.io/badge/license-%20MIT%20%2F%20CC%20BY--SA%204.0-blue.svg)](https://github.com/KitSprout/UWB-Adapter/blob/master/LICENSE)
[![GitHub pcb library](https://img.shields.io/badge/pcb%20library-%20v2.11-yellow.svg)](https://github.com/KitSprout/AltiumDesigner_PcbLibrary/releases/tag/v2.11)


UWB Node 是一款 UWB 室內定位開發模組，採用 STM32F411CE 芯片以及 DWM1000 模組，板上有九軸慣性感測器、氣壓計，並規劃電池電壓檢測功能，預留部分擴充腳位，藉以連接藍牙、WIFI 模組以及 OLED 螢幕。

Hardware
========
* 控制器　 : [STM32F411CE](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1877/PF260148) 48Pin 100MHz DSP FPU
* 感測器   : [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) + [LPS25HB](http://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps25hb.html)
* UWB 　　 : DecaWave [DWM1000](http://www.decawave.com/products/dwm1000-module)
* PCB 尺寸 : 32 x 42 mm
* 設計軟體 [Altium Designer 16](http://www.altium.com/en/products/altium-designer) ( PcbLib use AD [PcbLib v2.11](https://github.com/KitSprout/AltiumDesigner_PcbLibrary/releases/tag/v2.11) )

View
========
<img src="https://lh3.googleusercontent.com/7Pw2hs1VYsGLjtABpcFduyzJaxsVsAAvhohSY-x8XIz-WK31cFIT2X43xj-p394FAXs0iLL4Wc8royz1e4jFeHi3TARjxX5AzuuOlNrBI7QiajPAcbEBuMt77ak4K4IzaatG8WM_uOcWS6AmBGBJ-KT_Zw-iTWHCJgIPIIlmfjJvOif68xk-AD3xJUZSptwifh8aKBvMnu35YqOmgDu1c7bzXekoAAanMeD20_sLfXhT4Zh3vwmK9LmcC7urvqN1IaRK-wl6o5K-LymB_P4UbW_Z66m2u1NT90yDo92-0egTt2BG4BXXjm5GdxKQTz-7IJzjLhhJHtJWxrOO5ao6ToImAK9OSZmr12vGsJrYmf6N8YOA6vlUPkfb0bFeXyg1M8rdVAbpjH4SXkH0Ka_AXu8Iv0bsCoQ0rHnlWd1SSb1ki4iv25exyza05BcAfQrY9XK-CCB1ocp4lH8cR5_ZRQJR0R60A7IVf3xkmi0smRJTkBoC97GAlTJdXd6RmHd2VZ3mkZaGS8tBZ1TSRifELL9rqSL8AWuw8282KX_N_7mtIxMuQM4nzkpI9t-Qb5-gl0qxlwWPFqd17SYPnSWdSXP8paCOF32J4h_60bsFZ0dmQOI1=w1223-h917-no"/>
<img src="https://lh3.googleusercontent.com/SbYLdcZ7h0FxPGI4lBsNTedA9ZBeuQq0bPppfSDIG-wAagN5t5U6dmbyMbVCKdn9WW8nCj-cROBh_RXkAQPMJs9BAomTQ2w-TE1l7g6svvIr3Fs4jfBTCHie8hHsQ08FDLjE4mmSCLLyt-mBWFq3alw-vJlPqjKne6U5Qju7uqpaKY4fXs143D2f5LWlBsrcqXYQtk3LfijRtz9Sn_IE_EWGrJj0Z0_McFiSYzq2401oP5PM6lOo0MTE_gUony6QybTV7vWw5Si11aiApyJE0dlB95bZ35y2c6YnARqqZaqLyT0GdNKXWxcBjaETDxADjCEJLfeFzkSMwoflJbFz9mFkVJS2lFhpe3wgoQ1Lgv31JHaw7_Z_G6ApuGbxXnqSu3Bcm-e_j-4qUslOvohX_SSLU3hdMJKQdQEJcM2vPxq0QjSMBM0dvXEYNYL0hthqiUGv2FCG4_zqBwZ3uKx2gbTeYWiu4FB8B8hpAQH39uPaKsQrMW-SEvEKNo_wC8IFgUFmnf89ZJF9fbmuAom9x09OK452CJXqTR4-zziuNvSNQpcUGWB8UqSsoDzZiTtTzZKK6xZoOU4DM9BOxrhO0l5XV7Q8wiK5kcNxVvKldNorXGSy=w1502-h966-no"/>
<img src="https://lh3.googleusercontent.com/yfL6Q_kI0Ou3svsWRTehPvIl_Id2W6WUOrsX6MfSQMFwU1H8bDQuiFK3KFlxg_TXawHJx-cweIlqXBQ6IwmDtB1i3XGCUWwNJfQk-QSBUAciBagCx72HFDaTsmR3aIzB3K3cpMnnAvODvB8VkXcozqHi1r1V5YuYIgUn6ebkzKlTP3MuQoRcKq30YefuZYO2Y6bXg5_DKUsSXy7ftz1CAGNud8lNYlqjSg00A_AR8dauMTJoHTWLUvnTDUi2cUo1Ticv-odw5H_Ta3m0Gs38m6tweLNddYEyJHBNrmDsZEDScCtE0z27APNzOZT7EQWsQDF9jQsNe5CA-RNfwGetjxdxSk7aCyGygILkM4W8KF-m5w1oijOtTIRJpPc_l3czZoDMGg68uL3ktVCmnglX4VJW7FLKPfv9uYduffOTh4O2WSTczK0gRcTMIL09kXJyQSEpODa6j0Jkuo6zdCvnaoOpT1a3hJ7JAdlWkG3Vrck9eF1OaXHfYUvqRzGPsTYW8wxPQbus9b0CByI4yD92X6yBxw6bt6Tln8jV1xr5TQ22SHRjM5R02UoLsPucoTePwXQbXlqjv7atcuekVQClI5883OEBeIqCeCrqIZN1YfwKHM5k=w1855-h810-no"/>
<br />
[more photo...](https://goo.gl/photos/W93huj9bQzaHpJsv7)

Schematic
========
<img src="https://lh3.googleusercontent.com/_c9s5fUPP0ws6XSgjSPC8QyOcBHaIdRYKTwu9UE1_15Wp90IUHQyrbieDFd82lpufcRFu5bM-wlMdhjtihfZ9sIdDht5S62IjiPnVtzHU_ccfKQedIYL1uennMVyu7O5zc1yKRr_MU7ewd-UxwYsvxZQ01w6sh23egZCWBmCvuUXg-gfF2v1_FIpydlnbsQ9BxUtG-r-kO3x88DlxjYnGlR6I0jnHv8IphFt2rAHATIRheIO5OhtJ2IRpKPrb3H5_8t43qKEuzSvLgtk--Lzc98VN9yBdiKX6tlstigK1Q_oRB44BEH8RqWo6nuqWF4MNASAW_MWOuDk02qrbK03W9C74qZFI3Sy0Mllg3fFVteWc5JdUlH_hypUfvf3xItbXymntiivzXGdeW7ayZrssKSVovFNT_RTOwx78Y9vDzVA33ngH5hnWhlWnBga88GfHMxa67aIJX-HaZU6XeLyVlgI9UfmPPMCQaw0Pi8PVXHYGl5X4HlbOEBWUkGJWa3uZyMaFvEIhIQ2BQRCUcVSy_LgUbiZUoIh79G4jkHNiaN1Y13wM5LY4jCrjC0feePImryK2qlHYKVuhsxss_C66gGumLvV2BGmLpiEu7fQpFkgU7vv=w1243-h966-no"/>
