High-stability Gyro Sensor: XV7001BB
|                                                     |     |     |
| --------------------------------------------------- | --- | --- |
|                                                     |     |     |
| Features                                            |     |     |
| (cid:1)  SPI / I2C serial interface                 |     |     |
| (cid:1)  Angular rate output (16/24bit)             |     |     |
| (cid:1)  Excellent bias stability over temperature  |     |     |
CW(+)
| (cid:1)  Embedded temperature sensor                |     |                                                          |
| --------------------------------------------------- | --- | -------------------------------------------------------- |
| (cid:1)  Integrated user-selectable digital filter  |     |                                                          |
| (cid:1)  Wide supply voltage range 2.7 to 3.6V      |     |                                                          |
| (cid:1)  Low power consumption 0.9mA                |     |                                                          |
| (cid:1)  Rate range ±100 (cid:176)                  | /s  |                                                          |
| (cid:1)  Pb free                                    |     |                                                          |
| (cid:1)  EU RoHS compliant                          |     |                                                          |
|                                                     |     |                                                          |
|                                                     |     | Description                                              |
|                                                     |     | The XV7001BB feature superior bias output stability      |
|                                                     |     | and low noise. Epson achieves this performance by using  |
Applications  quartz substrates manufactured based on our proprietary
(cid:1)  Anti-vibration and attitude control for industrial  QMEMS technology.
applications etc.  In addition to compatibility for digital output interfaces
(cid:1)  Motion detection for man machine interface  (SPI, I2C), these sensors allow for independent interface
|     |     | power supply voltage settings to enable communications     |
| --- | --- | ---------------------------------------------------------- |
|     |     | with various interfaces. User-selectable low-pass filters  |
|     |     | and high-pass filters enable a wide range of cut-off       |
|     |     | frequencies. Designed for low power consumption, these     |
|     |     | sensors are suitable for a wide range of uses, from        |
|     |     | wearable devices and other consumer electronics to         |
|     |     | industrial equipment.                                      |
|     |     |                                                            |

  Seiko Epson Corporation
                     1 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

■About the symbols used in this catalog
● A lead-free product.
● Compliant with the EU RoHS directive.
* About products without the Pb-Free label
Product terminals are lead-free but the internal components of the product contain lead (high melting
point solder lead as well as the lead contained in the glass of an electronic component are both not
applicable under the EU RoHS directive).
● Indicates a product intended for use in an automobile (body, information systems, etc.). The
product has been designed and manufactured in accordance with a quality assurance program
suited for the on-board environment of an automobile.
● Indicates a product intended for use to further the safe operation of an automobile (driving,
stopping, turning). The product has been designed and manufactured in accordance with a
quality assurance program suited for the on-board safety of an automobile.
The following provisions shall apply to the use of this Application Manual.
(1) The manual provides summary information to serve as reference during your consideration of this product. This
manual does not guarantee any product specifications.
(2) The content of this manual may be changed without notification at any time.
(3) The unauthorized reproduction, copying, or use for any other purposes of any parts of this manual is strictly
prohibited.
(4) The circuit examples and usage methods indicated in this manual are merely reference information. We make no
guarantees whatsoever concerning the possibility of infringement or damages involving third party rights
(including industrial property rights) resulting from the use of this information. Furthermore, this manual grants
no permission concerning licensing in relation to the industrial property rights of Epson or any third party.
(5) This manual does not constitute delivery specifications. The use of this product shall require the official exchange
of delivery specifications between the company and Epson. In some cases, the information indicated in this
manual may vary from actual delivery specifications.
(6) Of the products indicated in this manual, the export of products defined as regulated substances under the Foreign
Exchange and Foreign Trade Control Act requires an export permit issued in accordance with said law.
(7) This product was designed for use in general consumer products. Prior approval by Epson is required to use this
product in the following special applications requiring a particularly high level of reliability. We shall bear no
liability whatsoever if this product is used for such purposes without approval.
Space equipment (artificial satellites, rockets, etc.), medical equipment used to maintain life, submarine
repeaters, power plant controllers, fire and theft prevention equipment, transportation equipment, and other
similar applications requiring a high degree of reliability.
Seiko Epson Corporation
2 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

Contents
1. Block diagram ・・・・・・・・・・・・・・・・・・・・ P. 4
2. Functional explanation ・・・・・・・・・・・・・・・・・・・・ P. 5
2-1) Detecting direction
2-2) Interface
2-3) Angular rate output
2-4) Temperature sensor output
3. Electrical characteristics ・・・・・・・・・・・・・・・・・・・・ P. 6
3-1) Absolute maximum ratings
3-2) Operating conditions
3-3) DC Characteristics
3-4) Operating sequence at Start-up
3-5) Characteristics
3-6) Temperature sensor
4. Dimensions ・・・・・・・・・・・・・・・・・・・・ P. 10
4-1) Outline dimensions
4-2) Pin name and description
4-3) Soldering pattern
5. Serial interface ・・・・・・・・・・・・・・・・・・・・ P. 12
5-1) 4-wire SPI
5-2) 3-wire SPI
5-3) I2C
5-4) Multi-slave function
5-5) Angular rate data read
5-6) Temperature sensor output read
5-7) Control MISO/SA0
6. User command register ・・・・・・・・・・・・・・・・・・・・ P. 29
7. Connection diagram ・・・・・・・・・・・・・・・・・・・・ P. 36
8. Soldering profile ・・・・・・・・・・・・・・・・・・・・ P. 38
9. Terminology ・・・・・・・・・・・・・・・・・・・・ P. 38
10. Handling precaution ・・・・・・・・・・・・・・・・・・・・ P. 39
11. Contact ・・・・・・・・・・・・・・・・・・・・ P. 40
Seiko Epson Corporation
3 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

1) Block diagram
The block diagram is indicated below.
VDDM
Regulator
VDDI
VDDL
Drive
Arms
DG
SS
Drive
SCLK
DS
Logic
Power-On
MOSI
Reset
MISO
User
Reserved2 OSC command Sensor
I/O register Element
Reserved1
Reference
data S1
Temp Sensor register
Detect S2
Non-volatile
GND
memory
Detection
Arms
Fig. 1.1 Block diagram
Drive
S1
アAナnロaloグgフ fiィltルer タ
AMP MIXER ADC Logic
pri1m次arLy PLFPF
S2
Fig. 1.2 Block diagram of detecting part
Seiko Epson Corporation
4 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

2) Functional explanation
2-1) Detecting direction
This product detects the angular rate of rotational movement. The correlation between the angular rate
detection axis direction and the detection polarity is shown in the diagram below.
検出回転軸
Detection rotational axis
Directio検n o出f th回e d転et方ect向ab＋le
angular rates +
##11 端Te子rmマinaーl mクark
2-2) Interface
Compatibility for SPI (4-wire, 3-wire) and I2C. As an SPI 4-wire mode expansion function, this sensor
includes a multi-slave function for use in systems that detect two or more axes at once. This makes it
possible to reduce I/O ports and board circuity. These sensors allow for interface power supply voltage
settings (VDDI) that are independent of the power supply voltage (VDDM) to enable communications with
interfaces of various logic levels. The multi-slave function is available only through factory settings
implemented by Epson.
2-3) Angular rate output
Angular rate data is output in a 2's compliment format. 16bit or 24bit can be selected from the register
settings. The AD converter sample rate is 13.770 kHz at frequency code H and 14.160 kHz at frequency
code J. Angular rate data also can be output after processing using the low-pass filter (LPF) or the high-pass
filter (HPF).
LPF: The order is 2nd/3rd/4th, the cutoff frequency can be selected from the following 14 stages.
(10/35/45/50/70/85/100/140/175/200/285/345/400/500 [Hz])
HPF: The order is 1st. When enabled, the cutoff frequency can be selected from the following 7 stages.
(0.01/0.03/0.1/0.3/1/3/10 [Hz])
2-4) Temperature sensor output
Temperature data is output in a 2's compliment format. 8bit/10bit/12bit can be selected from the register
settings.
Seiko Epson Corporation
5 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

3)  Electrical characteristics
3-1)   Absolute maximum ratings

Table 3.1  Absolute maximum ratings
Standard
| Parameter       | Symbol  |       |       |       | Unit         | Comment  |
| --------------- | ------- | ----- | ----- | ----- | ------------ | -------- |
|                 |         | Min.  | Typ.  | Max.  |              |          |
| Supply voltage  | VDDM    | -0.3  |       | 4     | V  GND = 0V  |          |
Supply voltage for
|     | VDDI  | -0.3  |     | 4   | V  GND = 0V  |     |
| --- | ----- | ----- | --- | --- | ------------ | --- |
interface
| Storage temperature  | T   | -40  |     | 85  | ºC    |     |
| -------------------- | --- | ---- | --- | --- | ----- | --- |
STG
| Condition for soldering  |     |     | 350 ºC  3 s  |     |     |     |
| ------------------------ | --- | --- | ------------ | --- | --- | --- |

3-2)   Operating conditions

|     |     | Table 3.2  Operating conditions  |     |     |     |     |
| --- | --- | -------------------------------- | --- | --- | --- | --- |
Standard
| Parameter       | Symbol  |       |       |       | Unit          | Comment  |
| --------------- | ------- | ----- | ----- | ----- | ------------- | -------- |
|                 |         | Min.  | Typ.  | Max.  |               |          |
| Supply voltage  | VDDM    | 2.7   |       | 3.6   | V  GND = 0 V  |          |
Supply voltage for
|     | VDDI  | 1.65  |     | 3.6  | V  GND = 0 V  |     |
| --- | ----- | ----- | --- | ---- | ------------- | --- |
interface
| Operating temperature  | T OPR   | -20  |     | +80  | ºC    |     |
| ---------------------- | ------- | ---- | --- | ---- | ----- | --- |
Supply voltage start up
|     | tPu  | 0.01  |     | 100  | ms  VDDM0% → 90%  |     |
| --- | ---- | ----- | --- | ---- | ----------------- | --- |

time
| SPI clock frequency  |     |     |     | 10   | MHz  VDDI = 2.4 to 3.6 V   |     |
| -------------------- | --- | --- | --- | ---- | -------------------------- | --- |
| SPI clock frequency  |     |     |     | 5    | MHz  VDDI = 1.65 to 2.4 V  |     |
| I2C clock frequency  |     |     |     | 400  | kHz                        |     |
<Notes>
1.  Using the drive frequency integral multiplier as communications clock may result in fluctuations in the angular
rate output.
2.  Acquiring angular rate data as a frequency that is a fraction of the integer for the drive frequency can result in
fluctuations in the angular rate output.
  Seiko Epson Corporation
                     6 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

3-3)  DC Characteristics
Table 3.3  DC Characteristics
(VDDM = 2.7 to 3.6 V, VDDI = 1.65 to 3.6 V, GND = 0 V, Ta = -20 to +80 ºC)
Standard
| Parameter  |     | Symbol  |             |       |       | Unit  | Note  |
| ---------- | --- | ------- | ----------- | ----- | ----- | ----- | ----- |
|            |     |         | Min.        | Typ.  | Max.  |       |       |
|            |     | VIH     | VDDI × 0.7  |       | -     | V     |       |
Logic input voltage
|     |     | VIL  | -           |     | VDDI × 0.3  | V   |              |
| --- | --- | ---- | ----------- | --- | ----------- | --- | ------------ |
|     |     | VOH  | VDDI - 0.4  |     | -           | V   | Load = 1 mA  |
Logic output voltage
|     |     | VOL  | -   |     | 0.4  | V   | Load = 1 mA  |
| --- | --- | ---- | --- | --- | ---- | --- | ------------ |

| 3-4)   Operating sequence at Start-up  |     |     |     |     |     |     |     |
| -------------------------------------- | --- | --- | --- | --- | --- | --- | --- |

Table 3.4  Operating sequence at Start-up
(VDDM = 2.7 to 3.6 V, VDDI = 1.65 to 3.6 V, GND = 0 V, Ta = -20 to +80 ºC)
Standard
|                                  | Parameter  |     | Symbol  | Condition  |       |       | Unit   |
| -------------------------------- | ---------- | --- | ------- | ---------- | ----- | ----- | ------ |
|                                  |            |     |         |            | Min.  | Typ.  | Max.   |
| Serial communication wait time   |            |     | tIF     | -          | 1     | -     | -  ms  |
Temperature sensor data read start time   tTSEN  -  -  -  80  ms
| Start-up time  |     |     | tSTA  | Output code ±1°/s  | -   | -   | 200  ms  |
| -------------- | --- | --- | ----- | ------------------ | --- | --- | -------- |

<Notes>
| 1.  | Conduct serial communication after tIF.                   |     |     |     |     |     |     |
| --- | --------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 2.  | Conduct temperature sensor data acquisition after tTSEN.  |     |     |     |     |     |     |
| 3.  | Conduct angular rate data acquisition after tSTA.         |     |     |     |     |     |     |
VDDM×90%
VDDM
GND
0.01～100ms
|     | xpor                          | レReジgiスsteタr  |     |     |     |     |     |
| --- | ----------------------------- | ------------- | --- | --- | --- | --- | --- |
|     | (I（n内ter部naノl nーodドe名 na）me)  | リrセesッetト     |     |     |     |     |     |
tIF
Seriaシl cリoアmmルu通nic信ation
ウwエaiイt tトim時e 間
温S度tarセt tンemサpeデraーturタe
tTSEN
取senりso込r みdat開a r始ead
tempsensor
(Internal node name)  （内部ノード名）
tSTA
|     | A角ng速ula度r rate    |     |     |     |     | O出utp力ut コcoーde ド |     |
| --- | ------------------ | --- | --- | --- | --- | ----------------- | --- |
|     | 出 o力utpコutー coドde  |     |     |     |     | ±1°/s             |     |

Fig. 3.4 Operating sequence at Start-up

  Seiko Epson Corporation
                     7 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

3-5) Characteristics

Table 3.5  Characteristics
(Unless otherwise specified, VDDM = 2.7 to 3.6 V, VDDI = 1.65 to 3.6 V GND = 0 V, Ta = -20 to +80 ºC)
Standard
| Parameter               | Symbol  |                         | Condition  |         |         |         | Unit       |
| ----------------------- | ------- | ----------------------- | ---------- | ------- | ------- | ------- | ---------- |
|                         |         |                         |            | Min.    | Typ.    | Max.    |            |
|                         |         | Frequency symbol: H     |            | 49.000  | 49.575  | 50.150  | kHz        |
| Drive frequency         | Fd      |                         |            |         |         |         |            |
|                         |         | Frequency symbol: J     |            | 50.450  | 51.025  | 51.600  | kHz        |
| Detuning frequency      | Df      |                         |            | 0.7     | 0.9     | 1.1     | kHz        |
|                         |         | 16bit                   |            |         | 280     |         |            |
| Scale factor            | So      |                         |            |         |         |         | LSB/(°/s)  |
|                         |         | 24bit                   |            |         | 71680   |         |            |
| Scale factor tolerance  | Sp      | Ta = +25 ºC             |            | -2      |         | +2      | %          |
| Scale factor variation  |         | VDDM = 3V, Ta = +25 ºC  |            |         |         |         |            |
|                         | Spt     |                         |            | -3      |         | +3      | %          |
| over temperature        |         | standard                |            |         |         |         |            |
| Bias                    | ZRL     | Ta = +25 ºC             |            |         | 0       |         | LSB        |
| Bias tolerance          | ZRL     | Ta = +25 ºC             |            | -1      |         | +1      | °/s        |
| Bias variation over     |         | VDDM = 3V, Ta = +25 ºC  |            |         |         |         |            |
|                         | ZRL     |                         |            | -5      |         | +5      | °/s        |
t
| temperature             |       | standard     |     |       |      |       |      |
| ----------------------- | ----- | ------------ | --- | ----- | ---- | ----- | ---- |
| Rate range              | I     |              |     | -100  |      | +100  | °/s  |
| Non linearity           | NI    | Ta = +25 ºC  |     | -0.5  |      | +0.5  | %FS  |
| Cross axis sensitivity  | CS    | Ta = +25 ºC  |     | -5    |      | +5    | %    |
| Current consumption     | Iop1  |              |     |       | 900  | 1300  | µA   |
| Stand-by current        | Iop2  |              |     |       | 160  | 340   | µA   |
| Sleep current           | Iop3  |              |     |       | 3    | 25    | µA   |

  Seiko Epson Corporation
                     8 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

3-6) Temperature sensor

Table 3.6  Temperature sensor
(Unless otherwise specified, VDDM = 2.7 to 3.6 V, VDDI = 1.65 to 3.6 V GND = 0 V, Ta = -20 to +80 ºC)
Standard
| Parameter    | Symbol                         | Condition  |       |       |       | Unit  |
| ------------ | ------------------------------ | ---------- | ----- | ----- | ----- | ----- |
|              |                                |            | Min.  | Typ.  | Max.  |       |
| Output code  | 8bit mode, Ta = +25 ºC         |            | 20    | 25    | 30    |       |
|              | Tout  10bit mode, Ta = +25 ºC  |            | 80    | 100   | 120   | LSB   |
|              | 12bit mode, Ta = +25 ºC        |            | 320   | 400   | 480   |       |
Temperature output
|     | Tacc  Ta = +25 ºC  |     | -5  |     | 5   | ºC  |
| --- | ------------------ | --- | --- | --- | --- | --- |
accuracy
| Temperature coefficient  | 8bit mode   |     | 0.9   | 1   | 1.1   |         |
| ------------------------ | ----------- | --- | ----- | --- | ----- | ------- |
|                          | 10bit mode  |     | 3.6   | 4   | 4.4   |         |
|                          | Tsen        |     |       |     |       | LSB/ºC  |
|                          | 12bit mode  |     | 14.4  | 16  | 17.6  |         |
  Seiko Epson Corporation
                     9 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

4) Dimensions
4-1) Outline dimensions
Top View Bottom View
#10 #9 #8 #7 #6
VDDMRese VDDI MISO SCLK
SCLK MISO VDDI ReseVDDM
rved2 /SA0 /SCL
/SCL /SA0 rved2
#1 #2 #3 #4 #5
MOSI SS VDDLRese GND
GND Rese VDDL SS MOSI
/SDA rved1
rved1 /SDA
端子電極の材質：W
Terminal electrode material: W
端子電極の処理：Ni・Auメッ
Terminal electrode treatment: Ni/Au plating
キ
Fig. 4.1 Outline dimensions
4-2) Pin name and description
Table 4.2 Pin name and description
Pin
Pin name I/O Pin Description
number.
4-wire SPI communications mode: serial data input
MOSI/
#1 I/O: Input/Output 3-wire SPI communications mode: serial data input/output
SDA
I2C communications mode: serial data input/output
4-wire SPI communications mode: slave select
#2 SS I: Input 3-wire SPI communications mode: slave select
I2C communications mode: connect to VDDI.
Internal regulator voltage output
#3 VDDL O: Output
Connect to the bypass capacitor 1uF.
#4 Reserved1 I: Input Connect to GND.
#5 GND - GND
#6 VDDM - Power supply voltage
Logic "L" level output
#7 Reserved2 O: Output
Do not connect.
#8 VDDI - Power supply voltage for digital interface
4-wire SPI communications mode: serial data output
MISO/ 3-wire: SPI communications mode: Do not connect.
#9 I/O: Input / Output
SA0 I2C communications mode: select lowest bit of slave address
Default status set to pull down (approx. 100 kΩ).
SCLK/
#10 I: Input Serial clock (4-wire/3-wire/I2C)
SCL
Seiko Epson Corporation
10 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

4-3) Soldering pattern
An example of the recommended foot pattern is shown in the following diagram. During actual board
design, give due consideration to design aspects such as mounting density and solder mount reliability to
ensure optimal design.
Fig. 4.3 Soldering pattern
Seiko Epson Corporation
11 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

5) Serial interface
Access to the sensor is conducted via serial communication. You can select from three methods for the serial
interface: 4-wire SPI, 3-wire SPI, or I2C.
4-wire SPI communication is enabled by turning on the power supply VDDM, waiting for the serial
communication wait time tIF noted in Section 3-4 to elapse, and then setting the slave select signal (SS) to
logic level "L" (because register name SPISel noted in Section 6-16 has a default value of "0": 4-wire SPI).
I2C communication is enabled by setting the SS to logic level "H" (because register name I2C_EN noted in
Section 6-16 has a default value of "1": I2C enable). 3-wire SPI communication is enabled by setting register
name SPISel to "1": 3-wire SPI and setting the SS to logic level "L". However, setting register name SPISel
to "1": 3-wire SPI will disable I2C communication.
When the multi-slave function indicated in Section 5-4 is enabled via factory settings, the 4-wire SPI, 3-wire
SPI, and I2C functions noted in this section cannot be used.
5-1) 4-wire SPI
4-wire SPI communication is 8-bit width serial communication based on the SS, clock signal (SCLK), data
input signal (MOSI), and data output signal (MISO). Set register name SPISel (register for selecting 4-wire
SPI/3-wire SPI) indicated in Section 6-16 to "0" (the default value is "0": 4-wire SPI). To use 4-wire SPI
communication, set register name I2C_EN (register for enabling I2C) to "0" disable (default value is "1":
enable).
The SS falls and the initial byte becomes the address. During serial data transfer, the SS must be maintained
at logic level "L." If the SS is set to logic level "H," the serial data transfer will be canceled.
The initial address bit (MSB) is the write/read control bit. Set as "0" to write data to the register and set as
"1" to read data from the register.
The subsequent bit 2 (A[6: 5]) is the slave device (Gyro) address when the multi-slave function is enabled.
Refer to Section 5-4(Multi-Slave Function) and transfer the address of the slave device (Gyro) you want to
access. Set to "00" when the multi-slave function is disabled.
Bit 5 (A<4: 0>) on the LSB side of the address is the register address. Set the address of the register you
want to access. The 2nd byte is the settings value for each register. Refer to the register map in Chapter 6
and transfer the values you want to set.
SS
SCLK
MOSI W A6 A5 A4 A3 A2 A1 A0 D7 D6 D5 D4 D3 D2 D1 D0
MISO
High-Z
L論o理gi"cL l"eレveベl ル“L”
High-Z
Fig. 5.1.1 Writing to a Register
The register write sequence is shown in Fig. 5.1.1. Write data is transferred after the address. Maintain the
SS at logic level "L" during the period between address and data transfer. During the write sequence, the
MISO logic level is output as "L." X is "1" or "0."
Seiko Epson Corporation
12 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

SS
SCLK
| MOSI   | R A6             | A5 A4 A3 A2 | A1 A0 |       |       |             |        |
| ------ | ---------------- | ----------- | ----- | ----- | ----- | ----------- | ------ |
| High-Z | Logic level “L”  |             |       |       |       |             | High-Z |
| MISO   | 論理"L"レベル         |             |       | D7 D6 | D5 D4 | D3 D2 D1 D0 |        |
Fig. 5.1.2 Reading a Register

The register read sequence is shown in Fig. 5.1.2. After the address transfer is complete, data is output
simultaneously with SCLK fall beginning from the 2nd byte. Similar to the write sequence, during data
non-output, the MISO logic level is output as "L." X is "1" or "0." Angular rate data reads are based on
16-bit output (or 24-bit output based on the angular rate data format selection indicated in Chapter 6). After
reading the angular rate data from the 1st byte, maintain logic level "L" for the SS and continue clock input
via the SCLK until the desired bit is read (the same applies to reading the temperature sensor data).
SS
SCLK
|     | MOSI | W A6                            | A5 A4 | A3 A2 | A1 A0 |        |     |
| --- | ---- | ------------------------------- | ----- | ----- | ----- | ------ | --- |
|     |      | High-Z L論og理ic" Lle"vレeベl “ルL”  |       |       |       | High-Z |     |
|     | MISO |                                 |       |       |       |        |     |
Fig. 5.1.3 Specifying an Address (Command)

The sequence for only the address transfer (command) is shown in Fig. 5.1.3. The register map indicated in
Chapter 6 includes the items for only a partial address transfer (command). Similar to the register write
sequence, set the first bit (MSB) of the address to "0." Bit 5 (A<4: 0>) on the LSB side of the address is the
register address (command). Set the address (command) you want to execute. After transferring the address
(command), set the SS from logic level "L" to logic level "H" and end the serial communication. During the
address tranfer sequence, the MISO logic level is output as "L." X is "1" or "0."

The timing chart is indicated below.

Table 5.1.1  4-wire SPI_AC characterisitc
(VDDM = 2.7 to 3.6 V, GND = 0 V, Ta = -20 to +80 ºC)
Standard
Parameter  Symbol  Condition  VDDI = 1.65 to 2.4 V  VDDI = 2.4 to 3.6 V  Unit
|                         |        |     | Min.  | Typ.  | Max.  Min.  | Typ.  Max.  |     |
| ----------------------- | ------ | --- | ----- | ----- | ----------- | ----------- | --- |
| SS setup time           | tSSS   |     | 15    | -     | -  15       | -  -        | ns  |
| SS hold time            | tSSH   |     | 100   | -     | -  100      | -  -        | ns  |
| SS high pulse width     | tSSHW  |     | 30    | -     | -  30       | -  -        | ns  |
| Clock cycle             | tSCYC  |     | 200   | -     | -  100      | -  -        | ns  |
| Clock high pulse width  | tSHW   |     | 90    | -     | -  40       | -  -        | ns  |
| Clock low pulse width   | tSLW   |     | 90    | -     | -  40       | -  -        | ns  |
| Data setup time         | tSDS   |     | 10    | -     | -  10       | -  -        | ns  |
| Dat a hold time         | tSDH   |     | 10    | -     | -  10       | -  -        | ns  |
Read access time  tSACC  Max CL = 30 pF  -  -  100  -  -  5.0  ns
|     | tSOH  |     | -   | -   | 30  -  | -  30  | ns  |
| --- | ----- | --- | --- | --- | ------ | ------ | --- |
Output disable time
  Seiko Epson Corporation
                     13 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

tSCYC tSSHW
| SS   |      | ∬   |           |     | ∬    |
| ---- | ---- | --- | --------- | --- | ---- |
|      | tSSS |     | tSHW tSLW |     | tSSH |
| SCLK |      | ∬   |           |     | ∬    |
tSDS tSDH
| MOSI | "0" A6 | A5 ∬ | A0  | D7  | ∬ D0 |
| ---- | ------ | ---- | --- | --- | ---- |
High-Z High-Z
|     | Lo論g理ic "lLe"vレelベ “Lル”  | ∬   |     |     | ∬   |
| --- | ------------------------ | --- | --- | --- | --- |
MISO

Writing to the 4-wire SPI Register

tSCYC tSSHW
∬ ∬
SS
|      | tSSS |     | tSHW tSLW |     | tSSH |
| ---- | ---- | --- | --------- | --- | ---- |
| SCLK |      | ∬   |           |     | ∬    |
tSDS tSDH
| MOSI   | "1" A6                   | A5 ∬ | A0  |       | ∬      |
| ------ | ------------------------ | ---- | --- | ----- | ------ |
|        |                          |      |     | tSACC | tSOH   |
| High-Z | Lo論gi理c "leLv"eレl ベ“Lル”  |      |     |       | High-Z |
| MISO   |                          | ∬    |     | D7    | ∬ D0   |

Reading the 4-wire SPI Register
(Note) X is "1" or "0."

  Seiko Epson Corporation
                     14 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

5-2) 3-wire SPI
3-wire SPI communication is 8-bit width serial communication based on the slave select signal (SS), clock
signal (SCLK), and data input/output signal (MOSI). Set register name SPISel (register for selecting 3-wire
SPI/4-wire SPI) indicated in Chapter 6 (Table 6.16 Serial Interface Settings) to "1" (the default value is "0":
4-wire SPI). Setting SPISel to "1" will disable I2C communication.
Similar to 4-wire SPI, the SS falls and the initial byte becomes the address. During serial data transfer, the
SS must be maintained at logic level "L." If the SS is set to logic level "H," the serial data transfer will be
canceled.
The initial address bit (MSB) is the write/read control bit. Set as "0" to write data to the register and set as
"1" to read data from the register. Bit 5 (A<4: 0>) on the LSB side of the address is the register address. Set
the address of the register you want to access. The 2nd byte is the settings value for each register. Refer to
the register map in Chapter 6 and transfer the values you want to set.
SS
SCLK
MOSI W A6 A5 A4 A3 A2 A1 A0 D7 D6 D5 D4 D3 D2 D1 D0
Fig. 5.2.1 Writing to a Register
The register write sequence is shown in Fig. 5.2.1. Write data is transferred after the address. Maintain the
SS at logic level "L" during the period between address and data transfer. X is "1" or "0."
SS
SCLK
MOSI R A6 A5 A4 A3 A2 A1 A0 D7 D6 D5 D4 D3 D2 D1 D0
Fig. 5.2.2 Reading a Register
The register read sequence is shown in Fig. 5.2.2. After the address transfer is complete, data is output
simultaneously with SCLK fall beginning from the 2nd byte. Angular rate data reads are based on 16-bit
output (or 24-bit output based on the angular rate data format selection indicated in Chapter 6). After reading
the angular rate data from the 1st byte, maintain the SS at logic level "L" and continue clock input via the
SCLK until the desired bit is read. The same applies to reading the temperature sensor data. X is "1" or "0."
SS
SCLK
MOSI W A6 A5 A4 A3 A2 A1 A0
Fig. 5.2.3 Specifying an Address (Command)
Seiko Epson Corporation
15 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

The sequence for only the address transfer (command) is shown in Fig. 5.2.3. The register map indicated in
Chapter 6 includes the items for only a partial address transfer (command). Similar to the register write
sequence, set the first bit (MSB) of the address to "0." Bit 5 (A<4: 0>) on the LSB side of the address is the
register address (command). Set the address (command) you want to execute. After transferring the address
(command), set the SS from logic level "L" to logic level "H" and end the serial communication. X is "1" or
"0."

The timing chart is indicated below.

Table 5.2.1  3-wire SPI_AC characterisitc
(VDDM = 2.7 to 3.6 V, GND = 0 V, Ta = -20 to +80 ºC)
Standard
Paramter  Symbol  Condition  VDDI = 1.65 to 2.4 V  VDDI = 2.4 to 3.6 V  Unit
|                |        |     |     | Min.  | Typ.  | Max.  Min.  | Typ.  | Max.  |     |
| -------------- | ------ | --- | --- | ----- | ----- | ----------- | ----- | ----- | --- |
| SS setup time  | tSSS   |     |     | 15    | -     | -           | 15    | -  -  | ns  |
| SS hold time   | tSSH   |     |     | 100   | -     | -           | 100   | -  -  | ns  |
|                | tSSHW  |     |     | 30    | -     | -           | 30    | -  -  | ns  |
SS high pulse width
| Clock cycle  | tSCYC  |     |     | 200  | -   | -   | 100  | -  -  | ns  |
| ------------ | ------ | --- | --- | ---- | --- | --- | ---- | ----- | --- |
|              | tSHW   |     |     | 90   | -   | -   | 40   | -  -  | ns  |
Clock high pulse width
| Clock low pulse width  | tSLW   |                 |     | 90  | -   | -    | 40  | -  -    | ns  |
| ---------------------- | ------ | --------------- | --- | --- | --- | ---- | --- | ------- | --- |
| Data setup time        | tSDS   |                 |     | 10  | -   | -    | 10  | -  -    | ns  |
| Dat a hold time        | tSDH   |                 |     | 10  | -   | -    | 10  | -  -    | ns  |
|                        | tSACC  | Max CL = 30 pF  |     | -   | -   | 100  | -   | -  5.0  | ns  |
Read access time
| Output disable time  | tSOH  |     |     | -   | -   | 30  | -   | -  30  | ns  |
| -------------------- | ----- | --- | --- | --- | --- | --- | --- | ------ | --- |

|     |      |     |     |     | tSCYC |      |      | tSSHW |     |
| --- | ---- | --- | --- | --- | ----- | ---- | ---- | ----- | --- |
| SS  |      |     |     | ∬   |       |      | ∬    |       |     |
|     | tSSS |     |     |     | tSHW  | tSLW | tSSH |       |     |
|     |      |     |     | ∬   |       |      | ∬    |       |     |
SCLK
|     |     | tSDS | tSDH |     |     |     |     |     |     |
| --- | --- | ---- | ---- | --- | --- | --- | --- | --- | --- |
|     | "0" | A6   | A5   | ∬   | A0  | D7  | ∬   | D0  |     |
MOSI

Fig. 5.2.4 Writing to the 3-wire SPI Register

|     |      |     |     |     | tSCYC |      |      | tSSHW |     |
| --- | ---- | --- | --- | --- | ----- | ---- | ---- | ----- | --- |
| SS  |      |     |     | ∬   |       |      | ∬    |       |     |
|     | tSSS |     |     |     | tSHW  | tSLW | tSSH |       |     |
|     |      |     |     | ∬   |       |      | ∬    |       |     |
SCLK
|      |     | tSDS | tSDH |     |     | tSACC |     | tSOH |     |
| ---- | --- | ---- | ---- | --- | --- | ----- | --- | ---- | --- |
| MOSI | "1" | A6   | A5   | ∬   | A0  | D7    | ∬   | D0   |     |

Fig. 5.2.5 Reading the 3-wire SPI Register

 (Note)  X is "1" or "0."

  Seiko Epson Corporation
                     16 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

5-3) I2C
I2C communication is 8-bit width serial communication based on the clock signal (SCL) and data address
signal (SDA). Set register name I2C_EN (register enabling for I2C) indicated in Chapter 6 (Table 6.16 Serial
Interface Settings) to "1" (the default value is "1": Enable). To use I2C communication, fix the slave select
signal (SS) used in 4-wire/3-wire SPI communication to logic level "H" (interface power supply voltage
VDDI).
I2C communication is initiated by issuing the start condition (ST, with SCL status at logic level "H", SDA is
changed from logic level "H" to logic level "L") from the master. Or, communication is stopped by issuing
the stop condition (SP, with SCL status at logic level "H", SDA is changed from logic level "L" to logic level
"H") from the master.
To access the internal register, read (Read=”1”)/write (Write=”0”) the slave device address (ADR, address
that adds the SA0 "0" or "1" to "110101") to/from the master and transmit the total 1 byte that includes 1 bit
ID signal. After ADR receipt, the slave will check to see if the address matches its own address. If matching,
the slave returns an ACK (acknowledge), after which communication is possible. If the address does not
match, the slave returns to idle mode and waits until another ST is issued.
Internally, the SA0 terminal is set to pull down (approx. 100 kΩ). If the SA0 terminal is set to "0," then
connect to N.C. or GND. Or, if the SA0 terminal is set to "1," then connect to VDDI. In this case, a current
of approximately 30 uA @VDDI = 3V will flow to the SA0 terminal. To reduce the current, add a desired
resistor to the VDDI and SA0 terminal. Alternatively, you can change the terminal setting from pull down to
pull up by rewriting the register name SelMISO[1: 0] indicated in Section 6-15 after turning the power ON
and once the serial communication wait time tIF indicated in Section 3-4 has elapsed. Refer to Section 5-7
MISO/SA0 Terminal Control Methods for details on control methods.
Next, send the internal register address (SUB-ADR). Input "0" for the first bit (MSB) of the address (there is
not function allocation). The remaining LSB-side 7 bits (A<6: 0> are the register address (for detail, refer to
register map in Chapter 6). After transferring the addres of the register you wish to access, return an ACK.
The following sequence differs between register write, register read, and address (command) transfer. Refer
to the sequences in Table 5.3.1 through 5.3.3.
Angular rate data reads are based on 16-bit output (or 24-bit output based on the angular rate data format
selection indicated in Chapter 6). After reading the angular rate data from the 1st byte, set the master to
return an ACK (acknowledge) instead of an NACK (non-acknowledge) and then read the 2nd byte or the 3rd
byte. The same applies to reading the temperature sensor data.
Table 5.3.1 Writing to a Register
Master ST ADR W SUB-ADR DATA SP
Slave ACK ACK ACK
Table 5.3.2 Reading a Register
Master ST ADR W SUB-ADR ST ADR R NACK SP
Slave ACK ACK ACK DATA
Table 5.3.3 Specifying an Address (Command)
Master ST ADR W SUB-ADR SP
Slave ACK ACK
ST : start condition
SP : stop condition
ADR : slave device address (110101+SA0)
Seiko Epson Corporation
17 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

| R/W      |     | : Read = “1”, Write = “0”             |     |     |     |     |     |     |     |
| -------- | --- | ------------------------------------- | --- | --- | --- | --- | --- | --- | --- |
| SUB-ADR  |     | : internal register address           |     |     |     |     |     |     |     |
| DATA     |     | : internal register read/write data   |     |     |     |     |     |     |     |
| ACK      |     | : “Low”                               |     |     |     |     |     |     |     |
| NACK     |     | : “High,” send at read complete.      |     |     |     |     |     |     |     |

As an example of a waveform, register write, register read, and address (command) transfer are shown in
Fig. 5.3.4 through 5.3.6.

| SS  |     | HiHgihg h(（VVDDDDII）) 固Fi定xed |     |     |     |     |     |     |     |
| --- | --- | ----------------------------- | --- | --- | --- | --- | --- | --- | --- |
SCL
|            |     |             |         |      | High-Z |       |       |       | High-Z |
| ---------- | --- | ----------- | ------- | ---- | ------ | ----- | ----- | ----- | ------ |
| SDA-Master | A6  | A5 A4 A3    | A2 A1   | A0 W | "0"    | A6 A5 | A4 A3 | A2 A1 | A0     |
|            | "1" | "1" "0" "1" | "0" "1" |      |        |       |       |       |        |
"0"or"1"
| SDA-Slave |     | High-Z |     |     | ACK |     | High-Z |     | ACK |
| --------- | --- | ------ | --- | --- | --- | --- | ------ | --- | --- |
Start

| SS  |     | HiHgihg h(（VVDDDDII）) 固Fi定xed |     |     |     |     |     |     |     |
| --- | --- | ----------------------------- | --- | --- | --- | --- | --- | --- | --- |
SCL
High-Z
| SDA-Master | D7 D6 | D5 D4 D3 | D2 D1 | D0  |     |     |     |     |     |
| ---------- | ----- | -------- | ----- | --- | --- | --- | --- | --- | --- |
| SDA-Slave  |       | High-Z   |       | ACK |     |     |     |     |     |
Stop
Fig. 5.3.4 Writing to a Register

| SS  |     | HiHgihgh (（VVDDDDII）)固 F定ixed |     |     |     |     |     |     |     |
| --- | --- | ----------------------------- | --- | --- | --- | --- | --- | --- | --- |
SCL
|            |         |             |       | High-Z |     |       |       |       | High-Z |
| ---------- | ------- | ----------- | ----- | ------ | --- | ----- | ----- | ----- | ------ |
| SDA-Master | A6 A5   | A4 A3 A2    | A1 A0 | W      | "0" | A6 A5 | A4 A3 | A2 A1 | A0     |
|            | "1" "1" | "0" "1" "0" | "1"   |        |     |       |       |       |        |
"0"or"1"
SDA-Slave
|     |     | High-Z |     |     | ACK |     | High-Z |     | ACK |
| --- | --- | ------ | --- | --- | --- | --- | ------ | --- | --- |
Start

| SS  |     | HHigihgh (（VVDDDDII）)固 F定ixed |     |     |     |     |     |     |     |
| --- | --- | ----------------------------- | --- | --- | --- | --- | --- | --- | --- |
SCL
|     | A6  | A5 A4 A3 A2 | A1 A0 | R   |     |     | High-Z |     | NACK |
| --- | --- | ----------- | ----- | --- | --- | --- | ------ | --- | ---- |
SDA-Master
|     | "1" "1" | "0" "1" "0" | "1" |     |     |     |     |     |     |
| --- | ------- | ----------- | --- | --- | --- | --- | --- | --- | --- |
"0"or"1"
SDA-Slave
|     |         | High-Z |     |     | ACK D7 | D6 D5 | D4 D3 | D2 D1 | D0     |
| --- | ------- | ------ | --- | --- | ------ | ----- | ----- | ----- | ------ |
|     | Restart |        |     |     |        |       |       |       | Stop   |
Fig. 5.3.5 Reading a Register

  Seiko Epson Corporation
                     18 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

| SS  | HigHh（igVhD D(VI）D固D定I) Fixed |     |     |     |     |     |     |
| --- | ----------------------------- | --- | --- | --- | --- | --- | --- |
SCL
|            |         |             |       | High-Z   |          | High-Z   |     |
| ---------- | ------- | ----------- | ----- | -------- | -------- | -------- | --- |
| SDA-Master | A6 A5   | A4 A3 A2    | A1 A0 | W "0" A6 | A5 A4 A3 | A2 A1 A0 |     |
|            | "1" "1" | "0" "1" "0" | "1"   |          |          |          |     |
"0"or"1"
| SDA-Slave |       |        |     |     |        | ACK |        |
| --------- | ----- | ------ | --- | --- | ------ | --- | ------ |
|           |       | High-Z |     | ACK | High-Z |     |        |
|           | Start |        |     |     |        |     | Stop   |
Fig. 5.3.6 Specifying an Address (Command)

The timing chart is indicated below.

Table 5.3.1  I2C_AC characteristic
(VDDM = 2.7 to 3.6 V, GND = 0 V, Ta = -20 to +80 ºC)
Standard
|     | Parameter  |     | Symbol  | Conditions  |       |             | Unit  |
| --- | ---------- | --- | ------- | ----------- | ----- | ----------- | ----- |
|     |            |     |         |             | Min.  | Typ.  Max.  |       |
|     |            |     | tSCL    |             | 2.5   | -  -        | us    |
Clock cycle
| Clock high pulse width  |     |     | tWH  |     | 0.6  | -  -  | us  |
| ----------------------- | --- | --- | ---- | --- | ---- | ----- | --- |
|                         |     |     | tWL  |     | 1.3  | -  -  | us  |
Clock low pulse width
| Data setup time  |     |     | tDS  |     | 0.1  | -  -  | us  |
| ---------------- | --- | --- | ---- | --- | ---- | ----- | --- |
|                  |     |     | tDH  |     | 0.0  | -  -  | us  |
Data hold time
| START condition hold time  |     |     | tSH  |     | 0.6  | -  -  | us  |
| -------------------------- | --- | --- | ---- | --- | ---- | ----- | --- |
|                            |     |     | tRS  |     | 0.6  | -  -  | us  |
Time restart condition setup time
| STOP condition setup time  |     |     | tPS  |     | 0.6  | -  -  | us  |
| -------------------------- | --- | --- | ---- | --- | ---- | ----- | --- |
|                            |     |     | tWS  |     | 1.3  | -  -  | us  |
Between STOP and START condition

tWH tWL
| SCL |     |     |     | ∬   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
REPEATED
START
|     | tSH | tDS |     | tDH |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
tRS
| SDA |     |     |     | ∬   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
START
tWS
START
tPS
STOP

Fig. 5.3.7  I2C timing

  Seiko Epson Corporation
                     19 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

5-4) Multi-slave function
The multi-slave function is an extended function of SPI 4-wire mode that is available only through factory
settings implemented by Epson. The multi-slave function allows you to reduce I/O ports and board circuitry
by connecting multiple (maximum of 3) slave devices (Gyro) to a single master device (MPU). When the
multi-slave function is enabled via factory settings, other serial communication (4-wire SPI, 3-wire SPI, and
I2C communication) cannot be used.
SS
SCLK
MPU 1s1t 軸ax目isG gyyroro
（(マMスaタstーerデ deバvイicスe)）
MOSI
（ス(レSlーavブe デdeバviイceス) 1）
MISO
2n2d軸 ax目isG gyryoro
（ス(レSlーavブe デdeバviイceス) 2）
3r3d軸 ax目isG gyyroro
（ス(レSlーavブe デdeバviイceス) 3）
Fig. 5.4.1 Connection Examples
A connection example using 3 slave devices is shown in Fig. 5.4.1. Three slave devices (Gryo) share a serial
communications port connection on a single master device (MPU).
Each slave device (Gryo) is assigned the respective address shown in Table 5.4.1. Setting the address A[6: 5]
indicated in Chapter 5 enables serial communication with the desired slave device.
Table 5.4.1 Slave Device Addresses
A[6] A[5] Selected slave device
0 0 All slave devices
0 1 1st axis Gyro (frequency code H)
1 0 2nd axis Gyro (frequency code J)
1 1 3rd axis Gyro (frequency code K)
(Note) When the maximum 3 slave devices are connected
Furthermore, when the address A[6: 5] is set to "00", a register write command will write the same data to
all slave devices but register read commands are not common to all devices. Even if the read/write control
bit (first bit of address: MSB) indicated in Chapter 5 is mistakenly set to "1" (read), the MISO value is
"High-Z" (excluding global angular rate reads).
Seiko Epson Corporation
20 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

SS
SCLK
MOSI W A6 A5 A4 A3 A2 A1 A0 D7 D6 D5 D4 D3 D2 D1 D0
High-Z
MISO
Fig. 5.4.2 Writing to a Register
The register write sequence when using the multi-slave function is shown in Fig. 5.4.2. Write data is
transferred after the slave device address and register address. Maintain the SS at logic level "L" during the
period between address and data transfer. During the write sequence, the MISO value is "High-Z". X is "1"
or "0".
SS
SCLK
MOSI R A6 A5 A4 A3 A2 A1 A0
High-Z High-Z
MISO D7 D6 D5 D4 D3 D2 D1 D0
Fig. 5.4.3 Writing to a Register
The register read sequence when using the multi-slave function is shown in Fig. 5.4.3. After the slave device
address and register address transfer is complete, data is output simultaneously with SCLK fall beginning
from the 2nd byte. Similar to the write sequence, during data non-output, the MISO setting is "High-Z". X is
"1" or "0". Angular rate data reads are based on 16-bit output (or 24-bit output based on the angular rate data
format selection indicated in Chapter 6). After reading the angular rate data from the 1st byte, maintain logic
level "L" for the SS and continue clock input via the SCLK until the desired bit is read (the same applies to
reading the temperature sensor data).
SS
SCLK
MOSI W A6 A5 A4 A3 A2 A1 A0
High-Z
MISO
Fig. 5.4.4 Specifying an Address (Command)
The sequence for only the address transfer (command) via the multi-slave function is shown in Fig. 5.4.4.
Maintain the SS at logic level "L" during address transfer. During the address transfer sequence, the MISO
value is "High-Z". X is "1" or "0".
Seiko Epson Corporation
21 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

The timing chart is indicated below.

Table 5.4.2  4-wire SPI multi slave_AC characteristic
(VDDM = 2.7 to 3.6 V, GND = 0 V, Ta = -20 to +80 ºC)
Standard
Parameter  Symbol  Condition  VDDI = 1.65 to 2.4 V  VDDI = 2.4 to 3.6 V  Unit
|                         |     |        |     | Min  | Typ  | Max  | Min  Typ  | Max   |     |
| ----------------------- | --- | ------ | --- | ---- | ---- | ---- | --------- | ----- | --- |
| SS setup time           |     | tSSS   |     | 15   | -    | -    | 15        | -  -  | ns  |
| SS hold time            |     | tSSH   |     | 100  | -    | -    | 100       | -  -  | ns  |
| SS high pulse width     |     | tSSHW  |     | 30   | -    | -    | 30        | -  -  | ns  |
| Clock cycle             |     | tSCYC  |     | 200  | -    | -    | 100       | -  -  | ns  |
| Clock high pulse width  |     | tSHW   |     | 90   | -    | -    | 40        | -  -  | ns  |
| Clock low pulse width   |     | tSLW   |     | 90   | -    | -    | 40        | -  -  | ns  |
| Data setup time         |     | tSDS   |     | 10   | -    | -    | 10        | -  -  | ns  |
| Data hold time          |     | tSDH   |     | 10   | -    | -    | 10        | -  -  | ns  |
Read access time  tSACC  Max CL = 30 pF  -  -  100  -  -  5.0  ns
| Output disable time  |     | tSOH  |     | -   | -   | 30  | -   | -  30  | ns  |
| -------------------- | --- | ----- | --- | --- | --- | --- | --- | ------ | --- |

|     |     |     |     |     | tSCYC |     |     | tSSHW |     |
| --- | --- | --- | --- | --- | ----- | --- | --- | ----- | --- |
|     |     |     |     | ∬   |       |     | ∬   |       |     |
SS
|      | tSSS |      |      |     | tSHW | tSLW | tSSH |     |     |
| ---- | ---- | ---- | ---- | --- | ---- | ---- | ---- | --- | --- |
| SCLK |      |      |      | ∬   |      |      | ∬    |     |     |
|      |      | tSDS | tSDH |     |      |      |      |     |     |
| MOSI | "0"  | A6   | A5   | ∬   | A0   | D7   | ∬    | D0  |     |
High-Z
| MISO |     |     |     | ∬   |     |     | ∬   |     |     |
| ---- | --- | --- | --- | --- | --- | --- | --- | --- | --- |

Fig. 5.4.5 Writing to a Register

|        |      |      |      |     | tSCYC |       |      | tSSHW  |     |
| ------ | ---- | ---- | ---- | --- | ----- | ----- | ---- | ------ | --- |
| SS     |      |      |      | ∬   |       |       | ∬    |        |     |
|        | tSSS |      |      |     | tSHW  | tSLW  | tSSH |        |     |
| SCLK   |      |      |      | ∬   |       |       | ∬    |        |     |
|        |      | tSDS | tSDH |     |       |       |      |        |     |
|        |      |      |      | ∬   |       |       | ∬    |        |     |
| MOSI   | "1"  | A6   | A5   |     | A0    |       |      |        |     |
|        |      |      |      |     |       | tSACC |      | tSOH   |     |
| High-Z |      |      |      |     |       |       |      | High-Z |     |
| MISO   |      |      |      | ∬   |       | D7    | ∬    | D0     |     |

Fig. 5.4.6 Reading a Register

(Note 1)  X is "1" or "0".

  Seiko Epson Corporation
                     22 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

| 5-5)   | Angular rate data read  |     |     |     |     |     |     |
| ------ | ----------------------- | --- | --- | --- | --- | --- | --- |
5-5-1) When multi-slave function is disabled
The angular rate read function is conducted using the register name DatAccOn indicated in Chapter 6 (Table
6 User Command Register). Angular rate data uses the 2's compliment expression and has a data width of 16
bit or 24 bit (switch using the register name DataFormat indicated in Table 6.11). During serial
communication, after reading the angular rate data from the 1st byte, maintain logic level "L" for the SS and
continue clock input via the SCLK until the desired bit is read.

Table 5.1 Angular rate data output control
|     |     |     |     | Data  Data output order   |     |     |     |
| --- | --- | --- | --- | ------------------------- | --- | --- | --- |
A[6: 5]
|     |     |     |     | Format  1byte  | 2byte     | 3byte    |     |
| --- | --- | --- | --- | -------------- | --------- | -------- | --- |
|     |     |     | 00  | 0  D[15: 8]    | D[7: 0]   |          |     |
|     |     |     | ↑   | 1  D[23: 16]   | D[15: 8]  | D[7: 0]  |     |

(Note)  Register name DataFormat is the angular data format (DataFormat = 0: 16bit output/1: 24bit output)
indicated in Chapter 6 (Table 6.11 Angular rate data read control)

5-5-2) When multi-slave function is enabled
The angular rate read function is conducted using the register name DatAccOn indicated in Chapter 6 (Table
6. User Command Register). Angular rate data uses the 2's compliment expression and has a data width of
16 bit or 24 bit (switch using the register name DataFormat indicated in Table 6.11). There are two read
methods, global angular rate read and normal angular rate read.

| SS   |        |     |          |          | ～   | ～   | ～      |
| ---- | ------ | --- | -------- | -------- | --- | --- | ------ |
| SCLK |        |     |          |          | ～   | ～   | ～      |
|      |        | A6  | A5 A4 A3 | A2 A1 A0 |     |     |        |
| MOSI |        | R 0 | 0 0 1    | 0 1 0    | ～   | ～   | ～      |
|      | High-Z |     |          |          |     |     | High-Z |
MISO D1[15] D1[14] ～ D1[0] D2[15] D2[14] ～ D2[0] D3[15] D3[14] ～ D3[0]
|                                  | High-Z |     |     |               |         |                       | High-Z |
| -------------------------------- | ------ | --- | --- | ------------- | ------- | --------------------- | ------ |
| MMIISSOO__11s軸t a目xisG gyyrroo   |        |     |     | D1[15] D1[14] | ～ D1[0] | ～                     | ～      |
|                                  | High-Z |     |     |               |         |                       | High-Z |
| MMIISSOO__22n軸d a目xiGs gyyroro   |        |     |     |               | ～       | D2[15] D2[14] ～ D2[0] | ～      |
| MMIISSOO__33r軸d a目xisG gyyrroo   | High-Z |     |     |               | ～       | ～                     | High-Z |
D3[15] D3[14] ～ D3[0]
Fig. 5.5.2.1 Global angular rate read

Global angular rate read is shown in Fig. 5.5.2.1. To conduct a global angular rate read, set the slave device
address A[6: 5] to "00". After the angular rate data for the 1st axis gyro, the angular rate for the 2nd gyro and
then the 3rd gyro are output. During serial communication, maintain logic level "L" for the SS and continue
clock input via the SCLK until the desired bit is read. If the logic level for the SS is changed from "L" to
"H" before the desired bit is read, then no further angular rate data is output. Redo the angular rate data read
command (DatAccOn).

If the SCLK is stopped with the SS level maintained at level "L" before the desired bit is read, the angular
rate data read can be output by restarting the SCLK input.
  Seiko Epson Corporation
                     23 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

SS
| SCLK |     | ～            | ～   | ～   | ～   |     | ～            | ～   | ～   | ～   |
| ---- | --- | ------------ | --- | --- | --- | --- | ------------ | --- | --- | --- |
| MOSI | R   | Aアdドdレreスss  |     |     |     | R   | Aアdドdレreスss  |     |     |     |
High-Z 11s軸t ax目isG gyyrroo  22nd軸 ax目isG gyyrroo  33rd軸 ax目isG gyyrroo  11s軸t ax目isG gyyrroo  22n軸d a目xisG gyyrroo  33rd軸 ax目isG gyyrroo
MISO
Fig. 5.5.2.2 Global angular rate read operation method 1

Global angular rate read operation method 1 is shown in Fig. 5.5.2.2. After the angular rate data for the 1st
axis gyro, the angular rate for the 2nd gyro and then the 3rd gyro are output. After the desired bit is read,
raise the SS from logic level "L" to logic level "H." Run the angular rate data read command (DatAccOn) to
read the angular rate data again.

SS
|     | SCLK |     | ～   | ～   | ～   | ～   | ～   | ～   |     | ～   |
| --- | ---- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
Aアdドdレreスss
|     | MOSI | R   |     |     |     |     |     |     |     |     |
| --- | ---- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
High-Z
MISO 11st軸 ax目is Ggyyrroo  22n軸d a目xisG gyyrroo  11s軸t ax目isG gyyrroo  22n軸d a目xisG gyyrroo  11s軸t ax目isG gyyrroo  22n軸d a目xisG gyyrroo
|     |     |     |     | 11s順t o目rd目er  |     | 22順nd o目rd目er  |     |     | 33r順d o目rd目er  |     |
| --- | --- | --- | --- | -------------- | --- | -------------- | --- | --- | -------------- | --- |
Fig. 5.3.2.3 Global angular rate read operation method 2 (2-axis setting)

SS
|     | SCLK |     | ～            | ～   | ～   | ～   | ～   | ～   |     | ～   |
| --- | ---- | --- | ------------ | --- | --- | --- | --- | --- | --- | --- |
|     | MOSI | R   | Aアdドdレreスss  |     |     |     |     |     |     |     |
High-Z
MISO 11st軸 ax目is Ggyyrroo  22nd軸 ax目isG gyyrroo  33rd軸 ax目isG gyyrroo  11s軸t ax目is Ggyyrroo  22n軸d a目xisG gyyrroo  33rd軸 ax目isG gyyrroo
|     |     |     |     |     | 11順st o目rd目er  |     |     | 22n順d o目rd目er  |     |     |
| --- | --- | --- | --- | --- | -------------- | --- | --- | -------------- | --- | --- |
Fig. 5.3.2.4 Global angular rate read operation method 2 (3-axis setting)

Table 5.5.2.1 Global angular rate read operation method 2 - Data output order
| Epson factory settings   |     |     |            |     |     | Data output order   |     |     |            |     |
| ------------------------ | --- | --- | ---------- | --- | --- | ------------------- | --- | --- | ---------- | --- |
|                          |     |     | 1st order  |     |     | 2nd order           |     |     | 3rd order  |     |
2-axis setting
1st axis gyro  2nd axis gyro  1st axis gyro  2nd axis gyro  1st axis gyro  2nd axis gyro
|     |     |     |     | 1st order  |     |     |     | 2nd order   |     |     |
| --- | --- | --- | --- | ---------- | --- | --- | --- | ----------- | --- | --- |
3-axis setting
1st axis gyro  2nd axis gyro  3rd axis gyro  1st axis gyro  2nd axis gyro  3rd axis gyro

For the global angular rate read operation method 2, 2-axis settings (Epson factory settings) are shown in
Fig. 5.5.2.3 and 3-axis settings (Epson factory settings) are shown in Fig. 5.5.2.4. The data output order is
shown in Table 5.5.2.1. After the angular rate data for the 2-axis or 3-axis gyro is output, input the SCLK
with the SS logic level at "L" to output the 2nd order angular rate data (1st - 2nd gyro or 1st - 3rd gyro). This
angular rate data read can be repeated in the order of 3rd, 4th until the SS logic level is set to "H"
  Seiko Epson Corporation
                     24 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

The read angular rate data update is conducted during the read period for the lower 8 bits of the angular rate
data of the previous axis (ex: when using a 3-axis setting, the angular rate data for 1st axis gyro of the 2nd
order is updated during the read period for the lower 8 bits of angular rate data from the 3rd axis gyro of the
1st order).
There are two global angular rate read types, 2-axis setting and 3-axis setting (both Epson factory settings).
Mistakenly mixing the gyro for the 2-axis and 3-axis settings will result in a conflict in the output angular
rate data.
SS ～
SCLK ～
A6 A5 A4 A3 A2 A1 A0
MOSI R 0 1 0 1 0 1 0 ～
High-Z S任pe意cifのy ｽdeﾚsｰirﾌeﾞdﾃ ﾞsﾊlﾞaｲvｽeを de指vi定ce High-Z
MISO D[15] D[14] ～ D[0]
High-Z High-Z
MMIISSOO__11s軸t a目xisG gyyrroo D1[15] D1[14] ～ D1[0]
High-Z
MMIISSOO__22n軸d a目xisG gyyroro
High-Z
MMIISSOO__33r軸d a目xisG gyyrroo
Fig. 5.5.2.5 Normal angular rate read
Normal angular rate read is shown in Fig. 5.5.2.5. For the slave device address A[6: 5] used during normal
angular rate read, set a desired slave device address (any of the Gyro addresses for the 1st through 3rd axes)
from the addresses indicated in Chapter 5 (Table 5.4.1 Slave Device Addresses). During serial
communication, maintain logic level "L" for the SS and continue clock input via the SCLK until the desired
bit is read. If the logic level for the SS is changed from "L" to "H" before the desired bit is read, then no
further angular rate data is output. Redo the angular rate data read command (DatAccOn).
If the SCLK is stopped with the SS logic level maintained at "L" before the desired bit is read, the angular
rate data output can be continued by restarting the SCLK input.
After all angular rate data is output, no additional angular rate data is output even if the SCLK is input with
the SS logic level maintained at "L" (MISO is output at logic level "L".) To read angular rate data again, set
the SS logic level to "H" (MISO is set to "High-Z") and execute the angular rate data read command
(DatAccOn) again.
The angular rate data output control for both global angular rate read and normal angular rate read are
indicated in Table 5.5.2.2.
Table 5.5.2.2 Angular rate data output control
Data Data output order
Read method A[6: 5]
Format 1byte 2byte 3byte 4byte 5byte 6byte 7byte 8byte 9byte
2-axis 0 D1[15: 8] D1[7: 0] D2[15: 8] D2[7: 0]
Global setting 1 D1[23: 16] D1[15: 8] D1[7: 0] D2[23: 16] D2[15: 8] D2[7: 0]
Angular rate 00
Read 3-axis 0 D1[15: 8] D1[7: 0] D2[15: 8] D2[7: 0] D3[15: 8] D3[7: 0]
setting 1 D1[23: 16] D1[15: 8] D1[7: 0] D2[23: 16] D2[15: 8] D2[7: 0] D3[23: 16] D3[15: 8] D3[7: 0]
Other than 0 D[15: 8] D[7: 0]
Normal angular rate read
00 1 D[23: 16] D[15: 8] D[7: 0]
(Note 1) The two types of global angular rate read, 2-axis setting and 3-axis setting, are both Epson factory
settings.
(Note 2) The address A[6: 5] is the slave device address indicated in Table 5.4.1.
Seiko Epson Corporation
25 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

(Note 3) Register name DataFormat is the angular rate data format (DataFormat=0: 16bit output/1: 24bit
output) indicated in Chapter 6 (Table 6.11 Angular rate data read control)
(Note 4) D1 is the angular rate data for the 1st axis gyro (frequency signal H), D2 is the angular rate data
for the 2nd axis gyro (frequency signal J), and D3 is the angular rate data for the 3rd axis gyro
(frequency signal K).
(Note 5) D is the angular rate data for the slave device (any of the 1st through 3rd axis gyro) specified with
the address A[6: 5].
(Note 6) Set the register name OutCtl[1: 0] indicated in Chapter 6 (Table 6.11 Angular rate Data Read
Control) to "01" (angular rate data output).
(Note 7) With global angular rate read, all slave devices are selected. After D1, the angular rate data for the
1st axis gyro, the angular rate data for D2 and D3 are output. Mistakenly specifying a separate
angular rate data format (DataFormat=0: 16bit output/1: 24bit output) for each slave device will
result in a conflict in the output angular rate data. As such, make sure the angular rate data format
setting is the same for each slave device.
Ex: In the case of 1st axis gyro DataFormat=1 (24bit output) and 2nd axis gyro DataFormat=0
(16bit output), the angular rate data output in the 3rd byte will result in a conflict.
(Note 8) Global angular rate read (address A[6: 5]="00") can be repeated in the order of 2nd, 3rd until the
SS logic level is set to "H".
(Note 9) During the global angular rate read (address A[6: 5]="00"), if there is no slave device on a certain
axis, then the angular rate data for that axis is set to "High-Z". To perform the angular rate data
read on an axis with no slave device, use SCLK to input the clock with the SS logic level
maintained at "L" and run an empty angular rate data read for the axis you want to read.
Ex 1: Gyro on 1st axis does not exist > D1[15: 0] or D1[23: 0] is set to "High-Z".
Ex 2: Gyro on 2nd axis does not exist > D2[15: 0] or D2[23: 0] is set to "High-Z".
Ex 3: Gyro on 3rd axis does not exist > D3[15: 0] or D3[23: 0] is set to "High-Z".
Seiko Epson Corporation
26 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

5-6)   Temperature sensor output read
The temperature sensor data read is conducted using the register name TempRd indicated in Chapter 6
(Table 6 User Command Register). Temperature sensor data uses the 2's compliment expression and has a
data width of 8 bit, 10 bit, or 12 bit (switch using the register name TsDataFormat indicated in Table 6.15,
the default value is 12 bit). Similar to the angular rate data read function, continue the serial communication
until the desired bit is read. Temperature sensor data is updated in intervals of approximately 2.4 ms.

Table 5.6 Temperature sensor data output control
Data output order
| TsDataFormat[1]  TsDataFormat[0]  |     |           |                    |
| --------------------------------- | --- | --------- | ------------------ |
|                                   |     | 1 byte    | 2 byte             |
| 0                                 | 0   | D[7: 0]   |                    |
| 0                                 | 1   | D[9: 2]   | D[1: 0] (Note 1)   |
| 1                                 | 0   | D[11: 4]  | D[3: 0] (Note 2)   |
| 1                                 | 1   |           |                    |

(Note 1)  Logic level "L" is output at LSB-side 6 bit.
(Note 2)  Logic level "L" is output at LSB-side 4 bit.

  Seiko Epson Corporation
                     27 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

| 5-7)   Control MISO/SA0  |     |     |     |     |     |     |
| ------------------------ | --- | --- | --- | --- | --- | --- |
MISO/SA0 terminal status as indicated in Table 5.7 can be changed by rewriting register name SelMISO[1:
0] indicated in Section 6-15.

Table 5.7 MISO/SA0 Terminal Control Method
I2C_EN
| Mode  | SPISel  |     | SelMISO[1]  | SelMISO[0]  | SS  | MISO/SA0          |
| ----- | ------- | --- | ----------- | ----------- | --- | ----------------- |
|       | X       | X   | X           | X           | 0   | Output (Note 2)   |
Multi-slave
|     | X            | X                 | X   | X   | 1   | High-Z            |
| --- | ------------ | ----------------- | --- | --- | --- | ----------------- |
|     | 0 (4-wire)   | 0 (I2C Disable)   | X   | X   | 0   | Output            |
|     | 0            | 0                 | 0   | 0   | 1   | Output Level ”L”  |
4-wire SPI
|             | 0            | 0                | 0   | 1   | 1   | Output Level ”H”    |
| ----------- | ------------ | ---------------- | --- | --- | --- | ------------------- |
|             | 0            | 0                | 1   | X   | 1   | High-Z              |
|             | 1 (3-wire)   | X                | 0   | 0   | X   | Output Level ”L”    |
| 3-wire SPI  | 1            | X                | 0   | 1   | X   | Output Level ”H”    |
|             | 1            | X                | 1   | X   | X   | High-Z              |
|             | 0            | 1 (I2C Enable)   | 0   | 0   | 1   | Input               |
| I2C         | 0            | 1                | 0   | 1   | 1   | High-Z (Note 3)     |
|             | 0            | 1                | 1   | 0   | 1   | Input (pull-down)   |
|             | 0            | 1                | 1   | 1   | 1   | Input (pull-up)     |

| (Note 1)  | The default value for SelMISO[1] is "1" and "0" for SelMISO[0].     |     |     |     |     |     |
| --------- | ------------------------------------------------------------------- | --- | --- | --- | --- | --- |
| (Note 2)  | The status is output during data read, High-Z during other times.   |     |     |     |     |     |
| (Note 3)  | SA0 is fixed at "0" within the IC.                                  |     |     |     |     |     |

  Seiko Epson Corporation
                     28 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

6)  User command register

Table 6  User command register
| Address  | Register  | R/W  |     | Function  |
| -------- | --------- | ---- | --- | --------- |
0x00  Reserved
| 0x01  | DspCtl1  | R/W  | DSP Settings 1                 |     |
| ----- | -------- | ---- | ------------------------------ | --- |
| 0x02  | DspCtl2  | R/W  | DSP Settings 2                 |     |
| 0x03  | DspCtl3  | R/W  | DSP Settings 3                 |     |
| 0x04  | StsRd    | R    | Status read                    |     |
| 0x05  | SlpIn    | C    | Sleep-in                       |     |
| 0x06  | SlpOut   | C    | Sleep-out                      |     |
| 0x07  | Stby     | C    | Standby                        |     |
| 0x08  | TempRd   | R    | Temperature sensor data read   |     |
0x09  SWRst  C  Software reset (user command register reset)
| 0x0a  | DatAccOn  | R    | Angular rate data read                |     |
| ----- | --------- | ---- | ------------------------------------- | --- |
| 0x0b  | OutCtl1   | R/W  | Angular rate data read control        |     |
| 0x0c  | AutoC     | C    | Zero rate level calibration command   |     |
| 0x0d  | DspRes    | C    | Digital filter reset                  |     |
0x0e  Reserved
0x0f  Reserved
0x10  Reserved
0x11  Reserved
0x12  Reserved
0x13  Reserved
0x14  Reserved
0x15  Reserved
0x16  Reserved
0x17  Reserved
0x18  Reserved
0x19  Reserved
0x1a  Reserved
| 0x1b  | MemLoad       | C    | Reference data register reset    |     |
| ----- | ------------- | ---- | -------------------------------- | --- |
| 0x1c  | TsDataFormat  | R/W  | Temperature sensor data format   |     |
0x1d  Reserved
0x1e  Reserved
| 0x1f  | IFCtl  | R/W  | Serial interface settings   |     |
| ----- | ------ | ---- | --------------------------- | --- |

| R      | : register read                      |     |     |     |
| ------ | ------------------------------------ | --- | --- | --- |
| R/W    | : register read and register write   |     |     |     |
| C      | : Specify address (command)          |     |     |     |

(Note)  Reserved resister must not be changed. Writing to those registers may cause permanent damage to
the device.

  Seiko Epson Corporation
                     29 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

6-1) DSP Settings 1
Table 6.1 DSP Settings 1
| Address  | bit  Register  | Default  | R/W  | Function  | Settings  |
| -------- | -------------- | -------- | ---- | --------- | --------- |
|          | 7  Reserved    | 0        | R    | Reserved  | Reserved  |
HpfFc[2: 0]
|     | 6  HpfFc[2]  | 0   | R/W  |     | 000: 0.01 Hz  |
| --- | ------------ | --- | ---- | --- | ------------- |
001: 0.03 Hz
010: 0.1 Hz
HPF
|     | 5  HpfFc[1]  | 1   | R/W  |     | 011: 0.3 Hz  |
| --- | ------------ | --- | ---- | --- | ------------ |
fc select
100: 1 Hz
101: 3 Hz
0x01
|     | 4  HpfFc[0]  | 0   | R/W  |     | 110: 10 Hz  |
| --- | ------------ | --- | ---- | --- | ----------- |
111: Not-available
|     | 3  Reserved   | 0   | R   | Reserved   | Reserved   |
| --- | ------------- | --- | --- | ---------- | ---------- |
|     | 2  Reserved   | 0   | R   | Reserved   | Reserved   |
HPF
|     | 1  EnableHpf  | 0   | R/W  |     | 0: Disable, 1: Enable  |
| --- | ------------- | --- | ---- | --- | ---------------------- |
Enable
|           | 0  Reserved                                                        | 1   | R   | Reserved  | Reserved  |
| --------- | ------------------------------------------------------------------ | --- | --- | --------- | --------- |
| (Note 1)  | Perform a reset to change settings other than to change the bit.   |     |     |           |           |
  If the bit is within the same register (same address), then settings can be changed simultaneously.
| (Note 2)  | Reserved resister must not be changed. Use the default settings.   |     |     |     |     |
| --------- | ------------------------------------------------------------------ | --- | --- | --- | --- |

  Seiko Epson Corporation
                     30 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

6-2) DSP Settings 2
Table 6.2 DSP Settings 2
| Address  | bit  Register  | Default  | R/W  Function  | Settings   |
| -------- | -------------- | -------- | -------------- | ---------- |
|          | 7  Reserved    | 0        | R  Reserved    | Reserved   |
|          | 6  Reserved    | 0        | R  Reserved    | Reserved   |
LpfOrder[1: 0]
|     | 5  LpfOrder[1]  | 0   | R/W  |     |
| --- | --------------- | --- | ---- | --- |
00: 2nd order
LPForder
01: 3rd order
select
|     | 4  LpfOrder[0]  | 0   | R/W  | 10: 4th order  |
| --- | --------------- | --- | ---- | -------------- |
11: Not Available
LpfFc[3: 0]
0000: 10Hz
|     | 3  LpfFc[3]  | 0   | R/W  |     |
| --- | ------------ | --- | ---- | --- |
0001: 35Hz
0010: 45Hz
0x02
0011: 50Hz
0100: 70Hz
|     | 2  LpfFc[2]  | 1   | R/W  | 0101: 85Hz  |
| --- | ------------ | --- | ---- | ----------- |
LPF
0110: 100Hz
fc select
0111: 140Hz
1000: 175Hz
|     | 1  LpfFc[1]  | 1   | R/W  |     |
| --- | ------------ | --- | ---- | --- |
1001: 200Hz
1010: 285Hz
1011: 345Hz
1100: 400Hz
|     | 0  LpfFc[0]  | 0   | R/W  |     |
| --- | ------------ | --- | ---- | --- |
1101: 500Hz
| (Note 1)  | Perform a reset to change settings other than to change the bit.   |     |     |     |
| --------- | ------------------------------------------------------------------ | --- | --- | --- |
  If the bit is within the same register (same address), then settings can be changed simultaneously.
| (Note 2)  | Reserved resister must not be changed. Use the default settings.   |     |     |     |
| --------- | ------------------------------------------------------------------ | --- | --- | --- |

  Seiko Epson Corporation
                     31 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

6-3) DSP Settings 3
Table 6.3 DSP Settings 3
| Address  | bit  Register  | Default  | R/W  Function     | Settings               |
| -------- | -------------- | -------- | ----------------- | ---------------------- |
|          | 7  Reserved    | 0        | R  Reserved       | Reserved               |
|          |                |          | Zero rate level   | 0: Disable, 1: Enable  |
|          | 6  EnCalibCmd  | 0        | R/W               |                        |
calibration  (Note 2)
|     | 5  Reserved  | 0   | R  Reserved  | Reserved  |
| --- | ------------ | --- | ------------ | --------- |
|     | 4  Reserved  | 0   | R  Reserved  | Reserved  |
|     | 3  Reserved  | 0   | R  Reserved  | Reserved  |
SelFs[2: 0]  (Note 3)
| 0x03  | 2  SelFs[2]  | 0   | R/W  | 000: fs  |
| ----- | ------------ | --- | ---- | -------- |
001: fs/2
Select angular
010: fs/4
rate output
|     | 1  SelFs[1]  | 0   | R/W  | 011: fs/8  |
| --- | ------------ | --- | ---- | ---------- |
sampling
100: fs/16
rate fds
101: fs/32
|     | 0  SelFs[0]  | 0   | R/W  | 110: fs/64  |
| --- | ------------ | --- | ---- | ----------- |
111: fs/128

(Note 1) Perform a reset to change settings other than to change the bit.
  If the bit is within the same register (same address), then settings can be changed simultaneously.
(Note 2) To carry out zero rate level calibration in Section 6-12, setting should be Enable.
(Note 3) fs is the AD converter sample rate.  It carries out down sampling of the angular rate output.
(Note 4) Reserved resister must not be changed. Use the default settings.

  Seiko Epson Corporation
                     32 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

| 6-4) Status read  |     |     |     |     |     |
| ----------------- | --- | --- | --- | --- | --- |
Table 6.4 Status Read
| Address  | bit  Register  | Default  | R/W  | Function            | Settings                   |
| -------- | -------------- | -------- | ---- | ------------------- | -------------------------- |
|          | 7  Reserved    |          | R    | Reserved            | Reserved                   |
|          | 6  Reserved    |          | R    | Reserved            | Reserved                   |
|          | 5  Reserved    |          | R    | Reserved            | Reserved                   |
|          | 4  Reserved    |          | R    | Reserved            | Reserved                   |
|          |                |          |      | Temperature         | 0: Data output not         |
|          | 3  ProcOK      |          | R    | sensor data output  | available                  |
|          |                |          |      | flag                | 1: Data output available   |
0x04
|     | 2  preStsPOR  |     | R   |     | Bit[2: 0] (Note 1)   |
| --- | ------------- | --- | --- | --- | -------------------- |
100: After turning
power ON
|     | 1  preStsStby  |     | R   |     |     |
| --- | -------------- | --- | --- | --- | --- |
Status flag
010: Stand-by
000: Sleep
|     | 0  preStsSlpOut  |     | R   |     |     |
| --- | ---------------- | --- | --- | --- | --- |
001: Sleep out
| (Note 1)  | Only indicated combinations allowed.   |     |     |     |     |
| --------- | -------------------------------------- | --- | --- | --- | --- |

| 6-5)   Sleep-in  |     |     |     |     |     |
| ---------------- | --- | --- | --- | --- | --- |
Specify address as "0x05". No data read or write.
During sleep, only register access is possible.
Status for angular rate data and temperature sensor data is "0".
Conduct the sleep-out command in Section 6-6 to disable sleep mode.

| 6-6)   Sleep-out  |     |     |     |     |     |
| ----------------- | --- | --- | --- | --- | --- |
Specify address as "0x06". No data read or write.
Returns to normal operations from sleep mode or standby mode.
This resets the DSP.

| 6-7)   Standby   |     |     |     |     |     |
| ---------------- | --- | --- | --- | --- | --- |
Specify address as "0x07". No data read or write.
During standby, the detection circuit is set to disable.
Status for angular rate data and temperature sensor data is "0".
Conduct the sleep-out command in Section 6-6 to disable standby mode.
However, you cannot transition from sleep mode to standby mode.

| 6-8)   Temperature sensor data read  |     |     |     |     |     |
| ------------------------------------ | --- | --- | --- | --- | --- |
Specify address as "0x08". Only data read (no data write).
Refer to Section 5-6(Temperature Sensor Data Read) regarding the temperature sensor data read function.

| 6-9)   Software reset  |     |     |     |     |     |
| ---------------------- | --- | --- | --- | --- | --- |
Specify address as "0x09". No data read or write.
The user command register indicated in Table 6 is reset (set to the default value).

| 6-10)  Angular rate data read  |     |     |     |     |     |
| ------------------------------ | --- | --- | --- | --- | --- |
Specify address as "0x0a". Only data read (no data write).
Refer to Section 5-5(Angular rate Data Read) regarding the angular rate data read function.

  Seiko Epson Corporation
                     33 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

| 6-11)  Angular rate data read control  |     |     |     |     |     |     |
| -------------------------------------- | --- | --- | --- | --- | --- | --- |
Table 6.11 Angular rate data read control
| Address  | bit  | Register    | Default  | R/W  | Function      | Settings         |
| -------- | ---- | ----------- | -------- | ---- | ------------- | ---------------- |
|          | 7    | Reserved    | 0        | R    | Reserved      | Reserved         |
|          | 6    | Reserved    | 0        | R    | Reserved      | Reserved         |
|          | 5    | Reserved    | 0        | R    | Reserved      | Reserved         |
|          | 4    | Reserved    | 0        | R    | Reserved      | Reserved         |
|          | 3    | Reserved    | 0        | R    | Reserved      | Reserved         |
|          |      |             |          |      | Angular rate  | 0: 16bit output  |
| 0x0b     | 2    | DataFormat  | 0        | R/W  |               |                  |
|          |      |             |          |      | data format   | 1: 24bit output  |
OutCtl[1: 0]
|     | 1   | OutCtl[1]  | 0   | R/W  | Angular rate  | 00: Reserved                   |
| --- | --- | ---------- | --- | ---- | ------------- | ------------------------------ |
|     |     |            |     |      | data output   | 01: Angular rate data output   |
|     |     |            |     |      | selection     | 10: Reserved                   |
|     | 0   | OutCtl[0]  | 1   | R/W  |               |                                |
11: Reserved

| (Note 1)  | Perform a reset to change settings other than to change the bit.   |     |     |     |     |     |
| --------- | ------------------------------------------------------------------ | --- | --- | --- | --- | --- |
If the bit is within the same register (same address), then settings can be changed simultaneously.
| (Note 2)  | Reserved resister must not be changed. Use the default settings.   |     |     |     |     |     |
| --------- | ------------------------------------------------------------------ | --- | --- | --- | --- | --- |

| 6-12)  Zero rate level calibration command  |     |     |     |     |     |     |
| ------------------------------------------- | --- | --- | --- | --- | --- | --- |
Specify address as "0x0c". No data read or write.
Zero rate level calibration is carried out at the moment command conducted.
To issue this command, enable register name EnCalibCmd indicated in Table 6.1.4.
To disable the calibration value, disable register name EnCalibCmd indicated in Table 6.1.4.

| 6-13)  Digital filter reset  |     |     |     |     |     |     |
| ---------------------------- | --- | --- | --- | --- | --- | --- |
Specify address as "0x0d". No data read or write.
This command conducts a reset of the digital filter.

| 6-14)  Reference data register reset  |     |     |     |     |     |     |
| ------------------------------------- | --- | --- | --- | --- | --- | --- |
Specify address as "0x1b". No data read or write.
This command loads the reference data register (reference data register: sensitivity settings, sensitivity and
temperature correction value during standby output, etc.) from the non-volatile memory indicated in Chapter
1 (Block diagram) and resets the reference data register. When powered on, the internal sequence circuit
conducts the data load automatically and the reference data register is reset. Separate from the data load at
power on, this command conducts the data load and resets the reference data register based on the timing
specified by the user.

  Seiko Epson Corporation
                     34 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

| 6-15)  | Temperature sensor data format  |     |     |     |     |     |
| ------ | ------------------------------- | --- | --- | --- | --- | --- |
Table 6.15 Temperature sensor data format
| Address  | bit  | Register  | Default  | R/W  | Function  | Settings  |
| -------- | ---- | --------- | -------- | ---- | --------- | --------- |
|          | 7    | Reserved  | 0        | R    | Reserved  | Reserved  |
TsDataFormat[1: 0]
|     | 6   | TsDataFormat[1]  | 1   | R/W  | Temperature  | 00: 8 bit output   |
| --- | --- | ---------------- | --- | ---- | ------------ | ------------------ |
|     |     |                  |     |      | sensor data  | 01: 10 bit output  |
|     |     |                  |     |      | format       | 10: 12 bit output  |
|     | 5   | TsDataFormat[0]  | 0   | R/W  |              |                    |
11: Not Available
0x1c
|     | 4   | Reserved    | 0   | R    | Reserved   | Reserved   |
| --- | --- | ----------- | --- | ---- | ---------- | ---------- |
|     | 3   | SelMISO[1]  | 1   | R/W  | MISO/SA0   |            |
|     |     |             |     |      | terminal   | (Note 3)   |
|     | 2   | SelMISO[0]  | 0   | R/W  |            |            |
status selection
|           | 1                                                                  | Reserved   | 1   | R   | Reserved   | Reserved   |
| --------- | ------------------------------------------------------------------ | ---------- | --- | --- | ---------- | ---------- |
|           | 0                                                                  | Reserved   | 1   | R   | Reserved   | Reserved   |
|           |                                                                    |            |     |     |            |            |
| (Note 1)  | Perform a reset to change settings other than to change the bit.   |            |     |     |            |            |
  If the bit is within the same register (same address), then settings can be changed simultaneously.
| (Note 2)  | Reserved resister must not be changed. Use the default settings.   |     |     |     |     |     |
| --------- | ------------------------------------------------------------------ | --- | --- | --- | --- | --- |
(Note 3)  Refer to Section 5-7(MISO/SA0 Terminal Control Methods) indicated in Chapter 5 for details on
settings.

| 6-16)  | Serial interface settings  |     |     |     |     |     |
| ------ | -------------------------- | --- | --- | --- | --- | --- |
Table 6.16 Serial interface settings
| Address  | bit  | Register          | Default  | R/W  | Function       | Settings        |
| -------- | ---- | ----------------- | -------- | ---- | -------------- | --------------- |
|          | 7    | Reserved          | 0        | R    | Reserved       | Reserved        |
|          |      |                   |          |      | Reserved       | Reserved        |
|          | 6    | Reserved          | 0        | R    |                |                 |
|          | 5    | Reserved          | 0        | R    | Reserved       | Reserved        |
|          |      |                   |          |      | Reserved       | Reserved        |
|          | 4    | Reserved          | 0        | R    |                |                 |
| 0x1f     | 3    | Reserved          | 0        | R    | Reserved       | Reserved        |
|          |      |                   |          |      | Reserved       | Reserved        |
|          | 2    | Reserved          | 0        | R    |                |                 |
|          |      |                   |          |      | 4-wire/3-wire  | 0: 4-wire SPI,  |
|          | 1    | SPISel (Note 3)   | 0        | R/W  |                |                 |
|          |      |                   |          |      | SPI select     | 1: 3-wire SPI   |
0  I2C_EN (Note 3)   1  R/W  I2C enable  0: Disable , 1: Enable

| (Note 1)  | Perform a reset to change settings other than to change the bit.   |     |     |     |     |     |
| --------- | ------------------------------------------------------------------ | --- | --- | --- | --- | --- |
  If the bit is within the same register (same address), then settings can be changed simultaneously.
(Note 2)  The mode indicated in Table 6.16 can be changed only when using SPI communication.
(Note 3)  Register settings are not set when the multi-slave function is enabled via factory settings.
| (Note 4)  | Reserved resister must not be changed. Use the default settings.  |     |     |     |     |     |
| --------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- |

  Seiko Epson Corporation
                     35 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

7)  Connection diagram

VDDI VDDM
|     |     |             |     |     |         | VDDI |      | VDDM      |     |         |             |
| --- | --- | ----------- | --- | --- | ------- | ---- | ---- | --------- | --- | ------- | ----------- |
|     |     | SS          |     |     |         | SS   |      | Reserved2 |     | N.C.    |             |
| MPU |     | MISO        |     |     |         | MISO | Gyro |           | GND |         |             |
|     |     | MOSI        |     |     |         | MOSI |      | Reserved1 |     |         |             |
|     |     | SCLK        |     |     |         | SCLK |      | VDDL      |     |         |             |
|     |     | C3          |     |     | C4      |      |      |           | C5  | C2      | C1          |
|     |     | 0.01u～0.1uF |     |     | 1u～10uF |      |      |           | 1uF | 1u～10uF | 0.01u～0.1uF |

Fig. 7.1  Example of 4-wire SPI connection

VDDI VDDM
|     |     |             |     |     |         | VDDI |      | VDDM      |     |         |             |
| --- | --- | ----------- | --- | --- | ------- | ---- | ---- | --------- | --- | ------- | ----------- |
|     |     | SS          |     |     |         | SS   |      | Reserved2 |     | N.C.    |             |
| MPU |     |             |     |     | N.C.    | MISO | Gyro |           | GND |         |             |
|     |     | MOSI        |     |     |         | MOSI |      | Reserved1 |     |         |             |
|     |     | SCLK        |     |     |         | SCLK |      | VDDL      |     |         |             |
|     |     | C3          |     |     | C4      |      |      |           | C5  | C2      | C1          |
|     |     | 0.01u～0.1uF |     |     | 1u～10uF |      |      |           | 1uF | 1u～10uF | 0.01u～0.1uF |

Fig. 7.2  Example of 3-wire SPI connection

VDDI VDDM
|     |     |             |         |         |                    |     | VDDI |      | VDDM      |             |             |
| --- | --- | ----------- | ------- | ------- | ------------------ | --- | ---- | ---- | --------- | ----------- | ----------- |
|     |     |             |         |         |                    |     | SS   |      | Reserved2 | N.C         |             |
| MPU |     |             | R       | S A 0 = | 0 ： N .C .  or GND |     | SA0  | Gyro | VSS       |             |             |
|     |     |             | 1k～10kΩ | S A 0 = | 1 ： VD D I         |     |      |      |           |             |             |
|     | SDA |             |         |         |                    |     | SDA  |      | Reserved1 |             |             |
|     | SCL |             |         |         |                    |     | SCL  |      | VDDL      |             |             |
|     |     | C3          |         |         | C4                 |     |      |      |           | C5 C2       | C1          |
|     |     | 0.01u～0.1uF |         | 1u～10uF |                    |     |      |      |           | 1uF 1u～10uF | 0.01u～0.1uF |

Fig. 7.3  Example of I2C connection

  Seiko Epson Corporation
                     36 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

VDDI VDDM
|      |             | VDDI    | VDDM           |             |
| ---- | ----------- | ------- | -------------- | ----------- |
| SS   |             | SS      | Reserved2 N.C. |             |
| MPU  |             | Gyro-A  |                |             |
| MISO |             | MISO    | GND            |             |
| MOSI |             | MOSI    | Reserved1      |             |
| SCLK |             | SCLK    | VDDL           |             |
|      | C3          | C4      | C5 C2          | C1          |
|      | 0.01u～0.1uF | 1u～10uF | 1uF 1u～10uF    | 0.01u～0.1uF |
VDDI VDDM
|     |     | VDDI | VDDM           |     |
| --- | --- | ---- | -------------- | --- |
|     |     | SS   | Reserved2 N.C. |     |
Gyro-B
|     |             | MISO    | GND         |             |
| --- | ----------- | ------- | ----------- | ----------- |
|     |             | MOSI    | Reserved1   |             |
|     |             | SCLK    | VDDL        |             |
|     | C3          | C4      | C5 C2       | C1          |
|     | 0.01u～0.1uF | 1u～10uF | 1uF 1u～10uF | 0.01u～0.1uF |
VDDI VDDM
|     |     | VDDI | VDDM           |     |
| --- | --- | ---- | -------------- | --- |
|     |     | SS   | Reserved2 N.C. |     |
Gyro-C
|     |             | MISO    | GND         |             |
| --- | ----------- | ------- | ----------- | ----------- |
|     |             | MOSI    | Reserved1   |             |
|     |             | SCLK    | VDDL        |             |
|     | C3          | C4      | C5 C2       | C1          |
|     | 0.01u～0.1uF | 1u～10uF | 1uF 1u～10uF | 0.01u～0.1uF |

Fig. 7.4  Example of Multi-slave connection

  Seiko Epson Corporation
                     37 / 40  Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

8) Soldering profile
Solder heat resistance verified under the air reflow furnace conditions below.
Temperature [ (cid:176) C ]
300
TP ; +260℃
+255℃ tp ; 30 s Max
250
Avg. Ramp-up
200
T
Ts
L
max
;
;
+
+
2
2
1
0
7
0
(cid:176)
(cid:176)
C
C
3 (cid:176) C / s Max.
(
6
+
0
2
s
17
t
t
(cid:176)
o
L
C
1
o
5
ve
0
r )
s 6
R
(cid:176)
a
C
m
/
p
s
- d
M
ow
a
n
x.
Ts min ; +150 (cid:176) C ts
150
60 s to 120 s
( +150 (cid:176)C to +200 (cid:176)C )
100
50
Time +25 (cid:176) C to Peak
480s Max.
0 60 120 180 240 300 360 420 480 540 600 660 720 780
Time [ s ]
9) Terminology
9-1) Cross axis sensitivity
The value is derived by dividing sensitivity around the X and Y axis by the sensitivity around the Z axis. The X, Y,
and Z axis directions are as shown in the following diagram.
+
＋
Z a
Ｚ
x
軸
is
+ ＋ Y Ｙ ax 軸 is ＋+XＸ a軸xis
#1端子マーク
#1 Terminal mark
9-2) Drive frequency and detuning frequency
The drive frequency is the resonance frequency (drive mode) of the sensor element continuously vibrated to gain
the Coriolis force. The detuning frequency is the natural frequency used for the mechanical-electrical transduction
of the Coriolis force. It is the difference from the drive frequency.
Seiko Epson Corporation
38 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

10) Handling precaution
Crystal devices are high precision products. Use the following precautions during handling.
1. The detuning frequency for this product is 900 Hz ±200 Hz. During board design, the customer must ensure
that the board resonance frequency is not within the vicinity of this detuning frequency. When mounting on a
board, align the sensor near a board loading component with low resonance variation.
2. Excessive shock from adsorption / chucking when mounting the sensor or excessive vibration or shock during
board cutting or an impact wrench after mounting can result in damage to the sensor or the deterioration of
sensor properties. Establish conditions that avoid vibration or shock to the sensor to ensure there is not loss in
performance.
3. To detect angular rate, this product uses a drive frequency to drive the sensor element. External application of a
signal with frequency components in the vicinity of the drive frequency or high-order harmonics can result in
fluctuations in angular rate output by the sensor. Be sure to confirm internally in advance concerning power
supply decoupling measures and serial interface communications frequency settings.
4. To prevent malfunctions caused by electromagnetic induction and static induction from other signal lines,
during pattern design do not pass other signal lines near the sensor or along the back of the package. Also, use
a pattern design that does not cross with other signal lines.
5. Confirm internally in advance concerning measures for vibration, shock, and noise. We will provide design
support if you provide us with board design information.
6. This product design incorporates shock resistance but there is the risk of product damage due to drops and
shock. Do not use this product if it has been dropped as we cannot guarantee product performance.
7. Applying ultrasonic vibration during ultrasonic washing can cause resonance damage to the crystal unit
depending on usage conditions. As we cannot specify the usage conditions (washer type, power, time, tub
position, etc.) at your company, we offer no guarantees concerning operability after the application of
ultrasonic vibration. Confirm internally prior to use if the use of ultrasonic washing is required.
8. Prior to use, conduct mounting tests internally to confirm there is no impact on performance. Similarly,
confirm prior to changing any conditions. During and after mounting, ensure that the sensor is not in contact
with boards or structural elements.
9. The sensor includes a static electricity protection circuit but application of significant static electricity can
result in damage to the sensor's internal IC. Make sure to use conductive materials for packaging and transport
containers as well. For the soldering iron, measurement circuit, etc., use products with no high-voltage leaks
and during mounting make sure to employ static electricity measures such as the use of a ground wire.
10. Keep reflow to no more than three times. Use a soldering iron to correct any soldering mistakes. Here, the
temperature of the iron type should be below +350 °C and less than 3 seconds. (Blower use not allowed)
11. We recommend using board production based on Epson pad dimensions.
12. This product has the same frequency noise as drive frequency. Remove using an appropriate filter circuits.
13. This product is designed to resist acoustic interference even when multiple sensors are operated in close
proximity but impedance common to board resonance and power supply could result in mechanical or
electrical interference. Confirm internally prior to use.
14. This product includes a POR circuit. To avoid POR circuit malfunctions, power supply voltage rise should be
conducted between 0.01 ms and 100 ms.
15. Do not use in high condensation or other environments prone to short circuits between terminals.
Seiko Epson Corporation
39 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015

11) Contact
America
Epson Electronics America, Inc.
214 Devcon Drive, San Jose, CA 95112, USA
Phone: +1-800-228-3964 FAX: +1-408-922-0238
Europe
Epson Europe Electronics GmbH
Riesstrasse 15, 80992 Munich, GERMANY
Phone: +49-89-14005-0 FAX: +49-89-14005-110
ASIA
Epson (China) Co., Ltd.
7F, Jinbao Bldg., No.89 Jinbao Street Dongcheng District, Beijing, China, 100005
Phone: +86-10-8522-1199 FAX: +86-10-8522-1120
Shanghai Branch
High-Tech Building, 900 Yishan Road, Shanghai 200233, China
Phone: +86-21-5423-5577 FAX: +86-21-5423-4677
Shenzhen Branch
12/F, Dawning Mansion, #12 Keji South Road, Hi-Tech Park, Shenzhen, China
Phone: +86-755-2699-3828 FAX: +86-755-2699-3838
Epson Hong Kong Ltd.
Unit 715-723 7/F, Trade Square, 681 Cheung Sha Wan Road, Kowloon, Hong Kong
Phone: (86) 755-26993828 (Shenzhen Branch(cid:15911)
FAX: (86) 755-26993838 (Shenzhen Branch(cid:15911)
Epson Taiwan Technology & Trading Ltd.
14F, No. 7, Song Ren Road, Taipei 110, TAIWAN
Phone: +886-2-8786-6688 FAX: +886-2-8786-6660
Epson Singapore Pte., Ltd.
No 1 HarbourFront Place, #03-02 HarbourFront Tower One, Singapore 098633
Phone: +65-6586-5500 FAX: +65-6271-3182
Seiko Epson Corp. Korea Office
5F (63Bldg.,Yoido-dong) 50, 63-ro, Yeongdeungpo-gu, Seoul, 150-763, Korea
Phone: +82-2-784-6027 FAX: +82-2-767-3677
Seiko Epson Corporation
40 / 40 Document No.: XV7001BB_AE_Ver. 1.00
Date: Mar. 19th 2015