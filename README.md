# ADXL345 accelerometer reader for Raspberry Pi

This simple command line tool provides an easy to use and reliable **non-realtime** access to ADXL345 three-axis digital accelerometer ([datasheet][adxl_manual]) over SPI interface of Raspberry Pi at sampling rates up to 3200 Hz. The output can be redirected to a standard output or CSV file.

## Usage

**Executable binary: [adxl345spi][adxl345spi_binary]**

```
Usage: adxl345spi [OPTION]...

Mandatory arguments to long options are mandatory for short options too.
  -s, --save FILE     save data to specified FILE (data printed to command-line
                      output, if not specified)
  -t, --time TIME     set the duration of data stream to TIME seconds
                      (default: 5 seconds) [integer]
  -f, --freq FREQ     set the sampling rate of data stream to FREQ samples per
                      second, 1 <= FREQ <= 3200 (default: 5 Hz) [integer]

Data is streamed in comma separated format, e. g.:
  time,     x,     y,     z
   0.0,  10.0,   0.0, -10.0
   1.0,   5.0,  -5.0,  10.0
   ...,   ...,   ...,   ...
  time shows seconds elapsed since the first reading;
  x, y and z show acceleration along x, y and z axis in fractions of <g>.

Exit status:
  0  if OK
  1  if error occurred during data reading or wrong cmdline arguments.
```

## Wiring

Fritzing diagrams for two popular revisions:

![adxl345spi_bb][adxl345spi_bb_png]

![adxl345spi_schem][adxl345spi_schem_png]

```
CS  : SPI_CE0_N (pin 24)
SDO : SPI_MISO (pin 21)
SDA : SPI_MOSI (pin 19)
SCL : SPI_CLK (pin 23)
```

Some ADXL345 breadboards have `VCC` pin marked as `3V3`.

## Build

### Dependencies

This utility is built using `pigpio` library, which provides a `C` interface to the General Purpose Input Outputs (GPIO) of Raspberry Pi. More information on provided `C` interface can be found [here][pigpio_info_C].

In order to compile `adxl345spi.c` file, install `pigpio` library first. The procedure can be shortly described as follows:

```
wget https://abyz.me.uk/rpi/pigpio/pigpio.zip
unzip pigpio.zip
cd PIGPIO
make -j4
sudo make install
```

More information can be found on the [download page][pigpio_download].

### Compilation

`gcc -Wall -pthread -o adxl345spi adxl345spi.c -lpigpio -lrt`

**Build is only tested on Raspbian OS.**

## Code features

Due to limitations of I2C bus, it is impossible to achieve high sampling rates using I2C interface. SPI interface is used to achieve sampling rates up to 3200 samples per second (upper limit of ADXL345 chip itself). SPI baud rate is set to 2 Mbps, to ensure that no readings are lost during transmission.

For a standard output, downsampling is achieved by calling `sleep()` between transmissions, providing someway unstable sampling rate. If a reliable sampling rate is required, output to a file should be used.

For a file output, data is always read at a higher than required sampling rate (~30000 Hz for 2 Mbps SPI baud rate) and accurately downsampled to a specified value. Applications that require stable sampling rates and reliable time steps (e.g. spectrum analysis via FFT) should use a file output option.

[adxl345spi_binary]: https://github.com/nagimov/adxl345spi/raw/master/adxl345spi
[adxl_manual]: http://www.analog.com/en/products/mems/accelerometers/adxl345.html
[pigpio_info_C]: https://abyz.me.uk/rpi/pigpio/cif.html
[pigpio_download]: https://abyz.me.uk/rpi/pigpio/download.html
[adxl345spi_bb_png]: adxl345spi_bb.png
[adxl345spi_schem_png]: adxl345spi_schem.png
