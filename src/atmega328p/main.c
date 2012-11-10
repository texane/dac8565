#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/* SPI module */

static inline void spi_setup_master(void)
{
  /* doc8161.pdf, ch.18 */

  /* ss is used by avr spi to determine master */
  /* set output mode even if pb2 not used by us */
  DDRB |= (1 << 2);

  /* spi output pins: sck pb5, mosi pb3 */
  DDRB |= (1 << 5) | (1 << 3);

  /* spi input pins: miso pb4 */
  DDRB &= ~(1 << 4);
  /* disable pullup (already by default) */
  PORTB &= ~(1 << 4);

  /* enable spi, msb first, master, freq / 128 (125khz), sck low idle */
  SPCR = (1 << SPE) | (1 << MSTR) | (3 << SPR0) | (1 << CPOL);

  /* clear double speed */
  SPSR &= ~(1 << SPI2X);
}

static inline void spi_set_sck_freq(uint8_t x)
{
  /* x one of SPI_SCK_FREQ_FOSCX */
  /* where spi sck = fosc / X */
  /* see atmega328 specs, table 18.5 */
#define SPI_SCK_FREQ_FOSC2 ((1 << 2) | 0)
#define SPI_SCK_FREQ_FOSC4 ((0 << 2) | 0)
#define SPI_SCK_FREQ_FOSC8 ((1 << 2) | 1)
#define SPI_SCK_FREQ_FOSC16 ((0 << 2) | 1)
#define SPI_SCK_FREQ_FOSC32 ((1 << 2) | 2)
#define SPI_SCK_FREQ_FOSC64 ((0 << 2) | 2)
#define SPI_SCK_FREQ_FOSC128 ((0 << 2) | 3)

  SPCR &= ~(3 << SPR0);
  SPCR |= (x & 3) << SPR0;

  SPSR &= ~(1 << SPI2X);
  SPSR |= (((x >> 2) & 1) << SPI2X);
}

static inline void spi_write_uint8(uint8_t x)
{
  /* write the byte and wait for transmission */

 redo:
#if 1 /* FIXME: needed for sd_read_block to work */
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
#endif

  SPDR = x;

  if (SPSR & (1 << WCOL))
  {
    /* access SPDR */
    volatile uint8_t fubar = SPDR;
    __asm__ __volatile__ ("" :"=m"(fubar));
    goto redo;
  }

  while ((SPSR & (1 << SPIF)) == 0) ;
}

static void spi_write_uint16(uint16_t x)
{
  spi_write_uint8((x >> 8) & 0xff);
  spi_write_uint8((x >> 0) & 0xff);
}


/* reference: dac8565.pdf */

static inline void dac8565_sync_low(void)
{
#define DAC8565_SYNC_MASK (1 << 0)
  PORTB &= ~DAC8565_SYNC_MASK;
}

static inline void dac8565_sync_high(void)
{
  PORTB |= DAC8565_SYNC_MASK;
}

static inline void dac8565_sck_delay(void)
{
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
}

static void dac8565_write_cmd(uint8_t cmd, uint16_t data)
{
  /* pulse sync */
  dac8565_sync_high();
  dac8565_sck_delay();
  dac8565_sync_low();

  /* wait for SCLK falling edge setup time, 4ns */
  dac8565_sck_delay();

  spi_write_uint8(cmd);
  spi_write_uint16(data);

  /* wait for SLCK to rise (0 ns) and set high */
  dac8565_sck_delay();
  dac8565_sync_high();
}

static inline void dac8565_write(uint8_t chan, uint16_t data)
{
  /* assume: 0 <= chan <= 3 */
  /* 2 to update both input and DAC registers */  
  const uint8_t cmd = (1 << 4) | (chan << 1);
  dac8565_write_cmd(cmd, data);
}

static void dac8565_setup(void)
{
  /* assume spi initialzed */

  DDRB |= DAC8565_SYNC_MASK;
  dac8565_sync_high();

#if 0
  /* reset page 6 */
  DDRD |= 1 << 2;
  PORTD |= 1 << 2;
  dac8565_sck_delay();
  dac8565_sck_delay();
  PORTD &= ~(1 << 2);
  dac8565_sck_delay();
  dac8565_sck_delay();
  dac8565_sck_delay();
  dac8565_sck_delay();
  PORTD |= 1 << 2;
#endif

  /* refer to page 33 */
  /* internal reference to default mode */
  /* power down all DACs to high impedance */

  dac8565_write_cmd(0x01, 0x0000);
  dac8565_write_cmd(0x35, 0xffff);
}

/* main */

int main(void)
{
  spi_setup_master();
  spi_set_sck_freq(SPI_SCK_FREQ_FOSC2);

  dac8565_setup();

  dac8565_write(0, 0xffff);
  while (1) ;

  while (1)
  {
    dac8565_write(0, 0x0000);
    dac8565_write(0, 0xffff);
  }

  return 0;
}
