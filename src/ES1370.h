#ifdef HAVE_SDL
#include "StdAfx.h"
#include "System.h"
#include "SystemComponent.h"
#include "PCIDevice.h"

#include <SDL3/SDL.h>
/* Start blatant GPL violation */

#define ES1370_REG_CONTROL        0x00
#define ES1370_REG_STATUS         0x04
#define ES1370_REG_UART_DATA      0x08
#define ES1370_REG_UART_STATUS    0x09
#define ES1370_REG_UART_CONTROL   0x09
#define ES1370_REG_UART_TEST      0x0a
#define ES1370_REG_MEMPAGE        0x0c
#define ES1370_REG_CODEC          0x10
#define ES1370_REG_SERIAL_CONTROL 0x20
#define ES1370_REG_DAC1_SCOUNT    0x24
#define ES1370_REG_DAC2_SCOUNT    0x28
#define ES1370_REG_ADC_SCOUNT     0x2c

#define ES1370_REG_DAC1_FRAMEADR    0xc30
#define ES1370_REG_DAC1_FRAMECNT    0xc34
#define ES1370_REG_DAC2_FRAMEADR    0xc38
#define ES1370_REG_DAC2_FRAMECNT    0xc3c
#define ES1370_REG_ADC_FRAMEADR     0xd30
#define ES1370_REG_ADC_FRAMECNT     0xd34
#define ES1370_REG_PHANTOM_FRAMEADR 0xd38
#define ES1370_REG_PHANTOM_FRAMECNT 0xd3c

static const unsigned dac1_samplerate[] = { 5512, 11025, 22050, 44100 };

#define DAC2_SRTODIV(x) (((1411200+(x)/2)/(x))-2)
#define DAC2_DIVTOSR(x) (1411200/((x)+2))

#define CTRL_ADC_STOP   0x80000000  /* 1 = ADC stopped */
#define CTRL_XCTL1      0x40000000  /* electret mic bias */
#define CTRL_OPEN       0x20000000  /* no function, can be read and written */
#define CTRL_PCLKDIV    0x1fff0000  /* ADC/DAC2 clock divider */
#define CTRL_SH_PCLKDIV 16
#define CTRL_MSFMTSEL   0x00008000  /* MPEG serial data fmt: 0 = Sony, 1 = I2S */
#define CTRL_M_SBB      0x00004000  /* DAC2 clock: 0 = PCLKDIV, 1 = MPEG */
#define CTRL_WTSRSEL    0x00003000  /* DAC1 clock freq: 0=5512, 1=11025, 2=22050, 3=44100 */
#define CTRL_SH_WTSRSEL 12
#define CTRL_DAC_SYNC   0x00000800  /* 1 = DAC2 runs off DAC1 clock */
#define CTRL_CCB_INTRM  0x00000400  /* 1 = CCB "voice" ints enabled */
#define CTRL_M_CB       0x00000200  /* recording source: 0 = ADC, 1 = MPEG */
#define CTRL_XCTL0      0x00000100  /* 0 = Line in, 1 = Line out */
#define CTRL_BREQ       0x00000080  /* 1 = test mode (internal mem test) */
#define CTRL_DAC1_EN    0x00000040  /* enable DAC1 */
#define CTRL_DAC2_EN    0x00000020  /* enable DAC2 */
#define CTRL_ADC_EN     0x00000010  /* enable ADC */
#define CTRL_UART_EN    0x00000008  /* enable MIDI uart */
#define CTRL_JYSTK_EN   0x00000004  /* enable Joystick port (presumably at address 0x200) */
#define CTRL_CDC_EN     0x00000002  /* enable serial (CODEC) interface */
#define CTRL_SERR_DIS   0x00000001  /* 1 = disable PCI SERR signal */

#define STAT_INTR       0x80000000  /* wired or of all interrupt bits */
#define STAT_CSTAT      0x00000400  /* 1 = codec busy or codec write in progress */
#define STAT_CBUSY      0x00000200  /* 1 = codec busy */
#define STAT_CWRIP      0x00000100  /* 1 = codec write in progress */
#define STAT_VC         0x00000060  /* CCB int source, 0=DAC1, 1=DAC2, 2=ADC, 3=undef */
#define STAT_SH_VC      5
#define STAT_MCCB       0x00000010  /* CCB int pending */
#define STAT_UART       0x00000008  /* UART int pending */
#define STAT_DAC1       0x00000004  /* DAC1 int pending */
#define STAT_DAC2       0x00000002  /* DAC2 int pending */
#define STAT_ADC        0x00000001  /* ADC int pending */

#define USTAT_RXINT     0x80        /* UART rx int pending */
#define USTAT_TXINT     0x04        /* UART tx int pending */
#define USTAT_TXRDY     0x02        /* UART tx ready */
#define USTAT_RXRDY     0x01        /* UART rx ready */

#define UCTRL_RXINTEN   0x80        /* 1 = enable RX ints */
#define UCTRL_TXINTEN   0x60        /* TX int enable field mask */
#define UCTRL_ENA_TXINT 0x20        /* enable TX int */
#define UCTRL_CNTRL     0x03        /* control field */
#define UCTRL_CNTRL_SWR 0x03        /* software reset command */

#define SCTRL_P2ENDINC    0x00380000  /*  */
#define SCTRL_SH_P2ENDINC 19
#define SCTRL_P2STINC     0x00070000  /*  */
#define SCTRL_SH_P2STINC  16
#define SCTRL_R1LOOPSEL   0x00008000  /* 0 = loop mode */
#define SCTRL_P2LOOPSEL   0x00004000  /* 0 = loop mode */
#define SCTRL_P1LOOPSEL   0x00002000  /* 0 = loop mode */
#define SCTRL_P2PAUSE     0x00001000  /* 1 = pause mode */
#define SCTRL_P1PAUSE     0x00000800  /* 1 = pause mode */
#define SCTRL_R1INTEN     0x00000400  /* enable interrupt */
#define SCTRL_P2INTEN     0x00000200  /* enable interrupt */
#define SCTRL_P1INTEN     0x00000100  /* enable interrupt */
#define SCTRL_P1SCTRLD    0x00000080  /* reload sample count register for DAC1 */
#define SCTRL_P2DACSEN    0x00000040  /* 1 = DAC2 play back last sample when disabled */
#define SCTRL_R1SEB       0x00000020  /* 1 = 16bit */
#define SCTRL_R1SMB       0x00000010  /* 1 = stereo */
#define SCTRL_R1FMT       0x00000030  /* format mask */
#define SCTRL_SH_R1FMT    4
#define SCTRL_P2SEB       0x00000008  /* 1 = 16bit */
#define SCTRL_P2SMB       0x00000004  /* 1 = stereo */
#define SCTRL_P2FMT       0x0000000c  /* format mask */
#define SCTRL_SH_P2FMT    2
#define SCTRL_P1SEB       0x00000002  /* 1 = 16bit */
#define SCTRL_P1SMB       0x00000001  /* 1 = stereo */
#define SCTRL_P1FMT       0x00000003  /* format mask */
#define SCTRL_SH_P1FMT    0

/* End blatant GPL violation */
#define NB_CHANNELS 3
#define DAC1_CHANNEL 0
#define DAC2_CHANNEL 1
#define ADC_CHANNEL 2


class CES1370 : public CPCIDevice
{
public:
  virtual int   SaveState(FILE* f) { return 0;  }
  virtual int   RestoreState(FILE* f) { return 0;  }
  virtual void  check_state() {}
  virtual void  init();

  virtual void  WriteMem_Bar(int func, int bar, u32 address, int dsize, u32 data);
  virtual u32   ReadMem_Bar(int func, int bar, u32 address, int dsize);

  CES1370(CConfigurator* cfg, class CSystem* c, int pcibus, int pcidev);
  virtual ~CES1370();

private:
  struct chan {
    uint32_t shift;
    uint32_t leftover;
    uint32_t scount;
    uint32_t frame_addr;
    uint32_t frame_cnt;
  };

  struct ES1370State {
    SDL_AudioDeviceID audio_be_out;
    SDL_AudioDeviceID audio_be_in;
    struct chan chan[3];
    SDL_AudioStream* dac_voice[2];
    SDL_AudioStream* adc_voice;

    uint32_t ctl;
    uint32_t status;
    uint32_t mempage;
    uint32_t codec;
    uint32_t sctl;
  } state;

  struct chan_bits {
    uint32_t ctl_en;
    uint32_t stat_int;
    uint32_t sctl_pause;
    uint32_t sctl_inten;
    uint32_t sctl_fmt;
    uint32_t sctl_sh_fmt;
    uint32_t sctl_loopsel;
    void (*calc_freq) (ES1370State* s, uint32_t ctl,
      uint32_t* old_freq, uint32_t* new_freq);
  };

  static void es1370_dac1_calc_freq(ES1370State* s, uint32_t ctl,
    uint32_t* old_freq, uint32_t* new_freq);

  static void es1370_dac2_and_adc_calc_freq(ES1370State* s, uint32_t ctl,
    uint32_t* old_freq,
    uint32_t* new_freq);

  struct chan_bits es1370_chan_bits[3] = {
    {CTRL_DAC1_EN, STAT_DAC1, SCTRL_P1PAUSE, SCTRL_P1INTEN,
     SCTRL_P1FMT, SCTRL_SH_P1FMT, SCTRL_P1LOOPSEL,
     es1370_dac1_calc_freq},

    {CTRL_DAC2_EN, STAT_DAC2, SCTRL_P2PAUSE, SCTRL_P2INTEN,
     SCTRL_P2FMT, SCTRL_SH_P2FMT, SCTRL_P2LOOPSEL,
     es1370_dac2_and_adc_calc_freq},

    {CTRL_ADC_EN, STAT_ADC, 0, SCTRL_R1INTEN,
     SCTRL_R1FMT, SCTRL_SH_R1FMT, SCTRL_R1LOOPSEL,
     es1370_dac2_and_adc_calc_freq}
  };
  void es1370_update_status(ES1370State* s, uint32_t new_status);
  void es1370_reset(ES1370State* s);
  void es1370_maybe_lower_irq(ES1370State* s, uint32_t sctl);
  void es1370_update_voices(ES1370State* s, uint32_t ctl, uint32_t sctl);
  uint32_t es1370_fixup(ES1370State* s, uint32_t addr);
  void es1370_write(void* opaque, u64 addr, uint64_t val, unsigned size);
  uint64_t es1370_read(void* opaque, u64 addr, unsigned size);
  void es1370_transfer_audio(ES1370State* s, struct chan* d, int loop_sel, int max, bool* irq);
  void es1370_run_channel(ES1370State* s, size_t chan, int free_or_avail);

  static void es1370_dac_callback_dac1(void* userdata, SDL_AudioStream* stream, int additional_amount, int total_amount);
  static void es1370_dac_callback_dac2(void* userdata, SDL_AudioStream* stream, int additional_amount, int total_amount);
  static void es1370_dac_callback_adc(void* userdata, SDL_AudioStream* stream, int additional_amount, int total_amount);
};
#endif /* HAVE_SDL */
