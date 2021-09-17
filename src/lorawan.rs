use core::convert::TryInto;

use lorawan_device::{Timings, radio::{Bandwidth as LoraBandwidth, CodingRate as LoraCodingRate, Error as LoraError, Event as LoraEvent, PhyRxTx, Response as LoraResponse, RxQuality, SpreadingFactor as LoraSpreadingFactor}};
use stm32wl_hal::{spi::{SgMiso, SgMosi}, subghz::*};

use crate::rfswitch::RfSwitch;

const SUBGHZ_HP_MAX: i8 = 22;
const SUBGHZ_LP_MAX: i8 = 15;

#[derive(Debug)]
pub enum State {
    Idle,
    Txing,
    Rxing,
}

#[derive(Debug)]
pub struct LorawanRadio {
    subghz: SubGhz<SgMiso, SgMosi>,
    state: State,
    buffer_rx: [u8; 256],
    buffer_len: u8,
    rfs: RfSwitch,
    high_power: bool,
}
#[derive(Debug)]
pub enum Response {
    Busy,
    TxDone(u32),
    RxDone(u32, RxQuality),
    TxTimeout,
    RxTimeout,
    RxError,
}

#[derive(Debug)]
pub enum Event {
    Irq(Status, u16),
}

#[derive(Debug)]
pub enum Error {
    UnexpectedIrq(u16),
    UnexpectedPhyEvent,
    Timeout,
    PowerHPTooHigh,
    PowerLPTooHigh,
}

impl LorawanRadio {
    /// Creates a new LoraWan device and configures the minimum needed settings:
    /// * CRC Enabled
    /// * Variable header
    /// * 8 byte preamble length
    /// * Packet type: LoRa
    ///
    /// It is up to the user to set the other paremeters (e.g. public/private network) using the
    /// `set_lora_packet_params` method.
    /// 
    /// The `rfs` parameter is an RF Switch to change between RX and TX that must be implemented
    /// on the board. For now, this is the type of the `bsp` for the NUCLEO board
    ///
    /// TODO: Make RfSwitch a generic with a trait
    ///
    /// The `high_power` boolean controls which transmitter is used by default. It can be changed
    /// using the `set_hp` method. The reason for having this setting instead of dynamically
    /// choosing the transmitter based on the requested power, is that some boards only
    /// implement one of the paths (e.g. only high power).
    pub fn new(mut subghz: SubGhz<SgMiso, SgMosi>, rfs: RfSwitch, high_power: bool) -> Self {
        radio_config(&mut subghz, high_power);
        
        LorawanRadio {
            subghz,
            state: State::Idle,
            buffer_rx: [0; 256],
            buffer_len: 0,
            rfs,
            high_power,
        }
    }

    /// Give back the SubGHz peripheral and the RF Switch
    pub fn free(self) -> (SubGhz<SgMiso, SgMosi>, RfSwitch) {
        (self.subghz, self.rfs)
    }

    /// Set whether the preferred transmitter is High Power or not (Low Power)
    pub fn set_hp(&mut self, hp: bool) {
        self.high_power = hp;
    }

    /// Whether the preferred transmitter is High Power
    pub fn is_hp_set(&self) -> bool {
        self.high_power
    }

    fn handle_event_from_idle(&mut self, event: LoraEvent<Self>
    ) -> (State, Result<LoraResponse<Self>, LoraError<Self>>) {
        match event {
            LoraEvent::TxRequest(config, buf) => {
                defmt::info!("LoraEvent::TxRequest: {:X}", buf);
                defmt::info!("TxRequest: {}, power: {}", config.rf, config.pw);

                radio_config(&mut self.subghz, self.high_power);
                let result = {
                    self.subghz.set_rf_frequency(&RfFreq::from_frequency(433_175_000)).unwrap();//config.rf.frequency)).unwrap();
                    
                    self.subghz.set_lora_mod_params(
                        &LoRaModParams::new()
                        .set_sf(sf_transform(config.rf.spreading_factor))
                        .set_cr(cr_transform(config.rf.coding_rate))
                        .set_bw(bw_transform(config.rf.bandwidth))
                        .set_ldro_en(true)).unwrap();

                    self.subghz.set_lora_packet_params(
                        &LoRaPacketParams::new()
                        .set_preamble_len(8)
                        .set_header_type(HeaderType::Variable)
                        .set_payload_len(buf.len() as u8)
                        .set_crc_en(true)
                        .set_invert_iq(false)
                    )
                    
                };

                match result {
                    Ok(_) => {
                        if self.high_power {
                            if config.pw > SUBGHZ_HP_MAX {
                                return (State::Idle, Err(LoraError::PhyError(Error::PowerHPTooHigh)))
                            }
                            //self.rfs.set_tx_hp();
                        } else {
                            if config.pw > SUBGHZ_LP_MAX {
                                return (State::Idle, Err(LoraError::PhyError(Error::PowerLPTooHigh)))
                            }
                            //self.rfs.set_tx_lp();
                        }
                        const IRQ_CFG: CfgIrq = CfgIrq::new()
                            .irq_enable_all(Irq::TxDone)
                            .irq_enable_all(Irq::RxDone)
                            .irq_enable_all(Irq::Timeout);
                        self.subghz.set_irq_cfg(&IRQ_CFG).unwrap();
                        self.subghz.set_buffer_base_address(0,0).unwrap();
                        self.subghz.write_buffer(0, buf).unwrap();
                        self.subghz.set_tx(Timeout::DISABLED).unwrap();
                        (State::Txing, Ok(LoraResponse::Txing))
                    },
                    Err(_) => {
                        (State::Idle, Err(LoraError::PhyError(Error::UnexpectedPhyEvent)))
                    },
                }
            },
            LoraEvent::RxRequest(config) => {
                defmt::info!("RxRequest: {}", config);
                //self.rfs.set_rx();
                let result = {
                    radio_config(&mut self.subghz, self.high_power);
                        // We have checked the frequency at creation time so we can unwrap safely
                        
                        self.subghz.set_rf_frequency(&RfFreq::from_frequency(433_175_000)).unwrap();//config.frequency))?;
    
                        self.subghz.set_lora_mod_params(
                            &LoRaModParams::new()
                            .set_sf(sf_transform(LoraSpreadingFactor::_12))
                            .set_cr(cr_transform(LoraCodingRate::_4_5))
                            .set_bw(bw_transform(LoraBandwidth::_125KHz))
                            .set_ldro_en(true)
                        ).unwrap();
    
                        self.subghz.set_lora_packet_params(
                            &LoRaPacketParams::new()
                            .set_crc_en(true)
                            .set_header_type(HeaderType::Variable)
                            .set_preamble_len(8)
                            .set_payload_len(255)
                            .set_invert_iq(true)
                        ).unwrap();
    
                        
                        const IRQ_CFG: CfgIrq = CfgIrq::new()
                            .irq_enable_all(Irq::RxDone)
                            .irq_enable_all(Irq::Timeout)
                            .irq_enable_all(Irq::PreambleDetected)
                            .irq_enable_all(Irq::HeaderErr)
                            .irq_enable_all(Irq::Err);
                            /*
                            .irq_enable_all(Irq::CadDetected)
                            .irq_enable_all(Irq::CadDone)
                            .irq_enable_all(Irq::PreambleDetected)
                            .irq_enable_all(Irq::HeaderValid)
                            .irq_enable_all(Irq::HeaderErr)
                            .irq_enable_all(Irq::Err);*/
                        self.subghz.set_irq_cfg(&IRQ_CFG).unwrap();
                        self.subghz.set_rx(Timeout::DISABLED)
                };
                match result {
                    Ok(_) => (State::Rxing, Ok(LoraResponse::Rxing)),
                    Err(_) => (State::Idle, Err(LoraError::PhyError(Error::UnexpectedPhyEvent))),
                }
            },
            LoraEvent::PhyEvent(_) => (State::Idle, Err(LoraError::PhyError(Error::UnexpectedPhyEvent))),
            LoraEvent::CancelRx => (State::Idle, Err(LoraError::CancelRxWhileIdle)),
        }
    }

    fn handle_event_from_txing(&mut self, event: LoraEvent<Self>
    ) -> (State, Result<LoraResponse<Self>, LoraError<Self>>) {
        match event {
            LoraEvent::PhyEvent(phyevent) => match phyevent {
                Event::Irq(_status, irq_status) => {
                    self.subghz.set_standby(StandbyClk::Rc).unwrap();
                    if irq_status & Irq::TxDone.mask() != 0 {
                        (State::Idle, Ok(LoraResponse::TxDone(0)))
                    } else if irq_status & Irq::Timeout.mask() != 0 {
                        (State::Idle, Err(LoraError::PhyError(Error::UnexpectedIrq(irq_status))))
                    } else { // Only TxDone or Timeout are expected while TXing
                        (State::Txing, Err(LoraError::PhyError(Error::UnexpectedIrq(irq_status))))
                    }
                }
            },
            LoraEvent::TxRequest(_, _) => (State::Txing, Err(LoraError::TxRequestDuringTx)),
            LoraEvent::RxRequest(_) => (State::Txing, Err(LoraError::RxRequestDuringTx)),
            LoraEvent::CancelRx => (State::Txing, Err(LoraError::CancelRxDuringTx)),
        }
    }

    fn handle_event_from_rxing(&mut self, event: LoraEvent<Self>
    ) -> (State, Result<LoraResponse<Self>, LoraError<Self>>) {
        match event {
            LoraEvent::PhyEvent(phyevent) => match phyevent {
                Event::Irq(status, irq_status) => {
                    defmt::info!("RxIng: Radio IRQ");
                    assert_eq!(status.mode().unwrap(), StatusMode::StandbyRc);
                    if (irq_status & Irq::RxDone.mask()) != 0 {
                        let (_rx_status, rx_len, _rx_ptr) = self.subghz.rx_buffer_status().unwrap();
                        let pkt_status = self.subghz.lora_packet_status().unwrap();
                        let rssi = pkt_status.rssi_pkt().to_integer();
                        let snr = pkt_status.snr_pkt().to_integer().try_into().unwrap_or(127);
                        defmt::info!("RX info: pkt_status: {}, rx_status: {}, rx_len: {}", pkt_status, _rx_status, rx_len);
                        self.subghz.read_buffer(0, &mut self.buffer_rx[..(rx_len as usize)]).unwrap();
                        self.buffer_len = rx_len;
                        // TODO Put radio to sleep?
                        (
                            State::Idle,
                            Ok(LoraResponse::RxDone(RxQuality::new(rssi, snr))),
                        )
                    } else if (irq_status & Irq::Timeout.mask()) != 0 {
                        (State::Idle, Err(LoraError::PhyError(Error::Timeout)))
                    } else if (irq_status & Irq::TxDone.mask()) != 0 {
                        (State::Idle, Err(LoraError::PhyError(Error::UnexpectedIrq(irq_status))))
                    } else {
                        (State::Rxing, Ok(LoraResponse::Rxing))
                    }
                }
            },
            LoraEvent::CancelRx => {
                defmt::error!("CancelRx while Rxing");
                self.subghz.set_standby(StandbyClk::Rc).unwrap();
                // TODO Put radio to sleep?
                (State::Idle, Ok(LoraResponse::Idle))
            }
            LoraEvent::TxRequest(_, _) => (State::Rxing, Err(LoraError::TxRequestDuringRx)),
            LoraEvent::RxRequest(_) => (State::Rxing, Err(LoraError::RxRequestDuringRx)),
        }
    }
}

impl PhyRxTx for LorawanRadio {
    type PhyEvent = Event;
    type PhyResponse = Response;
    type PhyError = Error;

    fn get_received_packet(&mut self) -> &mut [u8] {
        &mut self.buffer_rx[..(self.buffer_len as usize)]
    }

    fn get_mut_radio(&mut self) -> &mut Self {
        self
    }

    fn handle_event(
        &mut self,
        event: LoraEvent<Self>,
    ) -> Result<LoraResponse<Self>, LoraError<Self>> {
        let (new_state, response) = match &self.state {
            State::Idle => self.handle_event_from_idle(event),
            State::Txing => self.handle_event_from_txing(event),
            State::Rxing => self.handle_event_from_rxing(event),
        };
        self.state = new_state;
        response
    }
}

impl Timings for LorawanRadio {
    fn get_rx_window_offset_ms(&self) -> i32 {
        -200
    }
    fn get_rx_window_duration_ms(&self) -> u32 {
        800
    }
}

fn radio_config(subghz: &mut SubGhz<SgMiso, SgMosi>, high_power: bool) {
    subghz.set_standby(StandbyClk::Rc).unwrap();
    
    subghz.set_tcxo_mode(&TcxoMode::new()
        .set_txco_trim(TcxoTrim::Volts1pt7)
        .set_timeout(Timeout::from_millis_sat(40))).unwrap();
    subghz.set_regulator_mode(RegMode::Ldo).unwrap();

    subghz.calibrate_image(CalibrateImage::ISM_430_440).unwrap();

    subghz.set_buffer_base_address(0,0).unwrap();
    
    if high_power {
        subghz.set_pa_config(&PaConfig::new()
            .set_pa_duty_cycle(0x2)
            .set_hp_max(0x2)
            .set_pa(PaSel::Hp)).unwrap();
    } else {
        subghz.set_pa_config(&PaConfig::new()
            .set_pa_duty_cycle(0x1)
            .set_hp_max(0x0)
            .set_pa(PaSel::Lp)).unwrap();
    }

    subghz.set_pa_ocp(Ocp::Max140m).unwrap();
    if high_power {
        subghz.set_tx_params(&TxParams::new()
            .set_ramp_time(RampTime::Micros40)
            .set_power(0x16)).unwrap();   
    } else {
        subghz.set_tx_params(&TxParams::new()
            .set_ramp_time(RampTime::Micros40)
            .set_power(0x0A)).unwrap(); 
    }
    subghz.set_packet_type(PacketType::LoRa).unwrap();
    // Public means **any** LoRaWAN network
    // The distinction between public/private lorawan networks is not done through the SyncWord
    subghz.set_lora_sync_word(LoRaSyncWord::Public).unwrap();
}

fn sf_transform(item: LoraSpreadingFactor) -> SpreadingFactor {
    match item {
        LoraSpreadingFactor::_7  => SpreadingFactor::Sf7,
        LoraSpreadingFactor::_8  => SpreadingFactor::Sf8,
        LoraSpreadingFactor::_9  => SpreadingFactor::Sf9,
        LoraSpreadingFactor::_10 => SpreadingFactor::Sf10,
        LoraSpreadingFactor::_11 => SpreadingFactor::Sf11,
        LoraSpreadingFactor::_12 => SpreadingFactor::Sf12,
    }
}

fn cr_transform(item: LoraCodingRate) -> CodingRate {
    match item {
        LoraCodingRate::_4_5 => CodingRate::Cr45,
        LoraCodingRate::_4_6 => CodingRate::Cr46,
        LoraCodingRate::_4_7 => CodingRate::Cr47,
        LoraCodingRate::_4_8 => CodingRate::Cr48,
    }
}

fn bw_transform(item: LoraBandwidth) -> LoRaBandwidth {
    match item {
        LoraBandwidth::_125KHz => LoRaBandwidth::Bw125,
        LoraBandwidth::_250KHz => LoRaBandwidth::Bw250,
        LoraBandwidth::_500KHz => LoRaBandwidth::Bw500,
    }
}

/*
static void ComputeRxWindowParameters( void )
{
    // Compute Rx1 windows parameters
    RegionComputeRxWindowParameters( MacCtx.NvmCtx->Region,
                                     RegionApplyDrOffset( MacCtx.NvmCtx->Region,
                                                          MacCtx.NvmCtx->MacParams.DownlinkDwellTime,
                                                          MacCtx.NvmCtx->MacParams.ChannelsDatarate,
                                                          MacCtx.NvmCtx->MacParams.Rx1DrOffset ),
                                     MacCtx.NvmCtx->MacParams.MinRxSymbols,
                                     MacCtx.NvmCtx->MacParams.SystemMaxRxError,
                                     &MacCtx.RxWindow1Config );
    // Compute Rx2 windows parameters
    RegionComputeRxWindowParameters( MacCtx.NvmCtx->Region,
                                     MacCtx.NvmCtx->MacParams.Rx2Channel.Datarate,
                                     MacCtx.NvmCtx->MacParams.MinRxSymbols,
                                     MacCtx.NvmCtx->MacParams.SystemMaxRxError,
                                     &MacCtx.RxWindow2Config );

    // Default setup, in case the device joined
    MacCtx.RxWindow1Delay = MacCtx.NvmCtx->MacParams.ReceiveDelay1 + MacCtx.RxWindow1Config.WindowOffset;
    MacCtx.RxWindow2Delay = MacCtx.NvmCtx->MacParams.ReceiveDelay2 + MacCtx.RxWindow2Config.WindowOffset;

    if( MacCtx.NvmCtx->NetworkActivation == ACTIVATION_TYPE_NONE )
    {
        MacCtx.RxWindow1Delay = MacCtx.NvmCtx->MacParams.JoinAcceptDelay1 + MacCtx.RxWindow1Config.WindowOffset;
        MacCtx.RxWindow2Delay = MacCtx.NvmCtx->MacParams.JoinAcceptDelay2 + MacCtx.RxWindow2Config.WindowOffset;
    }
}
*/