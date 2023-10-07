use stm32f7::stm32f7x2::*;
use heapless::{Deque, Vec};
use heapless::spsc::Producer;
use crate::app::Config;

use crate::app::APB2_CLOCK;

pub struct Serial {
    pub usart: USART1,
    pub send_queue: Deque<u8, 2048>,
}

impl Serial {
    pub fn new(usart: USART1) -> Serial {
        usart.brr.write(|w| { w.brr().bits((APB2_CLOCK*2*8/16/230400) as u16)});
        usart.cr1.write(|w| { w.te().enabled().re().enabled().ue().enabled().rxneie().enabled() });
        Serial {
            send_queue: Deque::new(),
            usart
        }
    }
    pub fn write_queued(&mut self, b: &[u8]) {
        for c in b {
            let _ = self.send_queue.push_back(*c);
        }
        self.usart.cr1.modify(|_,w| { w.txeie().enabled() });
    }
    fn read_byte(&self) -> u8 {
        return self.usart.rdr.read().bits() as u8
    }
}

pub struct SerialProtocol {
    pub serial: Serial,
    raw_message: Vec<u8, 4096>,
}

#[allow(dead_code)]
impl SerialProtocol {
    pub fn new(serial: Serial) -> SerialProtocol {
        SerialProtocol {
            serial,
            raw_message: Vec::new(),
        }
    }
    pub fn process_byte(&mut self, config_queue_in: &mut Producer<'static, Config, 2>) {
        let b = self.serial.read_byte();
        self.serial.write_queued(&[b]);
        if b == b'\n' {
            self.process_message(config_queue_in);
        } else {
            let _ = self.raw_message.push(b);
        }
    }
    fn process_message(&mut self, config_queue_in: &mut Producer<'static, Config, 2>) {
        let parsed_config: Result<(Config, _), serde_json_core::de::Error> = serde_json_core::from_slice(&self.raw_message);
        if let Ok((config, _)) = parsed_config {
            let _ = config_queue_in.enqueue(config);
        }
        self.raw_message.clear();
    }
}
