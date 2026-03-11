//! 配置编解码：负责配置头、版本与校验。

use super::config::ConfigError;
use super::profile::{CorrectionParams, SERVO_COUNT};

#[repr(C)]
#[derive(Clone, Copy)]
struct ConfigHeader {
    magic: [u8; 4],
    version: u16,
    count: u16,
    payload_len: u16,
    checksum: u16,
}

pub struct ConfigBlob;

impl ConfigBlob {
    pub const MAGIC: [u8; 4] = *b"SRVO";
    pub const VERSION: u16 = 1;
    pub const PARAM_SIZE: usize = 8;
    pub const HEADER_SIZE: usize = 12;
    pub const PAYLOAD_SIZE: usize = SERVO_COUNT * Self::PARAM_SIZE;
    pub const MAX_SIZE: usize = Self::HEADER_SIZE + Self::PAYLOAD_SIZE;
}

pub enum LoadOutcome {
    Empty,
    Loaded([CorrectionParams; SERVO_COUNT]),
}

pub struct ConfigCodec;

impl ConfigCodec {
    pub fn encode(
        corrections: &[CorrectionParams; SERVO_COUNT],
        out: &mut [u8],
    ) -> Result<usize, ConfigError> {
        if out.len() < ConfigBlob::MAX_SIZE {
            return Err(ConfigError::InvalidDataLength);
        }

        let payload_start = ConfigBlob::HEADER_SIZE;
        for (i, params) in corrections.iter().enumerate() {
            let offset = payload_start + i * ConfigBlob::PARAM_SIZE;
            out[offset..offset + ConfigBlob::PARAM_SIZE].copy_from_slice(&params.to_bytes());
        }

        let payload_end = payload_start + ConfigBlob::PAYLOAD_SIZE;
        let checksum = crc16(&out[payload_start..payload_end]);
        let header = ConfigHeader {
            magic: ConfigBlob::MAGIC,
            version: ConfigBlob::VERSION,
            count: SERVO_COUNT as u16,
            payload_len: ConfigBlob::PAYLOAD_SIZE as u16,
            checksum,
        };
        write_header(&header, out)?;
        Ok(ConfigBlob::MAX_SIZE)
    }

    pub fn decode(input: &[u8]) -> Result<LoadOutcome, ConfigError> {
        if input.len() < ConfigBlob::MAX_SIZE {
            return Err(ConfigError::InvalidDataLength);
        }

        // 默认空介质场景：全 0 或全 0xFF
        let head = &input[..ConfigBlob::HEADER_SIZE];
        if head.iter().all(|&b| b == 0) || head.iter().all(|&b| b == 0xFF) {
            return Ok(LoadOutcome::Empty);
        }

        let header = read_header(input)?;
        if header.magic != ConfigBlob::MAGIC {
            return Err(ConfigError::InvalidHeader);
        }
        if header.version != ConfigBlob::VERSION {
            return Err(ConfigError::InvalidVersion);
        }
        if header.count as usize != SERVO_COUNT {
            return Err(ConfigError::InvalidDataLength);
        }
        if header.payload_len as usize != ConfigBlob::PAYLOAD_SIZE {
            return Err(ConfigError::InvalidDataLength);
        }

        let payload = &input[ConfigBlob::HEADER_SIZE..ConfigBlob::MAX_SIZE];
        if crc16(payload) != header.checksum {
            return Err(ConfigError::InvalidChecksum);
        }

        let mut corrections = [CorrectionParams::default(); SERVO_COUNT];
        for (i, corr) in corrections.iter_mut().enumerate() {
            let offset = i * ConfigBlob::PARAM_SIZE;
            *corr = CorrectionParams::from_bytes(&payload[offset..offset + ConfigBlob::PARAM_SIZE])?;
        }

        Ok(LoadOutcome::Loaded(corrections))
    }
}

fn write_header(header: &ConfigHeader, out: &mut [u8]) -> Result<(), ConfigError> {
    if out.len() < ConfigBlob::HEADER_SIZE {
        return Err(ConfigError::InvalidDataLength);
    }

    out[0..4].copy_from_slice(&header.magic);
    out[4..6].copy_from_slice(&header.version.to_le_bytes());
    out[6..8].copy_from_slice(&header.count.to_le_bytes());
    out[8..10].copy_from_slice(&header.payload_len.to_le_bytes());
    out[10..12].copy_from_slice(&header.checksum.to_le_bytes());
    Ok(())
}

fn read_header(input: &[u8]) -> Result<ConfigHeader, ConfigError> {
    if input.len() < ConfigBlob::HEADER_SIZE {
        return Err(ConfigError::InvalidDataLength);
    }

    Ok(ConfigHeader {
        magic: [input[0], input[1], input[2], input[3]],
        version: u16::from_le_bytes([input[4], input[5]]),
        count: u16::from_le_bytes([input[6], input[7]]),
        payload_len: u16::from_le_bytes([input[8], input[9]]),
        checksum: u16::from_le_bytes([input[10], input[11]]),
    })
}

fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            let lsb = crc & 1;
            crc >>= 1;
            if lsb != 0 {
                crc ^= 0xA001;
            }
        }
    }
    crc
}
