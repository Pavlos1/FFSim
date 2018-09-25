use std::io;
use std::io::{Read, Write};
use std::time::Duration;
use std::thread;
use std::mem::transmute;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use serial;
use serial::SerialPort;

use super::STOP_THREADS;
use super::flight_data::FLIGHT_DATA_SIZE;
use super::control_data::CONTROL_DATA_SIZE;
use super::BufferedFlightData;
use super::ControlData;
use super::BufferedControlData;

use super::FlightData;
use triple_buffer::{Input, Output};

fn ser_connect() -> io::Result<serial::SystemPort> {
    // FIXME: We probably want to be a bit more flexible
    let mut ser = serial::open(if cfg!(target_os = "windows") {
        "COM5"
    } else {
        "/dev/ttyUSB0"
    })?;

    // Loosely based on the example in
    // https://github.com/dcuddeback/serial-rs/tree/master/serial
    ser.reconfigure(&|settings| {
        settings.set_baud_rate(serial::BaudOther(4_000_000))?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    })?;

    //ser.set_timeout(Duration::from_millis(100))?;

    Ok(ser)
}

pub fn send_flight_data_thread(data_in_: Output<BufferedFlightData>, ser_: Arc<Mutex<Option<serial::SystemPort>>>) {
    let mut data_in = data_in_;
    let mut ser: Option<serial::SystemPort>;

    loop {
        if STOP_THREADS.load(Ordering::SeqCst) {
            break;
        }

        let guard = ser_.lock().unwrap();
        ser = guard.clone();
        drop(guard);

        let new_ser = match ser {
            Some(mut port) => {
                let data = FlightData::new(*data_in.read());
                let bytes: [u8; FLIGHT_DATA_SIZE] = unsafe { transmute(data) };
                match port.write_all(&bytes[..]) {
                    Ok(_) => Some(port),
                    Err(e) => {
                        println!("[FFSim] Lost serial connection: send, with error {:?}", e);
                        port.close();
                        None
                    },
                }
            }
            None => {
                match ser_connect() {
                    Ok(port) => {
                        println!("[FFSim] Got serial connection");
                        Some(port)
                    }
                    Err(e) => {
                        println!("[FFSim] Serial connection failed: send, with error {:?}", e);
                        None
                    },
                }
            }
        };
        ser = new_ser;

        let mut guard = ser_.lock().unwrap();
        *guard = ser.clone();
        drop(guard);

        thread::sleep(Duration::from_millis(20)); // 50Hz
    }
}

pub fn recv_control_data_thread(data_out_: Input<BufferedControlData>, ser_: Arc<Mutex<Option<serial::SystemPort>>>) {
    let mut data_out = data_out_;
    let mut ser: Option<serial::SystemPort>;

    let mut buf: [u8; CONTROL_DATA_SIZE] = [0; CONTROL_DATA_SIZE];
    let mut cursor: usize = 0;

    loop {
        if STOP_THREADS.load(Ordering::SeqCst) {
            break;
        }

        let guard = ser_.lock().unwrap();
        ser = guard.clone();
        drop(guard);

        match ser {
            Some(mut port) => {
                match port.read_exact(&mut buf[cursor..]) {
                    Ok(_) => {
                        // case 1: "SYNC" is at the start of the buffer, so we can
                        //         interpret the whole thing as a ControlData struct
                        if buf[..4] == *"SYNC".as_bytes() {
                            let cd: ControlData = unsafe { transmute(buf) };
                            if cd.verify() {
                                // Actually pass the control data on to the flightsim
                                data_out.write(BufferedControlData::from_external(cd));
                            } else {
                                println!("[FFSim] Bad checksum");
                            }
                            // In either case, we want to have an entirely fresh
                            // buffer the next time
                            cursor = 0;
                        }

                        // case 2: "SYNC" is a substring. Discard all bytes before the substring,
                        //         and move the rest up to make room for more input
                        else if let Some(pos) = buf.windows(4).position(|window|
                            *window == *"SYNC".as_bytes()) {
                            shift(&mut buf[..], pos);
                            cursor = CONTROL_DATA_SIZE - pos;
                        }

                        // case 3: "SYN" is at the end of the buf. The next input byte may well be
                        //         'C', so discard everything before "SYN" and move it to the front
                        else if buf[CONTROL_DATA_SIZE - 3..] == *"SYN".as_bytes() {
                            shift(&mut buf[..], CONTROL_DATA_SIZE - 3);
                            cursor = 3;
                        }
                        // The rest of the cases are fairly self-explanatory
                        else if buf[CONTROL_DATA_SIZE - 2..] == *"SY".as_bytes() {
                            shift(&mut buf[..], CONTROL_DATA_SIZE - 2);
                            cursor = 2;
                        } else if buf[CONTROL_DATA_SIZE - 1..] == *"S".as_bytes() {
                            shift(&mut buf[..], CONTROL_DATA_SIZE - 1);
                            cursor = 1;
                        } else {
                            cursor = 0;
                        }
                    }

                    Err(e) => {
                        println!("[FFSim] Lost serial connection: receive, with error {:?}", e);
                        cursor = 0; // unlikely that transmission will resume from the same point

                        let mut guard = ser_.lock().unwrap();
                        port.close();
                        *guard = None;
                        drop(guard);
                    }
                }
            }
            None => {
                thread::sleep(Duration::from_millis(200));
            }
        }
    }
}

fn shift<T: Copy>(arr: &mut [T], start_pos: usize) {
    let length = arr.len();
    for i in start_pos .. length {
        arr[i - start_pos] = arr[i];
    }
}

