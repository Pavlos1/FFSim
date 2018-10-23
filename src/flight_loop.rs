use xplm::flight_loop::LoopState;
use xplm::data::{DataReadWrite, ArrayReadWrite};
use std::time::SystemTime;
use std::time::UNIX_EPOCH;
use std::thread;
use std::fs::File;
use std::io::Write;
use FFSim;
use PLUGIN;
use NUM_LATENCY_MEASUREMENTS;

pub fn flight_loop(_loop_state: &mut LoopState) {
    // For latency computations, we measure the _start_ time from
    // before we start reading the DataRefs (since that also
    // contributes to latency.)
    //
    // N.B. This is the start time of the _new_ packet we are about to send.
    //
    // N.B. Also this is the start time of our part fo the flight loop, so
    //      subtracting these values from different flight loops gives us how
    //      long it takes an _integer number of flight loops_ to complete (in theory).
    let new_start_time = SystemTime::now();

    // `PLUGIN` is a global created by `xplane_plugin!`
    // It *should* be safe to take an exclusive reference,
    // since X-Plane does not call us concurrently.
    let plugin : &mut FFSim = unsafe { &mut *PLUGIN };

    // Read from triple buffer and update controls
    let control = *plugin.incoming.read();
    plugin.rudder.set(control.rudder);
    plugin.left_aileron.set(control.left_aileron);
    plugin.right_aileron.set(control.right_aileron);
    plugin.elevator1.set(control.elevator);
    plugin.elevator2.set(control.elevator);

    // If the time is set to UNIX_EPOCH, it means we read uninitialized data
    // from the triple buffer---ignore it.
    //
    // If the time is one we measured just previously, that means we have multiple
    // inputs from the controller for the same output---we care about the _first_
    // response to the output, so ignore it.
    if (control.time != UNIX_EPOCH) && (control.time != plugin.last_time) {
        // At this point the data in `control` is written out to the
        // sim, so we measure the _end_ time here.
        //
        // N.B. This is the end time for the packet we sent _last time_.
        match control.time.elapsed() {
            Ok(dur) => {
                // num_latencies >= NUM_LATENCY_MEASUREMENTS means we have concluded the experiment
                // already. We don't do this check earlier b/c we want the loop
                // to take the same amount of time regardless of if the experiment is
                // running. (Optimizing compiler might have other ideas though.)
                if plugin.num_latencies < NUM_LATENCY_MEASUREMENTS as isize {
                    if plugin.num_latencies >= 0 {
                        plugin.latencies[plugin.num_latencies as usize] = dur;

                        // Start measuring the total time of the experiment; this will be used
                        // to determine the average refresh rate of the physics engine during
                        // the test.
                        if plugin.num_latencies == 0 {
                            plugin.time_start = new_start_time;
                        }
                        plugin.cycle_count += 1;
                    }
                    plugin.num_latencies += 1;
                    plugin.last_time = control.time;

                    if plugin.num_latencies == NUM_LATENCY_MEASUREMENTS as isize {
                        // End of experiment. Spawn a new thread to write data
                        // to a file.
                        let latencies = plugin.latencies;
                        let time_start = plugin.time_start;
                        let cycles = plugin.cycle_count;
                        thread::spawn(move|| {
                            match File::create("latencies.csv") {
                                Ok(mut out) => {
                                    out.write_all("latencies,refresh\n".as_bytes())
                                        .unwrap();
                                    for i in 0 .. latencies.len() {
                                        out.write_all(format!("{}",
                                                              latencies[i].as_secs() * 1_000_000_000
                                                                  + latencies[i].subsec_nanos() as u64)
                                            .as_bytes()).unwrap();

                                        // write physics engine refresh rate into first row
                                        if i == 0 {
                                            let time_diff
                                                = new_start_time.duration_since(time_start)
                                                .unwrap();

                                            let time_diff_ns = (time_diff.as_secs()
                                                * 1_000_000_000 + time_diff.subsec_nanos() as u64)
                                                as f64;

                                            let adjusted_cycles = (cycles * 1_000_000_000)
                                                as f64;

                                            out.write_all(format!(",{}",
                                                                  adjusted_cycles/time_diff_ns)
                                                .as_bytes()).unwrap();
                                        }
                                        out.write_all("\n".as_bytes()).unwrap();

                                    }
                                    println!("[FFSim] Successfully wrote latencies");
                                },
                                Err(e) => {
                                    println!("[FFSim] Couldn't open file for writing latencies: {:?}", e);
                                }
                            }
                        });
                    }
                }
            },
            Err(e) => {
                println!("[FFSim] Did the clock change under us? e={:?}", e);
            }
        }
    }

    // Throttle is a bit trickier b/c it's an array,
    // but we only have one engine so we only set the
    // first element.
    let mut throttle_buf = [0.0; 8];
    throttle_buf[0] = control.throttle;
    plugin.throttle.set(&mut throttle_buf);

    // Write flight data into triple buffer
    let flight_data = plugin.get_data(new_start_time);
    plugin.outgoing.write(flight_data);
}