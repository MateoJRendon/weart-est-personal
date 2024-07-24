use crate::periph_managers::SpiMngr;

pub struct DataMngr {
    curr_meas_data: [u8; 528],
    curr_data_unhandled: bool,
    next_flash_page: u16,
    pub curr_usb_data: [u8; 528], // POSSIBLE HAZARD: Read by USB interrupt
    pub data_valid_for_usb_send: bool, // POSSIBLE HAZARD: Read and modified by USB interrupt
    next_usb_flash_page: Option<u16>,
}

impl DataMngr {
    // IMPORTANT TODO: Must select the 512 bytes per page option as its not the default!
    // For IDFBCAMP/IDFBCAMQ, the max number of chip parameters to measure is 32 dig + 20 analog + 4
    // voltages + 2 temperatures.
    // Each page in the external flash is 256/512 bytes, which means if each measurement is an f32
    // then each measurement cycle with 352 total measurements will require 2/2.75 pages of flash to store
    // ALSO, consider monitoring stress parameters between measurements as well for posterity
    // If we measure 2 parameters every minute we will end up with 120 f32 values per hour, which is
    // 2/0.9375 pages of data
    // This results in a total of 4/3.6875 pages of data each hour, which with 4096/8192 pages
    // means we can fit test data from 682.6/2221.6 hours onto the external flash chip, yay!
    pub const fn new() -> DataMngr {
        DataMngr {
            // Manage test data for storage in onboard flash memory
            curr_meas_data: [0 as u8; 528],
            next_flash_page: 0,
            // Manage test data for sending to the computer over USB
            curr_usb_data: [0 as u8; 528],
            curr_data_unhandled: false,
            data_valid_for_usb_send: false,
            next_usb_flash_page: None,
        }
    }

    pub fn new_test_start(&mut self) {
        // Upon starting a new test we abandon the previous test data
        self.next_flash_page = 0;
        self.next_usb_flash_page = None;
        self.data_valid_for_usb_send = false;
        self.curr_data_unhandled = false;
    }

    pub fn submit_data(&mut self, spi: &mut SpiMngr, data: [u8; 528]) -> Result<(), &'static str> {
        // If the most recently submitted data hasn't yet been sent over USB and there's no existing
        // backlog to send over USB, that means we've just now fallen behind on transferring data so
        // set the backlog pointer to that recently submitted data
        if self.next_usb_flash_page.is_none() && self.curr_data_unhandled {
            let prev_page = DataMngr::dec_mem_page(self.next_flash_page);
            self.next_usb_flash_page = Some(prev_page);
        }

        // Update the current data packet and indicate that it is fresh (i.e. not sent to USB yet)
        self.curr_meas_data = data;
        self.curr_data_unhandled = true;

        // Store the data packet on the external flash
        spi.write_mem_page(self.next_flash_page, &self.curr_meas_data)?;
        self.next_flash_page = DataMngr::inc_mem_page(self.next_flash_page);
        Ok(())
    }

    // THIS IS THE ONLY NON-INTERRUPT CODE THAT'S ALLOWED TO MODIFY THE POSSIBLE HAZARD DATA FIELDS
    // (OUTSIDE OF FLASH DUMPING)
    pub fn try_prep_usb_data(&mut self, spi: &mut SpiMngr) -> Result<(), &'static str> {
        // If USB peripheral still hasn't handled the current data packet then nothing to do
        if !self.data_valid_for_usb_send {
            // Check if there's a backlog of data to send over USB
            if let Some(address) = self.next_usb_flash_page {
                // Load next data in queue from flash and set flag to indicate data is ready
                spi.read_mem_page(address, &mut self.curr_usb_data)?;
                // Check if we've caught up to the current point in the test
                let new_address = DataMngr::inc_mem_page(address);
                // Compared against last stored data as this data will still be in
                // curr_meas_data and so we can grab it from there instead of flash for performance
                if new_address == DataMngr::dec_mem_page(self.next_flash_page) {
                    self.next_usb_flash_page = None;
                } else {
                    self.next_usb_flash_page = Some(new_address);
                }
                // Set the data ready flag for the USB peripheral to see
                // THIS MUST BE DONE AFTER CHANGING CURR_USB_DATA
                self.data_valid_for_usb_send = true;

            // If no backlog, check if the current measurement data hasn't been sent yet
            } else if self.curr_data_unhandled {
                self.curr_usb_data = self.curr_meas_data;
                // Set the data ready flag for the USB peripheral to see
                // THIS MUST BE DONE AFTER CHANGING CURR_USB_DATA
                self.data_valid_for_usb_send = true;
                self.curr_data_unhandled = false;
            }
        }
        Ok(())
    }

    // Simplified flash read for getting flash data sent to the computer quickly
    pub fn flash_dump_prep(&mut self, spi: &mut SpiMngr, page: u16) -> Result<(), &'static str> {
        // Ensure we don't overwrite any existing valid data
        if !self.data_valid_for_usb_send {
            spi.read_mem_page(page, &mut self.curr_usb_data)?;
            self.data_valid_for_usb_send = true;
        }
        Ok(())
    }

    fn inc_mem_page(address: u16) -> u16 {
        if address == 8191 {
            0
        } else {
            address + 1
        }
    }

    fn dec_mem_page(address: u16) -> u16 {
        if address == 0 {
            8192
        } else {
            address - 1
        }
    }
}

pub struct DataAssembler {
    meas_buffer: [u8; 528],
    pub curr_pos: usize,
}

impl DataAssembler {
    pub fn new() -> DataAssembler {
        DataAssembler {
            meas_buffer: [0; 528],
            curr_pos: 0,
        }
    }

    pub fn reset(&mut self) {
        self.meas_buffer = [0; 528];
        self.curr_pos = 0;
    }

    pub fn append_meas(&mut self, meas: f32) {
        self.meas_buffer[self.curr_pos..(self.curr_pos + 4)].copy_from_slice(&meas.to_be_bytes());
        self.curr_pos += 4;
    }

    pub fn get_as_bytes(&mut self) -> [u8; 528] {
        self.meas_buffer
    }
}
