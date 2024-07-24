from multiprocessing import Process
from progress.bar import Bar as ProgressBar

import time
from datetime import datetime
import numpy as np
import struct
import csv
import usb.core

from connection_handlers import *
from io_handling import *
from test_format import *

# This time is used to sleep the driver script to give the controller time to handle inputs
# Normally this can be quite small, but should be increased to a second or more when testing using
# semihosting as printout from the controller through the GDB debugger is extremely slow and prevents USB handling
WAIT_TIME = 0.5

### Core setup and loop code ###
def main():
    print('Starting up driver...')
    save_file = None
    send_to_db_data = []

    test_name = None
    test_info = None
    curr_step_index = 10000
    step_packet_index = 0
    test_is_running = False
    strs_elapsed = 0

    kb = KeyboardIn()
    # Ensure email is configured correctly
    #em_addr, em_pass = get_email_creds()
    #setup_email(em_addr, em_pass)
    em_srv = EmailServerLink()
    # Connect to MongoDB
    dataset = login_to_database()

    # FIXME: NEED TO STORE ELAPSED TIME IN RECOVERY FILE
    # Potentially recover test if a recovery file exists
    test_name, curr_step_index, save_file = check_recovery_file(kb)
    if test_name is not None:
        test_info = load_test_info(test_name)

    # Connect to the weartest controller
    dev = wait_for_usb_connect(initial_notice=True)

    # FIXME: Add try block for this
    cfg = dev.get_active_configuration()
    if cfg is None:
        dev.set_configuration()
        cfg = dev.get_active_configuration()
    ctrl_interface = cfg[(1, 0)]
    data_interface = cfg[(3, 0)]
    # TODO: Swap out hardcoded endpoint addresses in main loop with these endpoint objects
    ctrl_cmd = usb.util.find_descriptor(ctrl_interface, bEndpointAddress=0x01)
    dev_msg = usb.util.find_descriptor(ctrl_interface, bEndpointAddress=0x82)
    test_cfg = usb.util.find_descriptor(data_interface, bEndpointAddress=0x02)
    test_data = usb.util.find_descriptor(data_interface, bEndpointAddress=0x84)

    # Begin the core execution loop
    while True:
        ### Task 1: Poll USB to trigger the USB interrupt frequently
        time.sleep(WAIT_TIME)
        try:
            dev.write(0x01, chr(0x00))
        except usb.core.USBError as e:
            if 'Resource busy' in str(e):
                print('USB channel busy')
                continue
            print(f"USB communication failure: {e}")
            dev = usb_conn_lost_wrapper(e, em_srv)

        ### Task 2: Check for new user commands for the test controller
        if kb.new_event():
            cmd = kb.getkey()
            cmd_byte = None
            match str(cmd):
                # Dump flash data
                case 'd':
                    if not test_is_running:
                        if double_check_with_user(kb, "Are you sure you want to initiate a flash dump?"):
                            # Dump flash in separate control loop
                            flash_dump(dev, em_srv)
                # Load test onto board
                case 'l':
                    if not test_is_running:
                        if double_check_with_user(kb, "Are you sure you want to load a new test config?"):
                            test_name = transmit_test(dev, em_srv)
                            test_info = load_test_info(test_name)
                # Start test
                case 'g':
                    if test_info is not None and not test_is_running:
                        if double_check_with_user(kb, "Are you sure you want to start the test?"):
                            cmd_byte = chr(0x01)
                            # Setup the data and recovery files for the new test
                            curr_time = datetime.utcnow()
                            save_file = f"{test_name}-{curr_time.date()}.dat"
                            append_to_save_file(save_file, "\nNEW TEST START\n")
                            write_recovery_file(test_name, curr_step_index, save_file)
                            curr_step_index = 0
                            step_packet_index = 0
                            test_is_running = True
                            strs_elapsed = 0
                # Stop test
                case 's':
                    if test_is_running:
                        if double_check_with_user(kb, "Are you absolutely sure you want to stop the test?"):
                            cmd_byte = chr(0x02)
                            curr_step_index = 10000
                            test_is_running = False
                # Pause test
                case 'p':
                    if test_is_running:
                        if double_check_with_user(kb, "Are you absolutely sure you want to pause the test?"):
                            cmd_byte = chr(0x03)
                # Terminate this program safely
                case 'q':
                    if double_check_with_user(kb, "Are you absolutely sure you want to terminate this program?"):
                        exit()
                case _:
                    print('Invalid command input')
            if cmd_byte is not None:
                try:
                    dev.write(0x01, cmd_byte)
                    print('Command sent')
                except usb.core.USBError as e:
                    if 'Resource busy' in str(e):
                        print('USB channel busy')
                        continue
                    dev = usb_conn_lost_wrapper(e, em_srv)

        ### Task 3: Check for status messages from the test controller
        try:
            ctrl_msg = dev.read(0x82, 64, 500)
            # These zero length checks are needed due to how USB bulk transfers are terminated. The last packet in a bulk transfer cannot
            # be the max packet size (I don't know why), and so since the weartest controller always sends the max size a zero-size packet
            # will follow to indicate that the transfer is complete
            if len(ctrl_msg) > 0:
                ctrl_msg = bytearray(np.trim_zeros(ctrl_msg)).decode('utf-8')
                print(f"Device says: {ctrl_msg}")
    
                # FAULT HANDLING: Send emails to user under two conditions:
                # 1. Weartest controller sends a program message indicating something unusual or wrong, send that message via email
                # 2. Communication is lost with the weartest controller
                if ctrl_msg[:4] == 'CRIT' or ctrl_msg[:4] == 'WARN':
                    Process(target=em_srv.build_and_send_email, args=(ctrl_msg,)).start()
    
                if save_file:
                    append_to_save_file(save_file, f"Device says: {ctrl_msg}\n")
        except (usb.core.USBError, usb.core.USBTimeoutError) as e:
            #print('Status message read fail')
            # Unless it's a timeout error indicating no data yet, need to fall back to attempting connection
            if type(e) == usb.core.USBError and not 'timeout error' in str(e):
                if 'Resource busy' in str(e):
                    print('USB channel busy')
                    continue
                dev = usb_conn_lost_wrapper(e, em_srv)

        ### Task 4: Check for bulk measurement data from the test controller
        if test_is_running:
            try:
                # 0x82 is the bulk data endpoint for receiving data
                data_bytes = dev.read(0x84, 528, 500)
                if len(data_bytes) > 0:
                    data_buf = struct.unpack(f">{int(len(data_bytes) / 4)}f", data_bytes.tobytes())
                    print(f"Received data packet!")

                    # The test controller will send data as float arrays, the test information maps these floats to the parameters
                    step_name = test_info['step-order'][curr_step_index]
                    step_cfg = test_info[step_name]
                    if 'duration' in step_cfg.keys():
                        formatted_data = format_stress_report(data_buf, int(step_cfg['duration'] / 60), strs_elapsed, test_name)
                        strs_elapsed += step_cfg['duration']
                        curr_step_index += 1
                    else:
                        meas_order = step_cfg['what-to-meas']
                        if type(meas_order) == str:
                            meas_order = test_info[meas_order]
                        # Some steps may have more measurements than can fit in a single data packet
                        if len(meas_order) > 132:
                            first_item = step_packet_index * 132
                            if first_item + 132 >= len(meas_order):
                                formatted_data = format_measure_report(data_buf, meas_order[first_item:], strs_elapsed, test_name)
                                step_packet_index = 0
                                curr_step_index += 1
                            else:
                                formatted_data = format_measure_report(data_buf, meas_order[first_item:(first_item + 132)], strs_elapsed, test_name)
                                step_packet_index += 1

                        else:
                            formatted_data = format_measure_report(data_buf, meas_order, strs_elapsed, test_name)
                            step_packet_index = 0
                            curr_step_index += 1

                    if save_file:
                        append_to_save_file(save_file, f"{str(formatted_data)}\n")
                    unsent_data = try_database_upload(dataset, formatted_data, True, em_srv)
                    if unsent_data is not None:
                        send_to_db_data.append(unsent_data)
                        print(f"Error uploading data from test step {curr_step_index} to database.")

                    write_recovery_file(test_name, curr_step_index, save_file)

                    # If we've finished the test according to the configuration, clean up
                    if curr_step_index >= len(test_info['step-order']):
                        print('Test finished!')
                        if save_file:
                            append_to_save_file(save_file, "TEST COMPLETE\n")
                        # Email notification of test completion so the user knows to clean up the test
                        if len(send_to_db_data) == 0:
                            mail_msg = 'Test completed successfully! All data was uploaded to database successfully.'
                        else:
                            mail_msg = 'Test completed successfully! Some data has still not been uploaded to database.'
                            print('Some data still waiting for database upload')
                        Process(target=em_srv.build_and_send_email, args=(mail_msg,)).start()
                        # No longer need the test position recovery file
                        save_file = None
                        test_is_running = False
                        rm_recovery_file()
            except (usb.core.USBError, usb.core.USBTimeoutError) as e:
                if 'Resource busy' in str(e):
                    print('USB channel busy')
                    continue
                if type(e) == usb.core.USBError and not 'timeout error' in str(e):
                    dev = usb_conn_lost_wrapper(e, em_srv)

        ### Task 5: If the test is over and some of the test data hasn't been sent to the database, retry now
        if not test_is_running and len(send_to_db_data) > 0:
            unsent_data = try_database_upload(dataset, send_to_db_data[0], False, em_srv)
            if unsent_data is None:
                send_to_db_data.pop(0)
            # If this is the last data packet that needed to be uploaded and we reach this line of code then finished
            if len(send_to_db_data) == 1:
                print('All data successfully uploaded to database')
                mail_msg = 'All data has now been uploaded to database successfully.'
                Process(target=em_srv.build_and_send_email, args=(mail_msg,)).start()


FLASH_DUMP_CSV = 'data/flash_dump.csv'

def flash_dump(dev, em_srv):
    try:
        dev.write(0x01, chr(0x04))
        print('Initiating dump')
    except usb.core.USBError as e:
        dev = usb_conn_lost_wrapper(e, em_srv)

    with open(FLASH_DUMP_CSV, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=',')
        i = 0
        dump_bar = ProgressBar('Dumping Flash', max=8192)
        while i < 8192:
            # Delay and poll device
            time.sleep(WAIT_TIME)
            try:
                dev.write(0x01, chr(0x00))
            except usb.core.USBError as e:
                dev = usb_conn_lost_wrapper(e, em_srv)
            # Check for next flash packet
            try:
                data_bytes = dev.read(0x84, 528, 500)
                if len(data_bytes) > 0:
                    data_buf = struct.unpack(f">{int(len(data_bytes) / 4)}f", data_bytes.tobytes())
                    writer.writerow(data_buf)
                    i += 1
                    dump_bar.next()
            except (usb.core.USBError, usb.core.USBTimeoutError) as e:
                if type(e) == usb.core.USBError and not 'timeout error' in str(e):
                    dev = usb_conn_lost_wrapper(e, em_srv)
        dump_bar.finish()
        print("Flash dump completed! Returning to main program loop")


def transmit_test(dev, em_srv):
    # Ask the user what test configuration they wish to load onto the test controller
    available_tests = get_defined_tests_list()
    valid_test = False
    test_to_load = None
    while not valid_test:
        test_to_load = input('Type the name of the test to load onto the test controller, type "abort" to cancel:')
        if test_to_load in available_tests:
            valid_test = True
        elif test_to_load == "abort":
            return
        else:
            print(f"Test {test_to_load} is not defined, please input a test name that appears in test_def.json")
    print('\nLoading test...')

    # Command the test controller to enter the test load state
    try:
        dev.write(0x01, chr(0x05))
    except usb.core.USBError as e:
        if 'Resource busy' in str(e):
            print('USB channel busy')
        dev = usb_conn_lost_wrapper(e, em_srv)

    # Convert the requested test into bytes form for transmission
    test_def_bytes = convert_test_for_transmit(test_to_load)
    time.sleep(WAIT_TIME)

    # Now we loop to transfer all the test data to the controller, checking for warnings as we do so
    code_map = {0x30: 'digsets', 0x20: 'anasets', 0x10: 'measconfigs', 0x11: 'measorders', 0x40: 'teststeps', 0x41: 'testorder'}
    for code in code_map.keys():
        for item in test_def_bytes[code_map[code]]:
            # Send the next set of bytes corresponding to a test configuration item
            try:
                dev.write(0x02, [code] + item)
            except usb.core.USBError as e:
                dev = usb_conn_lost_wrapper(e, em_srv)
            time.sleep(WAIT_TIME)

            # Check for status messages from the controller regarding the test being loaded
            try:
                ctrl_msg = dev.read(0x82, 64, 500)
                if len(ctrl_msg) > 0:
                    ctrl_msg = bytearray(np.trim_zeros(ctrl_msg)).decode('utf-8')
                    print(f"Device says: {ctrl_msg}")
            except (usb.core.USBError, usb.core.USBTimeoutError) as e:
                if type(e) == usb.core.USBError and not 'timeout error' in str(e):
                    dev = usb_conn_lost_wrapper(e, em_srv)
            time.sleep(WAIT_TIME)

    # Now transmit the code indicating that the test is fully loaded
    try:
        dev.write(0x02, [0xff])
    except usb.core.USBError as e:
        dev = usb_conn_lost_wrapper(e, em_srv)
    time.sleep(WAIT_TIME)

    print('Test loaded!')
    return test_to_load


if __name__ == '__main__':
    main()

