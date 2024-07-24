from numpy import uint8
import struct
import pprint
from datetime import datetime

from io_handling import from_yaml


def format_stress_report(data, num_minutes: int, test_time: int, test_run: str):
    report = {'datatype': 'stress', 'testrun': test_run, 'submittedtime': datetime.utcnow(), 'time': test_time, 'values': []}
    for i in range(num_minutes):
        # Note that time is +30 as the stress parameter measurements are taken at the middle of each minute
        report['values'].append({'time': test_time + 30 + (i * 60), 'temp': data[i * 2], 'vdd': data[(i * 2) + 1]})
    return report


def format_measure_report(data, data_format, test_time: int, test_run: str, **extra_fields):
    report = {'datatype': 'measure', 'testrun': test_run, 'submittedtime': datetime.utcnow(), 'time': test_time, 'values': {}}
    for field in extra_fields:
        report[field] = extra_fields[field]
    for i, prm in enumerate(data_format):
        report['values'][prm] = data[i]
    return report


def load_test_info(name):
    test_defs = from_yaml('test_defs.yaml')
    test = test_defs[name]
    meas_defs = from_yaml('meas_defs.yaml')

    for step in test['step-order']:
        if 'what-to-meas' in test[step].keys() and type(test[step]['what-to-meas']) == str:
            test[test[step]['what-to-meas']] = meas_defs[test[step]['what-to-meas']]
    return test


def convert_test_for_transmit(name):
    # Each list of bytes can only be 63 bytes in length, as one byte is prepended to indicate the type of data transmitted and the buffer size is 64 bytes
    converted = {'testorder': [], 'teststeps': [], 'measorders': [], 'measconfigs': [], 'anasets': [], 'digsets': []}

    test_defs = from_yaml('test_defs.yaml')
    test = test_defs[name]

    meas_defs = from_yaml('meas_defs.yaml')

    # Generate the IDs mapping step names to unique byte-size numbers for the controller to use
    step_ids = {}
    id_inc = 0
    testorder = []
    for stepname in test['step-order']:
        if not stepname in step_ids:
            if not stepname in test.keys():
                raise Exception('Test step order contains undefined test steps')
            step_ids[stepname] = uint8(id_inc)
            id_inc += 1
        testorder.append(step_ids[stepname])
    if id_inc > 5000:
        raise Exception('Test step order too long! Max 5000 steps in a test')
    
    # Generate the IDs mapping measurement listings to unique byte-size numbers
    measlist_ids = {}
    measlists = {}
    id_inc = 0
    for step in step_ids:
        if 'what-to-meas' in test[step].keys():
            # Measure steps can either directly specify the list of measurements to take or refer to a predefined list
            meas_list = test[step]['what-to-meas']
            if type(meas_list) == list:
                # Every time we encounter a raw list of steps we check to see if the same list already exists
                match = False
                matching_id = None
                for ml in measlists:
                    match = True
                    for i, m in enumerate(ml):
                        if meas_list[i] != m:
                            match = False
                    if match:
                        matching_id = measlist_ids[ml]
                        break
                if match:
                    test[step]['what-to-meas'] = matching_id
                else:
                    measlist_ids[f"measlist{id_inc}"] = uint8(id_inc)
                    test[step]['what-to-meas'] = uint8(id_inc)
                    measlists[f"measlist{id_inc}"] = meas_list
                    id_inc += 1
            elif meas_list not in measlists.keys():
                measlists[meas_list] = meas_defs[meas_list]
                measlist_ids[meas_list] = uint8(id_inc)
                test[step]['what-to-meas'] = measlist_ids[meas_list]
                id_inc += 1
            else:
                test[step]['what-to-meas'] = measlist_ids[meas_list]
    if id_inc > 5:
        raise Exception('Too many measurement order listings! Max of five unique lists')
    for ml in measlists:
        if len(measlists[ml]) > 250:
            raise Exception('Measurement list too long! Max of 250 measurements per list')

    # Generate the IDs mapping measurement definitions to unique byte-size numbers for the controller
    meas_ids = {}
    id_inc = 0
    for ml in measlists:
        for i, m in enumerate(measlists[ml]):
            if not m in meas_ids.keys():
                meas_ids[m] = uint8(id_inc)
                id_inc += 1
            # At the same time, replace all the measurement names in the lists into their ids
            measlists[ml][i] = meas_ids[m]
    if id_inc > 255:
        raise Exception('Too many unique measurements configured! Max of 255')

    # Generate the IDs mapping analog control signal settings to unique byte codes
    ana_ctrl_ids = {}
    id_inc = 0
    for step in step_ids:
        if 'ana-ctrl' in test[step]:
            ctrl_name = test[step]['ana-ctrl']
            if not ctrl_name in ana_ctrl_ids:
                ana_ctrl_ids[ctrl_name] = uint8(id_inc)
                id_inc += 1
            # Also change the step definition reference to the byte code
            test[step]['ana-ctrl'] = ana_ctrl_ids[ctrl_name]
        else:
            # ID 10 represents the default fallback ctrl settings defined by the controller
            test[step]['ana-ctrl'] = uint8(10)
    if id_inc > 10:
        raise Exception('Too many unique analog control settings! Max of 10')

    # Do the exact same thing for the digital control settings
    dig_ctrl_ids = {}
    id_inc = 0
    for step in step_ids:
        if 'dig-ctrl' in test[step]:
            ctrl_name = test[step]['dig-ctrl']
            if not ctrl_name in dig_ctrl_ids:
                dig_ctrl_ids[ctrl_name] = uint8(id_inc)
                id_inc += 1
            # Also change the step definition reference to the byte code
            test[step]['dig-ctrl'] = dig_ctrl_ids[ctrl_name]
        else:
            # ID 10 represents the default fallback ctrl settings defined by the controller
            test[step]['dig-ctrl'] = uint8(10)
    if id_inc > 10:
        raise Exception('Too many unique digital control settings! Max of 10')

    ### Now construct the byte arrays ###

    # Convert the analog control configurations
    for cfg in ana_ctrl_ids:
        converted['anasets'].append([ana_ctrl_ids[cfg]] + list(struct.pack('>8f', *test[cfg])))
    # Convert the digital control configurations
    for cfg in dig_ctrl_ids:
        converted['digsets'].append([dig_ctrl_ids[cfg]] + test[cfg])
    # Convert measurement configurations
    meas_type_map = {'stress-voltage': 0x00, 'temperature': 0x01, 'digital': 0x02,
                     'frequency': 0x03, 'analog': 0x04, 'analog-slope': 0x05, 'analog-slope-naive': 0x06}
    for cfg in meas_ids:
        meas_type = meas_defs[cfg]['type']
        ctrls = 0b00_000_000
        if 'board-sel' in meas_defs[cfg]:
            ctrls += uint8(meas_defs[cfg]['board-sel']) << 3
        if 'chip-sel' in meas_defs[cfg]:
            ctrls += uint8(meas_defs[cfg]['chip-sel'])
        converted['measconfigs'].append([meas_ids[cfg], meas_type_map[meas_type], meas_defs[cfg]['channel'], ctrls])
    # Convert test steps
    step_type_map = {'stress': 0x00, 'measure': 0x01}
    for step in step_ids:
        step_type = 'stress' if 'duration' in test[step].keys() else 'measure'
        strs_floats = []
        strs_enabled_mask = 0b000_00000
        for i, cond in enumerate(['temp', 'vcore', 'vio', 'vcore2', 'vio2']):
            if cond in test[step]['conditions'].keys():
                strs_floats.append(test[step]['conditions'][cond])
                strs_enabled_mask += 0b1 << 4 - i
            else:
                strs_floats.append(0.0)
        strs_bytes = list(struct.pack('>5f', *strs_floats))
        type_specific = [0x00, 0x00, 0x00, test[step]['what-to-meas']] if step_type == 'measure' else list(test[step]['duration'].to_bytes(4, byteorder='big'))
        converted['teststeps'].append([step_ids[step], step_type_map[step_type], test[step]['dig-ctrl'], test[step]['ana-ctrl']]
                                      + type_specific + [strs_enabled_mask] + strs_bytes)

    # Convert the measurement orders into byte arrays
    # 64 bytes are [typecode, measorder-id, meas-1, ..., meas-n]
    for ml in measlists:
        i = 0
        while i <= len(measlists[ml]) - 62:
            converted['measorders'].append([measlist_ids[ml]] + measlists[ml][i:(i+62)])
            i += 62
        if len(measlists[ml]) % 62 == 0:
            # Need to add a termination code, 62 is the special case where the whole list fits into the first packet but not the terminator
            converted['measorders'].append([measlist_ids[ml]] + [0xff])
        else:
            converted['measorders'].append([measlist_ids[ml]] + measlists[ml][i:] + [0xff])

    # Convert the test order into byte arrays
    i = 0
    test_len = len(testorder)
    while i <= test_len - 63:
        converted['testorder'].append(testorder[i:(i + 63)])
        i += 63
    if test_len % 63 == 0:
        converted['testorder'].append([0xff])
    else:
        converted['testorder'].append(testorder[i:] + [0xff])
    # Provide the completed dictionary of formatted byte arrays to send
    return converted


if __name__ == '__main__':
    as_bytes = convert_test_for_transmit('short-test')
    pprint.pp(as_bytes)

